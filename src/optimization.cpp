#include <string>
#include <algorithm>

#include <optimization.hpp>

bool compareAreaChange(const update_node& a, const update_node& b)
{
    return a.area_change > b.area_change;
}



void optimization::local_search(void) {

    double area_diff;
    do {
        Polygon curr_poly;
        for (auto it = this->pl_points.begin(); it != this->pl_points.end(); ++it) curr_poly.push_back(*it);
        double curr_area = std::abs(curr_poly.area());

        //vector of all possible updates
        std::vector<update_node> updates;

        for (auto it = this->poly_line.begin(); it != this->poly_line.end(); ++it) {
            for (int i = 1; i <= this->L; i++) {
                //loop through all i-legth vertices
                for (size_t j = 0; j < this->poly_line.size(); j += i) {
                    // check if switching it and poly_line[j] gives optimized and simple polygon
                    std::vector<Segment> V;
                    auto it2 = this->poly_line.begin() + j;
                    for ( int z = 0; z < L; z++) {
                        V.push_back(*it2);
                        it2++;
                    }
                    std::vector<Point> temp_points = this->replace_edges( *it, V);

                    Polygon temp_poly;
                    for (auto it3 = temp_points.begin(); it3 != temp_points.end(); ++it3) temp_poly.push_back(*it3);

                    if (!temp_poly.is_simple()) continue;


                    if (!this->opt.compare("-max")) {
                        double temp_area= std::abs(temp_poly.area());
                        

                        double diff = temp_area - curr_area;
                        
                        if (diff <= 0) continue;

                        else {
                            update_node update;
                            update.e = *it;
                            update.V = V;
                            update.area_change = diff;
                            updates.push_back(update);
                        }
                    }

                    else if (!this->opt.compare("-min")) {
                        double temp_area= std::abs(temp_poly.area());

                        double diff = curr_area - temp_area;
                        
                        if (diff <= 0) continue;

                        else {
                            update_node update;
                            update.e = *it;
                            update.V = V;
                            update.area_change = diff;
                            updates.push_back(update);
                        }
                    }
                }

            }
        }        
        // sort update list based on area difference
        std::sort(updates.begin(),updates.end(),compareAreaChange);
        // loop on update list
        for (auto it = updates.begin(); it != updates.end(); ++it) {
            std::vector<Point> temp_points = this->replace_edges(it->e, it->V);

            Polygon temp_poly;
            for (auto it2 = temp_points.begin(); it2 != temp_points.end(); ++it2) temp_poly.push_back(*it2);

            if (!temp_poly.is_simple()) continue;

            if (!this->opt.compare("-max")) {
                double temp_area= std::abs(temp_poly.area());
                double diff = temp_area - curr_area;

                if (diff <= 0) continue;
                else this->pl_points = temp_points;
            }

            if (!this->opt.compare("-min")) {
                double temp_area= std::abs(temp_poly.area());
                double diff = curr_area - temp_area;

                if (diff <= 0) continue;
                else this->pl_points = temp_points;
            }
        }

        Polygon updated_poly;
        for (auto it = this->pl_points.begin(); it != this->pl_points.end(); ++it) updated_poly.push_back(*it);
        double updated_area = std::abs(updated_poly.area());

        if (!this->opt.compare("-max")) area_diff = updated_area - curr_area;
        else if (!this->opt.compare("-max")) area_diff = curr_area - updated_area;

        std::cout << "CURR AREA " << curr_area << std::endl;
        std::cout << "UPDATED AREA " << updated_area << std::endl;
        std::cout << "AREA DIFF " << area_diff << std::endl;
        this->poly_line = this->get_segment(this->pl_points);

    } while (area_diff >= this->threshold);

    
}

void optimization::simulated_annealing_local(void) {
    double T = 1.0;

    while(T >= 0) {
        // when n > 1000 and we get subdivision from cmd we use subdivision on a new function 
        // for the division and the global steps and we continue here with the local steps

        // transition step global or local (we transition  all points in a loop?)
        // global is the same step as in local search

        // check if is simple

        // check if optimizes poly 

        // if not check for metropolis criterion

        // update poly_line and pl_points

        // update T: T = T - 1/L

    }

}

void optimization::simulated_annealing_global(void) {
    double T = 1.0;

    while(T >= 0) {
        // when n > 1000 and we get subdivision from cmd we use subdivision on a new function 
        // for the division and the global steps and we continue here with the local steps

        // transition step global or local (we transition  all points in a loop?)
        // global is the same step as in local search

        // check if is simple

        // check if optimizes poly 

        // if not check for metropolis criterion

        // update poly_line and pl_points

        // update T: T = T - 1/L

    }

}

void optimization::simulated_annealing_subdivisions(void) {
    double T = 1.0;

    while(T >= 0) {
        // when n > 1000 and we get subdivision from cmd we use subdivision on a new function 
        // for the division and the global steps and we continue here with the local steps

        // transition step global or local (we transition  all points in a loop?)
        // global is the same step as in local search

        // check if is simple

        // check if optimizes poly 

        // if not check for metropolis criterion

        // update poly_line and pl_points

        // update T: T = T - 1/L

    }

}

std::vector<Point> optimization::replace_edges(Segment e, std::vector<Segment> V) {
    std::vector<Point> temp_points = this->pl_points;
    // insert points to the right index
    auto index = std::find(temp_points.begin(),temp_points.end(),e.target());
    for (auto it = V.begin(); it != V.end(); it++)  {
        temp_points.insert(index, it->source());
        index = std::find(temp_points.begin(),temp_points.end(),e.target());
    }
    index = std::find(temp_points.begin(),temp_points.end(),e.target());
    auto final_edge = V.end();
    temp_points.insert(index, final_edge->target());

    // delete points
    for (auto it = V.begin(); it != V.end(); it++) {
        index = std::find(temp_points.begin(),temp_points.end(),it->source());
        temp_points.erase(index);
    }
    final_edge = V.end();
    index = std::find(temp_points.begin(),temp_points.end(),final_edge->target());
    temp_points.erase(index);
    
    return temp_points;
}

std::vector<Segment> optimization::get_segment(std::vector<Point> points) {
    try {
        std::vector<Segment> seg;

        int i = 0;
        while(i != (points.size() - 1)) {
            seg.push_back(Segment(points[i], points[i+1]));
            i++;
        }

        seg.push_back(Segment(points[points.size() - 1], points[0]));

        return seg;
    } catch (...) {
        throw;
    }
}

optimization::optimization(std::vector<Point> pl_points,std::vector<Segment> poly_line, std::string alg, std::string L, std::string opt, std::string alg_param, std::string out_file)
    :out_file(out_file), pl_points(pl_points),poly_line(poly_line), opt(opt) {
        try {
            this->L = std::stoi(L);
            if (!alg.compare("local_search")) {
                this->threshold = std::stod(alg_param);
                this->local_search();
            }
            else if (!alg.compare("simulated_annealing")) {
                this->annealing = alg_param;
                if (!alg_param.compare("local")) this->simulated_annealing_local();
                else if (!alg_param.compare("global")) this->simulated_annealing_global();
                else if (!alg_param.compare("subdivision")) this->simulated_annealing_subdivision();
                else throw std::invalid_argument("\'Annealing\' must be \'local\', \'global\' or \'subdivision\'");
            }
            else throw std::invalid_argument("\'Algorithm\' must be \'local_search\' or \'simulated_annealing\'");
        } catch (...) {
            throw;
        }
    }
