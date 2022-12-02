#include <optimization.hpp>

#include <string>

void optimization::local_search(void) {

    do {
        for (auto it = this->poly_line.begin(); it != this->poly_line.end(); ++it) {
            for (int i = 1; i <= this->L; i++) {
                //vector of all possible updates
                std::vector<update_node> updates;
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
                    for (auto it = temp_points.begin(); it != temp_points.end(); ++it) temp_poly.push_back(*it);
                    Polygon curr_poly;
                    for (auto it = this->pl_points.begin(); it != this->pl_points.end(); ++it) temp_poly.push_back(*it);

                    if (!temp_poly.is_simple()) continue;

                    if (!area.compare("-max")) {
                        int temp_area= std::abs(temp_poly.area());
                        int curr_area = std::abs(curr_poly.area());

                        int diff = temp_area - curr_area;
                        
                        if (diff <= 0) continue;

                        else {
                            update_node update;
                            update.e = *it;
                            update.V = V;
                            update.area_change = diff;
                            updates.push_back(update);
                        }
                    }

                    else if (!area.compare("-min")) {
                        int temp_area= std::abs(temp_poly.area());
                        int curr_area = std::abs(curr_poly.area());

                        int diff = curr_area - temp_area;
                        
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
        // get curr area
        // sort update list based on area difference
        // loop on update list
        // check if updating poly line satisfies the conditions as before
        // if yes update poly line

        // calculate area diff between the two polygon
        int area_diff;
    } while (area_diff <= this->threshold);
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

optimization::optimization(std::vector<Point> pl_points, std::vector<Segment>poly_line, std::string alg, std::string L, std::string area, std::string alg_param, std::string out_file)
    :out_file(out_file), pl_points(pl_points), poly_line(poly_line), area(area) {
        try {
            this->L = std::stoi(L);
            if (!alg.compare("local_search")) {
                this->threshold = std::stod(alg_param);
                this->local_search();
            }
            else if (!alg.compare("simulated_annealing")) {
                this->annealing = alg_param;
                this->simulated_annealing();
            }
            else throw std::invalid_argument("\'Algorithm\' must be \'local_search\' or \'simulated_annealing\'");
        } catch (...) {
            throw;
        }
    }
