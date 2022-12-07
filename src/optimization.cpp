#include <string>
#include <algorithm>
#include <math.h>

#include <CGAL/convex_hull_2.h>
#include <CGAL/Convex_hull_traits_adapter_2.h>
#include <CGAL/property_map.h>

#include <CGAL/Kd_tree.h>
#include <CGAL/Fuzzy_iso_box.h>
#include <CGAL/Search_traits_2.h>

#include <optimization.hpp>

typedef CGAL::Convex_hull_traits_adapter_2<K, CGAL::Pointer_property_map<Point>::type> Convex_hull_traits_2;

typedef CGAL::Search_traits_2<K> Traits;
typedef CGAL::Kd_tree<Traits> Tree;
typedef CGAL:Fuzzy_iso_box<Traits> Fuzzy_iso_box;

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
    // R to be change
    double R = 0.5;
    std::vector<Point> ch_points = this->get_ch(this->pl_points);
    Polygon ch;
    for (auto it = ch_points.begin(); it != ch_points.end(); ++it) ch.push_back(*it);

    Polygon start_poly;
    for (auto it = this->pl_points.begin(); it != this->pl_points.end(); ++it) start_poly.push_back(*it);

    double ch_area = std::abs(ch.area());

    double start_area = std::abs(start_poly.area());

    std::cout << start_area << std::endl;

    double E;

    if (!this->opt.compare("-max")) E = this->pl_points.size() * (1 - start_area / ch_area);
    else E = this->pl_points.size() * start_area / ch_area;

    srand((unsigned) time(NULL));

    Tree tree;
    for (auto it = this->pl_points.begin(); it != this->pl_points.end(); ++it) tree.insert(*it);

    while (T >= 0) {
        Polygon curr_poly;
        for (auto it = this->pl_points.begin(); it != this->pl_points.end(); ++it) curr_poly.push_back(*it);
        double curr_area = std::abs(curr_poly.area());
        double updated_E;

        int q = rand() % this->pl_points.size();
        Point q_point = this->pl_points[q];

        std::vector<Point> temp_points = this->pl_points;

        auto qPos = temp_points.begin() + q;
        temp_points.erase(qPos);

        auto sPos = temp_points.begin() + q + 2;
        if(q == this->pl_points.size() - 2) sPos = temp_points.begin();

        temp_points.insert(sPos, q_point);

        Polygon temp_poly;
        for (auto it = temp_points.begin(); it != temp_points.end(); ++it) temp_poly.push_back(*it);

        // kd-tree validity check

        if (!this->opt.compare("-max")) {
                double temp_area= std::abs(temp_poly.area());
                double diff = temp_area - curr_area;
                updated_E = this->pl_points.size() * (1 - temp_area / ch_area);

                if (diff <= 0 && (exp( - ( updated_E - E) / T) < R)) continue;
                this->pl_points = temp_points;
        } else if (!this->opt.compare("-min")) {
                double temp_area= std::abs(temp_poly.area());
                double diff = curr_area - temp_area;
                updated_E = this->pl_points.size() * start_area / ch_area;

                if (diff <= 0 && (exp( - ( updated_E - E) / T) < R)) continue;
                this->pl_points = temp_points;
        }
        T = T - (double) 1 / this->L;
    }
    this->poly_line = this->get_segment(this->pl_points);
    Polygon end_poly;
    for (auto it = this->pl_points.begin(); it != this->pl_points.end(); ++it) end_poly.push_back(*it);

    double end_area = std::abs(end_poly.area());

    std::cout << end_area << std::endl;
}

void optimization::simulated_annealing_global(void) {
    double T = 1.0;
    // R to be change
    double R = 0.5;
    std::vector<Point> ch_points = this->get_ch(this->pl_points);
    Polygon ch;
    for (auto it = ch_points.begin(); it != ch_points.end(); ++it) ch.push_back(*it);

    Polygon start_poly;
    for (auto it = this->pl_points.begin(); it != this->pl_points.end(); ++it) start_poly.push_back(*it);

    double ch_area = std::abs(ch.area());

    double start_area = std::abs(start_poly.area());

    std::cout << start_area << std::endl;

    double E;

    if (!this->opt.compare("-max")) E = this->pl_points.size() * (1 - start_area / ch_area);
    else if (!this->opt.compare("-min")) E = this->pl_points.size() * start_area / ch_area;
    
    srand((unsigned) time(NULL));

    while (T >= 0) {
        Polygon curr_poly;
        for (auto it = this->pl_points.begin(); it != this->pl_points.end(); ++it) curr_poly.push_back(*it);
        double curr_area = std::abs(curr_poly.area());
        double updated_E;

        int q = rand() % this->pl_points.size();
        Point q_point = this->pl_points[q];

        std::vector<Point> temp_points = this->pl_points;
        
        auto qPos = temp_points.begin() + q;
        temp_points.erase(qPos);

        int s = rand() % this->pl_points.size();
        auto tPos = temp_points.begin() + s + 1;
        if(s == this->pl_points.size() - 1) tPos = temp_points.begin();

        temp_points.insert(tPos, q_point);

        Polygon temp_poly;
        for (auto it = temp_points.begin(); it != temp_points.end(); ++it) temp_poly.push_back(*it);

        if (!temp_poly.is_simple()) continue;
        

        
        if (!this->opt.compare("-max")) {
                double temp_area= std::abs(temp_poly.area());
                double diff = temp_area - curr_area;
                updated_E = this->pl_points.size() * (1 - temp_area / ch_area);

                if (diff <= 0) 
                    if (exp( - ( updated_E - E) / T) < R) {
                        // T = T - 1/L;
                        continue;
                    }
                this->pl_points = temp_points;

        }
        else if (!this->opt.compare("-min")) {
                double temp_area= std::abs(temp_poly.area());
                double diff = curr_area - temp_area;
                updated_E = this->pl_points.size() * start_area / ch_area;

                if (diff <= 0) 
                    if (exp( - ( updated_E - E) / T) < R) {
                        // T = T - 1/L;
                        continue;
                    }
                this->pl_points = temp_points;

        }
        T = T - (double) 1 / this->L;
    }
    this->poly_line = this->get_segment(this->pl_points);
    Polygon end_poly;
    for (auto it = this->pl_points.begin(); it != this->pl_points.end(); ++it) end_poly.push_back(*it);

    double end_area = std::abs(end_poly.area());
    
    std::cout << end_area << std::endl;

}

void optimization::simulated_annealing_subdivision(void) {
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

std::vector<Point> optimization::get_ch(std::vector<Point> points) {
    try {
        std::vector<Point> curr_ch;

        std::vector<std::size_t> indices(points.size()), out;
        std::iota(indices.begin(), indices.end(), 0);

        CGAL::convex_hull_2(indices.begin(), indices.end(), std::back_inserter(out), Convex_hull_traits_2(CGAL::make_property_map(points)));

        // push back points to current convex hull variable
        for(std::size_t j : out) curr_ch.push_back(points[j]);

        return curr_ch;
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
