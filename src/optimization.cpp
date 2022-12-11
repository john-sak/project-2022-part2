#include <string>
#include <algorithm>
#include <math.h>

#include <CGAL/convex_hull_2.h>
#include <CGAL/Convex_hull_traits_adapter_2.h>
#include <CGAL/property_map.h>

#include <CGAL/Kd_tree.h>
#include <CGAL/Fuzzy_iso_box.h>
#include <CGAL/Search_traits_2.h>

#include "../lib/polyline/polyline.hpp"
#include <optimization.hpp>

typedef CGAL::Convex_hull_traits_adapter_2<K, CGAL::Pointer_property_map<Point>::type> Convex_hull_traits_2;

typedef CGAL::Search_traits_2<K> Traits;
typedef CGAL::Kd_tree<Traits> Tree;
typedef CGAL::Fuzzy_iso_box<Traits> Fuzzy_iso_box;

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

        auto sPos = temp_points.begin() + q + 1; // q + 2 to get s, -1 because of line 178
        if (q == this->pl_points.size() - 1) sPos = temp_points.begin() + 1;
        else if (q == this->pl_points.size() - 2) sPos = temp_points.begin();

        temp_points.insert(sPos, q_point);

        Polygon temp_poly;
        for (auto it = temp_points.begin(); it != temp_points.end(); ++it) temp_poly.push_back(*it);

        int p = (q > 0 ? q - 1 : this->pl_points.size() - 1);
        int r = (q < this->pl_points.size() - 1 ? q + 1 : 0);
        int s = (r < this->pl_points.size() - 1 ? r + 1 : 0);

        Point p_point = this->pl_points[p], r_point = this->pl_points[r], s_point = this->pl_points[s];

        Point up_ri_point(std::max({p_point.x(), q_point.x(), r_point.x(), s_point.x()}), std::max({p_point.y(), q_point.y(), r_point.y(), s_point.y()}));
        Point lo_le_point(std::min({p_point.x(), q_point.x(), r_point.x(), s_point.x()}), std::min({p_point.y(), q_point.y(), r_point.y(), s_point.y()}));

        std::list<Point> result;
        Fuzzy_iso_box exact_range(lo_le_point, up_ri_point);
        tree.search(std::back_inserter(result), exact_range);


        std::vector<Segment> temp_line = this->get_segment(temp_points);
        std::vector<Segment> lines;

        for(auto it = temp_line.begin(); it != temp_line.end(); it++) {
            if(std::find(result.begin(), result.end(),it->source())!= result.end() || std::find(result.begin(), result.end(),it->target()) != result.end())
                lines.push_back(*it);
        }

        int flag = 0;
        Segment pr(p_point, r_point), qs(q_point, s_point);

        if (intersection(qs, pr)) continue;
        auto qsPos = std::find(lines.begin(), lines.end(), qs);
        lines.erase(qsPos);

        auto rqPos = std::find(lines.begin(), lines.end(), Segment(pr.target(), qs.source()));
        lines.erase(rqPos);


        auto prPos = std::find(lines.begin(), lines.end(), pr);
        lines.erase(prPos);

        for (Segment line : lines) {
            CGAL::Object result = intersection(line, pr);
            Point isPoint;
            Segment isSeg;
            // if line intersects whether pr or qs, this solution is *not* valid
            if (CGAL::assign(isPoint, result) &&  !(line.target() == pr.source())) {
                flag = 1;
                break;
            }
            else if (CGAL::assign(isSeg, result)) {
                flag = 1;
                break;

            }
            result = intersection(line, qs);
            if (CGAL::assign(isPoint, result) &&  !(line.source() == qs.target())) {
                flag = 1;
                break;
            }
            else if (CGAL::assign(isSeg, result)) {
                flag = 1;
                break;

            }

        }
            
        if (flag == 1) continue;

        if (!this->opt.compare("-max")) {
                double temp_area= std::abs(temp_poly.area());
                double diff = temp_area - curr_area;
                updated_E = this->pl_points.size() * (1 - temp_area / ch_area);

                if (diff <= 0) 
                    if (exp( - ( updated_E - E) / T) < R) continue;
                this->pl_points = temp_points;
        } else if (!this->opt.compare("-min")) {
                double temp_area= std::abs(temp_poly.area());
                double diff = curr_area - temp_area;
                updated_E = this->pl_points.size() * start_area / ch_area;

                if (diff <= 0) 
                    if (exp( - ( updated_E - E) / T) < R) continue;
                this->pl_points = temp_points;
        }
        T = T - (double) 1 / this->L;
    }
    this->poly_line = this->get_segment(this->pl_points);
    Polygon end_poly;
    for (auto it = this->pl_points.begin(); it != this->pl_points.end(); ++it) end_poly.push_back(*it);
    if(!end_poly.is_simple()) std::cout << "FUCKK" << std::endl;

    double end_area = std::abs(end_poly.area());

    std::cout << end_area << std::endl;
}

std::vector<Point> optimization::simulated_annealing_global(std::vector<Point> points) {
    double T = 1.0;
    double R;
    std::vector<Point> ch_points = this->get_ch(points);
    Polygon ch;
    for (auto it = ch_points.begin(); it != ch_points.end(); ++it) ch.push_back(*it);

    Polygon start_poly;
    for (auto it = points.begin(); it != points.end(); ++it) start_poly.push_back(*it);

    double ch_area = std::abs(ch.area());

    double start_area = std::abs(start_poly.area());

    std::cout << start_area << std::endl;

    double E;

    if (!this->opt.compare("-max")) E = points.size() * (1 - start_area / ch_area);
    else if (!this->opt.compare("-min")) E = points.size() * start_area / ch_area;
    
    srand((unsigned) time(NULL));

    while (T >= 0) {
        R = (double) rand() / RAND_MAX;
        Polygon curr_poly;
        for (auto it = points.begin(); it != points.end(); ++it) curr_poly.push_back(*it);
        double curr_area = std::abs(curr_poly.area());
        double updated_E;

        int q = rand() % points.size();
        Point q_point = points[q];

        std::vector<Point> temp_points = points;
        
        auto qPos = temp_points.begin() + q;
        temp_points.erase(qPos);

        int s = rand() % points.size();
        auto tPos = temp_points.begin() + s + 1;
        if(s == points.size() - 1) tPos = temp_points.begin();

        temp_points.insert(tPos, q_point);

        Polygon temp_poly;
        for (auto it = temp_points.begin(); it != temp_points.end(); ++it) temp_poly.push_back(*it);

        if (!temp_poly.is_simple()) continue;
        

        
        if (!this->opt.compare("-max")) {
                double temp_area= std::abs(temp_poly.area());
                double diff = temp_area - curr_area;
                updated_E = points.size() * (1 - temp_area / ch_area);

                if (diff <= 0) 
                    if (exp( - ( updated_E - E) / T) < R) {
                        // T = T - 1/L;
                        continue;
                    }
                points = temp_points;

        }
        else if (!this->opt.compare("-min")) {
                double temp_area= std::abs(temp_poly.area());
                double diff = curr_area - temp_area;
                updated_E = points.size() * start_area / ch_area;

                if (diff <= 0) 
                    if (exp( - ( updated_E - E) / T) < R) {
                        // T = T - 1/L;
                        continue;
                    }
                points = temp_points;

        }
        T = T - (double) 1 / this->L;
    }
    // this->poly_line = this->get_segment(points);
    Polygon end_poly;
    for (auto it = points.begin(); it != points.end(); ++it) end_poly.push_back(*it);

    double end_area = std::abs(end_poly.area());
    
    std::cout << end_area << std::endl;
    return points;
}

void optimization::simulated_annealing_subdivision(void) {
    // m to be change
    int m = 10;

    std::sort(this->pl_points.begin(), this->pl_points.end(), [] (const Point &a, const Point &b) {
        return (a.x() < b.x());
    });

    std::vector<std::vector<Point>> sub_points;
    std::vector<std::list<Segment>> marked_edges;

    int i = 0, k = 0;
    while (i < this->pl_points.size()) {
        std::vector<Point> division;
        std::list<Segment> marked;

        if (k != 0) marked.push_back(Segment(this->pl_points[i], this->pl_points[i + 1]));
        for (int j = 0; j < std::ceil(0.75 * m) && i < this->pl_points.size(); j++, i++) division.push_back(this->pl_points[i]);
        if (i >= this->pl_points.size() - 1) {
            if (i == this->pl_points.size() - 1) division.push_back(this->pl_points[i]);
            sub_points.push_back(division);
            marked_edges.push_back(marked);
            k++;
            break;
        }
        int j = 0;
        while (!(this->pl_points[i - 2].y() < this->pl_points[i - 1].y() && this->pl_points[i - 1].y() > this->pl_points[i].y())) {
            division.push_back(this->pl_points[i]);
            i++;
            j++;
            if (i == this->pl_points.size() - 1) {
                division.push_back(this->pl_points[i]);
                i++;
                break;
            }
            if (j == std::ceil(0.5 * m)) throw std::exception();
        }
        if (i != this->pl_points.size()) {
            i--;
            marked.push_back(Segment(this->pl_points[i - 1], this->pl_points[i]));
        }
        marked_edges.push_back(marked);
        sub_points.push_back(division);
        k++;
    }

    std::vector<Point> polygons[k];
    //create init  polygon
    for (int i = 0; i < k; i++) {
        // sub_points[i] contains Point, we want std::pair<float, float>
        std::vector<std::pair<float, float>> floats;

        polyline S(floats, "incremental", "1", "1a", "");

        polygons[i].resize(sub_points[i].size());

        int tries = 0;
        while (tries < 1000) {
            polygons[i] = this->simulated_annealing_global(sub_points[i]);
            tries++;

            // if (ploygons[i] contains marked_edges[i]) break;
        }
        if (tries > 1000) throw std::exception();
    }

    // connect polygons

    for (int i = 0; i < k; i++) {
        for (auto it = polygons[i].begin(); it < polygons[i].end(); ++it)
            std::cout << *it << " ";    

            std::cout << std::endl;
    }

    // for each set of subpoints create simple polygon using algo from part 1

    //for each polygon use global annealing
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
                else if (!alg_param.compare("global")) this->pl_points = this->simulated_annealing_global(this->pl_points);
                else if (!alg_param.compare("subdivision")) this->simulated_annealing_subdivision();
                else throw std::invalid_argument("\'Annealing\' must be \'local\', \'global\' or \'subdivision\'");
            }
            else throw std::invalid_argument("\'Algorithm\' must be \'local_search\' or \'simulated_annealing\'");
        } catch (...) {
            throw;
        }
    }
