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

    std::vector<Point> ch_points = this->get_ch(this->pl_points);
    Polygon ch;
    for (auto it = ch_points.begin(); it != ch_points.end(); ++it) ch.push_back(*it);

    this->ch_area = std::abs(ch.area());

    //get initial area
    Polygon start_poly;
    for (auto it = this->pl_points.begin(); it != this->pl_points.end(); ++it) start_poly.push_back(*it);
    this->start_area = std::abs(start_poly.area());


    double area_diff;
    auto start = std::chrono::high_resolution_clock::now();
    do {
        // get initial polygon area
        Polygon curr_poly;
        for (auto it = this->pl_points.begin(); it != this->pl_points.end(); ++it) curr_poly.push_back(*it);
        double curr_area = std::abs(curr_poly.area());

        //vector of all possible updates
        std::vector<update_node> updates;

        for (auto it = this->poly_line.begin(); it != this->poly_line.end(); ++it) {
            for (int i = 1; i <= this->L; i++) {
                //loop through all i-legth vertices
                for (size_t j = 0; j < this->poly_line.size(); j += i) {
                   // create i - length segment to be switched
                    std::vector<Segment> V;
                    auto it2 = this->poly_line.begin() + j;
                    for ( int z = 0; z < L; z++) {
                        V.push_back(*it2);
                        it2++;
                    }
                    // switch edges
                    std::vector<Point> temp_points = this->replace_edges( *it, V);

                    Polygon temp_poly;
                    for (auto it3 = temp_points.begin(); it3 != temp_points.end(); ++it3) temp_poly.push_back(*it3);
                    // check for polygon simplicity
                    if (!temp_poly.is_simple()) continue;


                    if (!this->opt.compare("-max")) {
                        // get new polygon area
                        double temp_area= std::abs(temp_poly.area());
                        

                        double diff = temp_area - curr_area;
                        // check is area is optimized 
                        if (diff <= 0) continue;

                        else {
                            // push possible update to list 
                            update_node update;
                            update.e = *it;
                            update.V = V;
                            update.area_change = diff;
                            updates.push_back(update);
                        }
                    }

                    else if (!this->opt.compare("-min")) {
                        // get new polygon area
                        double temp_area= std::abs(temp_poly.area());

                        double diff = curr_area - temp_area;
                        // check is area is optimized 
                        if (diff <= 0) continue;

                        else {
                            // push possible update to list 
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
        // sort update list based on area optimization
        std::sort(updates.begin(),updates.end(),compareAreaChange);
        // loop on update list
        for (auto it = updates.begin(); it != updates.end(); ++it) {
            // update polygon
            std::vector<Point> temp_points = this->replace_edges(it->e, it->V);

            Polygon temp_poly;
            for (auto it2 = temp_points.begin(); it2 != temp_points.end(); ++it2) temp_poly.push_back(*it2);
            // check for simplicity
            if (!temp_poly.is_simple()) continue;

            if (!this->opt.compare("-max")) {
                double temp_area= std::abs(temp_poly.area());
                double diff = temp_area - curr_area;
                // check for optimization
                if (diff <= 0) continue;
                // update polygon
                else this->pl_points = temp_points;
            }

            if (!this->opt.compare("-min")) {
                double temp_area= std::abs(temp_poly.area());
                double diff = curr_area - temp_area;
                // check for optimization
                if (diff <= 0) continue;
                // update polygon
                else this->pl_points = temp_points;
            }
        }
        // get optimized area
        Polygon updated_poly;
        for (auto it = this->pl_points.begin(); it != this->pl_points.end(); ++it) updated_poly.push_back(*it);
        double updated_area = std::abs(updated_poly.area());
        // get area difference
        if (!this->opt.compare("-max")) area_diff = updated_area - curr_area;
        else if (!this->opt.compare("-max")) area_diff = curr_area - updated_area;
        // update segments
        this->poly_line = this->get_segment(this->pl_points);

    } while (area_diff >= this->threshold);

    auto stop = std::chrono::high_resolution_clock::now();

    // get final area
    Polygon end_poly;
    for (auto it = this->pl_points.begin(); it != this->pl_points.end(); ++it) end_poly.push_back(*it);
    this->end_area = std::abs(end_poly.area());

    this->write_to_file("local_search", std::chrono::duration_cast<std::chrono::milliseconds>(stop - start).count());
    
}

void optimization::simulated_annealing_local(void) {
    double T = 1.0;
    double R;
    double E;

    // calculate initial energy
    if (!this->opt.compare("-max")) E = this->pl_points.size() * (1 - this->start_area / this->ch_area);
    else E = this->pl_points.size() * this->start_area / this->ch_area;

    // initialize kd-tree
    Tree tree;
    for (auto it = this->pl_points.begin(); it != this->pl_points.end(); ++it) tree.insert(*it);

    auto start = std::chrono::high_resolution_clock::now();

    while (T >= 0) {
        srand((unsigned) time(NULL));
        R = (double) rand() / RAND_MAX;

        // get current polygon area
        Polygon curr_poly;
        for (auto it = this->pl_points.begin(); it != this->pl_points.end(); ++it) curr_poly.push_back(*it);
        double curr_area = std::abs(curr_poly.area());
        double updated_E;
        // get random polygon point
        int q = rand() % this->pl_points.size();
        Point q_point = this->pl_points[q];

        std::vector<Point> temp_points = this->pl_points;
        // switch points in polygon line
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
        // get points in open axis-parallel rectangle bounding p, q, r, s 
        std::list<Point> result;
        Fuzzy_iso_box exact_range(lo_le_point, up_ri_point);
        tree.search(std::back_inserter(result), exact_range);


        std::vector<Segment> temp_line = this->get_segment(temp_points);
        std::vector<Segment> lines;
        // get possible intersected edges
        for(auto it = temp_line.begin(); it != temp_line.end(); it++) {
            if(std::find(result.begin(), result.end(),it->source())!= result.end() || std::find(result.begin(), result.end(),it->target()) != result.end())
                lines.push_back(*it);
        }

        int flag = 0;
        Segment pr(p_point, r_point), qs(q_point, s_point);
        // check if edges qs and pr intersect
        if (intersection(qs, pr)) continue;
        auto qsPos = std::find(lines.begin(), lines.end(), qs);
        lines.erase(qsPos);

        auto rqPos = std::find(lines.begin(), lines.end(), Segment(pr.target(), qs.source()));
        lines.erase(rqPos);


        auto prPos = std::find(lines.begin(), lines.end(), pr);
        lines.erase(prPos);
        // check if other edges in the rectangle bounding p, q, r, s intersect pr or qs
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
                // get updated area
                double temp_area= std::abs(temp_poly.area());
                double diff = temp_area - curr_area;
                // calculate updated energy
                updated_E = this->pl_points.size() * (1 - temp_area / this->ch_area);
                // check if area is optimized or metropolis criterion is valid
                if (diff <= 0) 
                    if (exp( - ( updated_E - E) / T) < R) continue;
                    // update polygon line
                this->pl_points = temp_points;
        } else if (!this->opt.compare("-min")) {
                // get updated area
                double temp_area= std::abs(temp_poly.area());
                double diff = curr_area - temp_area;
                // calculate updated energy
                updated_E = this->pl_points.size() * temp_area / this->ch_area;
                // check if area is optimized or metropolis criterion is valid
                if (diff <= 0) 
                    if (exp( - ( updated_E - E) / T) < R) continue;
                // update polygon line
                this->pl_points = temp_points;
        }
        // update T
        T = T - (double) 1 / this->L;
    }

    auto stop = std::chrono::high_resolution_clock::now();

    // update polygon segment
    this->poly_line = this->get_segment(this->pl_points);
    Polygon end_poly;
    for (auto it = this->pl_points.begin(); it != this->pl_points.end(); ++it) end_poly.push_back(*it);
    // get optimized area
    this->end_area = std::abs(end_poly.area());

    this->write_to_file("simulated_annealing", std::chrono::duration_cast<std::chrono::milliseconds>(stop - start).count());
}

std::vector<Point> optimization::simulated_annealing_global(std::vector<Point> points) {
    double T = 1.0;
    double R;
    // get convex hull
    std::vector<Point> ch_points = this->get_ch(points);
    Polygon ch;
    for (auto it = ch_points.begin(); it != ch_points.end(); ++it) ch.push_back(*it);

    Polygon start_poly;
    for (auto it = points.begin(); it != points.end(); ++it) start_poly.push_back(*it);
    // get convex hull area
    double ch_area = std::abs(ch.area());
    // get initial polygon area
    double start_area = std::abs(start_poly.area());

    double E;
    // calculate initial energy
    if (!this->opt.compare("-max")) E = points.size() * (1 - start_area / ch_area);
    else if (!this->opt.compare("-min")) E = points.size() * start_area / ch_area;
    
    while (T >= 0) {
        srand((unsigned) time(NULL));
        R = (double) rand() / RAND_MAX;
        Polygon curr_poly;
        for (auto it = points.begin(); it != points.end(); ++it) curr_poly.push_back(*it);
        // get current area
        double curr_area = std::abs(curr_poly.area());
        double updated_E;

        int q = rand() % points.size();
        Point q_point = points[q];

        std::vector<Point> temp_points = points;
        // do global transition
        auto qPos = temp_points.begin() + q;
        temp_points.erase(qPos);

        int s = rand() % points.size();
        auto tPos = temp_points.begin() + s + 1;
        if(s == points.size() - 1) tPos = temp_points.begin();

        temp_points.insert(tPos, q_point);

        Polygon temp_poly;
        for (auto it = temp_points.begin(); it != temp_points.end(); ++it) temp_poly.push_back(*it);
        // check for simplicity
        if (!temp_poly.is_simple()) continue;
        

        
        if (!this->opt.compare("-max")) {
                // get updated area
                double temp_area= std::abs(temp_poly.area());
                double diff = temp_area - curr_area;
                // get updated energy
                updated_E = points.size() * (1 - temp_area / ch_area);
                // check if area is optimized or metropolis criterion is valid
                if (diff <= 0) 
                    if (exp( - ( updated_E - E) / T) < R) continue;
                // update polygon line
                points = temp_points;

        }
        else if (!this->opt.compare("-min")) {
                // get updated area
                double temp_area= std::abs(temp_poly.area());
                double diff = curr_area - temp_area;
                // get updated energy
                updated_E = points.size() * temp_area / ch_area;
                // check if area is optimized or metropolis criterion is valid
                if (diff <= 0) 
                    if (exp( - ( updated_E - E) / T) < R) continue;
                // update polygon line
                points = temp_points;

        }
        // update T
        T = T - (double) 1 / this->L;
    }
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
    std::vector<Point> lower_hull;
    std::vector<Point> upper_hull;

    //create init  polygon
    for (int i = 0; i < k; i++) {
        // sub_points[i] contains Point, we want std::pair<float, float>
        std::vector<std::pair<float, float>> floats;

        for ( auto it = sub_points[i].begin(); it != sub_points[i].end(); ++it) floats.push_back(std::make_pair((float) it->x(),  (float) it->y()));

        polyline S(floats, "incremental", "1", "1a", "");

        polygons[i].resize(sub_points[i].size());

        int tries = 0, ok = 1;
        while (tries < 1000) {
            polygons[i] = this->simulated_annealing_global(sub_points[i]);
            tries++;

            ok = 1;
            for (auto it_marked = marked_edges[i].begin(); it_marked != marked_edges[i].end(); it_marked++) {
                auto it = std::find(polygons[i].begin(), polygons[i].end(), it_marked->source());
                if (it == polygons[i].end()) {
                    ok = 0;
                    break;
                }

                auto it_next = (it == polygons[i].end() - 1 ? polygons[i].begin() : it + 1);
                auto it_prev = (it == polygons[i].begin() ? polygons[i].end() - 1 : it - 1);
                if (*it_next != it_marked->target() && *it_prev != it_marked->target()) {
                    ok = 0;
                    break;
                }
            }
            if (ok != 0) break;
        }
        if (ok != 1) throw std::exception();

        if (i == 0) {
            auto it_list = marked_edges[i].begin();
            auto q_point = std::find(polygons[i].begin(), polygons[i].end(), it_list->target());
            if (q_point == polygons[i].end()) throw std::exception();
            if (polygons[i].begin()->y() > std::next(polygons[i].begin())->y()) {
                for (auto it = polygons[i].begin(); it != q_point; it++) lower_hull.push_back(*it);
                for (auto it = q_point; it != polygons[i].end(); it++) upper_hull.push_back(*it);
            } else {
                lower_hull.push_back(*polygons[i].begin());
                for (auto it = polygons[i].end() - 1; it != q_point; it--) lower_hull.push_back(*it);
                for (auto it = q_point; it != polygons[i].begin(); it--) upper_hull.push_back(*it);
            }
        } else if (i == (k - 1)) {
            if (polygons[i].begin()->y() > std::next(polygons[i].begin())->y()) for (auto it = std::next(polygons[i].begin()); it != polygons[i].end(); it++) lower_hull.push_back(*it);
            else for (auto it = polygons[i].end() - 1; it != polygons[i].begin(); it--) lower_hull.push_back(*it);
        } else {
            auto it_list = std::next(marked_edges[i].begin());
            auto q_point = std::find(polygons[i].begin(), polygons[i].end(), it_list->target());
            if (q_point == polygons[i].end()) throw std::exception();
            auto start = upper_hull.begin();
            if (polygons[i].begin()->y() > std::next(polygons[i].begin())->y()) {
                for (auto it = std::next(polygons[i].begin()); it != q_point; it++) lower_hull.push_back(*it);
                for (auto it = q_point; it != polygons[i].end(); it++) upper_hull.insert(start, *it);
            } else {
                for (auto it = polygons[i].end() - 1; it != q_point; it--) lower_hull.push_back(*it);
                for (auto it = q_point; it != polygons[i].begin(); it--) upper_hull.insert(start, *it);
            }
        }
    }

    std::vector<Point> all_points;
    for (auto it = lower_hull.begin(); it != lower_hull.end(); it++) all_points.push_back(*it);
    for (auto it = upper_hull.begin(); it != upper_hull.end(); it++) all_points.push_back(*it);

    for (auto it = all_points.begin(); it != all_points.end(); it++) std::cout << *it << std::endl;

    // just need to do
    // this->pl_points = all_points
    // this->simulated_annealing_local()
    // (?)
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

void optimization::write_to_file(std::string alg, int time) const {
    try {
        std::ofstream file(this->out_file);
        file << "Optimal Area Polygonization" << std::endl;
        for (Point p : this->pl_points) file << p.x() << " " << p.y() << std::endl;
        for (Segment s : this->poly_line) file << s.source() << " " << s.target() << std::endl;
        file << "Algorithm: " << alg  << this->opt << std::endl;
        file << "Area_initial: " << this->start_area << std::endl;
        file << "Area: " << this->end_area << std::endl;
        file << "Ratio_initial: " << (this->start_area / this->ch_area) << std::endl;
        file << "Ratio: " << (this->end_area / this->ch_area) << std::endl;
        file << "Construction time: " << time << " msec" << std::endl;
        file.close();
    } catch (...) {
        throw;
    }
    return;
}



optimization::optimization(std::vector<Point> pl_points,std::vector<Segment> poly_line, std::string alg, std::string L, std::string opt, std::string alg_param, std::string out_file, double start_area, double ch_area)
    :out_file(out_file), pl_points(pl_points),poly_line(poly_line), opt(opt), start_area(start_area), ch_area(ch_area) {
        try {
            this->L = std::stoi(L);
            if (!alg.compare("local_search")) {
                this->threshold = std::stod(alg_param);
                this->local_search();
            }
            else if (!alg.compare("simulated_annealing")) {
                this->annealing = alg_param;
                if (!alg_param.compare("local")) this->simulated_annealing_local();
                else if (!alg_param.compare("global")) {
                    auto start = std::chrono::high_resolution_clock::now();
                    this->pl_points = this->simulated_annealing_global(this->pl_points);
                    auto stop = std::chrono::high_resolution_clock::now();
                    this->poly_line = this->get_segment(this->pl_points);
                    Polygon end_poly;
                    for (auto it = this->pl_points.begin(); it != this->pl_points.end(); ++it) end_poly.push_back(*it);
                    //get final area
                    this->end_area = std::abs(end_poly.area());
    
                     this->write_to_file("simulated_annealing", std::chrono::duration_cast<std::chrono::milliseconds>(stop - start).count());
                    }
                else if (!alg_param.compare("subdivision")) this->simulated_annealing_subdivision();
                else throw std::invalid_argument("\'Annealing\' must be \'local\', \'global\' or \'subdivision\'");
            }
            else throw std::invalid_argument("\'Algorithm\' must be \'local_search\' or \'simulated_annealing\'");
        } catch (...) {
            throw;
        }
    }
