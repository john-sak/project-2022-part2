#include <vector>
#include <stdexcept>
#include <algorithm>
#include <iostream>
#include <fstream>
#include <limits>
#include <chrono>

#include <CGAL/convex_hull_2.h>
#include <CGAL/Convex_hull_traits_adapter_2.h>
#include <CGAL/property_map.h>
#include <CGAL/intersections.h>
#include <CGAL/squared_distance_2.h>

#include <polyline.hpp>

typedef CGAL::Convex_hull_traits_adapter_2<K, CGAL::Pointer_property_map<Point>::type> Convex_hull_traits_2;
typedef K::Intersect_2 Intersect;

void polyline::incremental(int init) {
    try {
        auto start = std::chrono::high_resolution_clock::now();

        // sort points
        this->sort_points(init);

        // initialize polygon
        int i = this->init_triangle();

        // expand polygon
        this->expand(i);

        //get polygon area
        Polygon pl_poly;
        for(auto it = this->poly_line.begin(); it != this->poly_line.end(); ++it) pl_poly.push_back(it->target());
        this->pl_area = std::abs(pl_poly.area());

        //get convex hull area
        Polygon ch_poly;
        std::vector<Point> ch_points = this->get_ch(this->points);
        for (auto it = ch_points.begin(); it != ch_points.end(); ++it) ch_poly.push_back(*it);
        this->ch_area = std::abs(ch_poly.area());

        auto stop = std::chrono::high_resolution_clock::now();

        // write to output file
        this->write_to_file("Incremental", std::chrono::duration_cast<std::chrono::milliseconds>(stop - start).count());
    } catch (...) {
        throw;
    }
    return;
}

void polyline::convex_hull(void) {
    try {
        auto start = std::chrono::high_resolution_clock::now();

        // get convex_hull of all points
        std::vector<Point> ch = this->get_ch(this->points);

        // initialize polyline to be the convex_hull of all the points
        for (int i = 0; i < ch.size(); i++) {
            this->pl_points.push_back(ch[i]);
            if (i < ch.size() - 1) this->poly_line.push_back(Segment(ch[i], ch[i + 1]));
            else this->poly_line.push_back(Segment(ch[i], ch[0]));
        }

        // for every edge in poly_line, find the closest visible point p and add it to the polyline
        // when there are no more edges to go through, we are done
        std::vector<Segment>::size_type size = this->poly_line.size();
        for (std::vector<Segment>::size_type i = 0; i < size; i++) {
            // for every edge on the polyline
            Segment edge = this->poly_line[i];

            // find closest visible point p and let p_index be the index of point p
            int min, found = 0, p_index;
            for (int j = 0; j < this->points.size(); j++) {
                // for every point
                Point p = this->points[j];

                // if p is in this->pl_points, continue
                if (!((*(std::find(this->pl_points.begin(), this->pl_points.end(), p)) == *this->pl_points.end()) && (p != *this->pl_points.end()))) continue;

                // if edge is not visible by p, continue
                Segment edge1(p, edge.source()), edge2(p, edge.target());
                if (!this->is_vis(edge1, edge2)) continue;

                // find closest visible point
                int dist = CGAL::squared_distance(edge, p);
                if (found == 0) {
                    p_index = j;
                    min = dist;
                    found = 1;
                } else if (dist < min) {
                    p_index = j;
                    min = dist;
                }
            }

            // if no visible points exist for this edge, continue to the next one
            if (found == 0) continue;

            // insert p to polyline and remove this->poly_line[i]
            this->poly_line.push_back(Segment(edge.source(), this->points[p_index]));
            this->poly_line.push_back(Segment(this->points[p_index], edge.target()));

            auto index = std::find(poly_line.begin(), poly_line.end(), edge);
            this->poly_line.erase(index);
            i--;

            //  add point to polygon points
            auto pindex = std::find(pl_points.begin(), pl_points.end(), edge.target());
            if (pindex == this->pl_points.begin()) this->pl_points.push_back(this->points[p_index]);
            else this->pl_points.insert(pindex, this->points[p_index]);


            // increment the size of the polyline to go through the edges we just added
            size++;
        }

        // get polygon segments in order
        this->poly_line.clear();
        this->poly_line = this->get_segment(this->pl_points);

        // get polygon area
        Polygon pl_poly;
        for(auto it = this->poly_line.begin(); it != this->poly_line.end(); ++it) pl_poly.push_back(it->target());
        this->pl_area = std::abs(pl_poly.area());

        // get conex hull area
        Polygon ch_poly;
        // ch is the convex_hull of all the points (see line 57)
        for (auto it = ch.begin(); it != ch.end(); ++it) ch_poly.push_back(*it);
        this->ch_area = std::abs(ch_poly.area());

        auto stop = std::chrono::high_resolution_clock::now();

        // write to output file
        this->write_to_file("Convex_Hull", std::chrono::duration_cast<std::chrono::milliseconds>(stop - start).count());
    } catch (...) {
        throw;
    }
    return;
}

void polyline::sort_points(int type) {
    try {
        switch (type) {
            case 1:
                // sort x descending
                std::sort(this->points.begin(), this->points.end(), [] (const Point &a, const Point &b) {
                    return (a.x() > b.x());
                });
                break;

            case 2:
                // sort x ascending
                std::sort(this->points.begin(), this->points.end(), [] (const Point &a, const Point &b) {
                    return (a.x() < b.x());
                });
                break;

            case 3:
                // sort y descending
                std::sort(this->points.begin(), this->points.end(), [] (const Point &a, const Point &b) {
                    return (a.y() > b.y());
                });
                break;

            case 4:
                // sort y ascending
                std::sort(this->points.begin(), this->points.end(), [] (const Point &a, const Point &b) {
                    return (a.y() < b.y());
                });
                break;

            default:
                throw "Error: Couldn't sort vector";
        }
        return;
    } catch (...) {
        throw;
    }
}

int polyline::init_triangle(void) {
    try {
        this->pl_points.push_back(this->points[0]);
        this->pl_points.push_back(this->points[1]);
        this->pl_points.push_back(this->points[2]);

        this->poly_line.push_back(Segment(this->points[0], this->points[1]));
        this->poly_line.push_back(Segment(this->points[1], this->points[2]));

        //if three initial points are not collinear triangle can be created
        if (!CGAL::collinear(this->points[0], this->points[1], this->points[2])) {
            this->poly_line.push_back(Segment(this->points[2], this->points[0]));
            return 3;
        }

        //if initial points are collinear
        int i = 3;
        int flag = 1;

        // stop when the first non collinear point is found
        do {
            //create next segment
            this->pl_points.push_back(this->points[i]);
            this->poly_line.push_back(Segment(this->points[i - 1], this->points[i]));

            if (!CGAL::collinear(this->points[0], this->points[1], this->points[i])) {
                flag = 0;

                //point i is not collinear so triangle can be closed
                this->poly_line.push_back(Segment(this->points[i], this->points[0]));
            }

            i++;
        } while (flag);

        // return number of points needed for triangle creation
        return i;
    } catch (...) {
        throw;
    }
}

void polyline::expand(int i) {
    try {
        // current convex hull variable
        std::vector<Point> curr_ch;

        // previous convex hull variable
        std::vector<Point> prev_ch;

        std::vector<Segment> curr_ch_segment;
        std::vector<Segment> prev_ch_segment;

        std::vector<Segment> red_edges;
        std::vector<Segment> vis_edges;

        // get convex hull of current polygon points
        curr_ch = get_ch(this->pl_points);

        while (i < this->points.size()) {
            Point p = this->points[i];

            // insert next point to polygon line points
            this->pl_points.push_back(this->points[i]);

            prev_ch = curr_ch;
            prev_ch_segment = this->get_segment(prev_ch);

            // get convex hull of current polygon points plus the next point to add
            curr_ch = get_ch(this->pl_points);
            curr_ch_segment = this->get_segment(curr_ch);

            // red lines of current convex hull are the ones removed from previous convex hull
            // compare the two segments to get red lines
            red_edges = this->get_red_edges(prev_ch_segment, curr_ch_segment);

            // point has the same x or y coordinate (depends of sorting) as a prev one
            if (red_edges.size() == 0) {

                if (!this->init.compare("1a") || this->init.compare("1b")) {

                    // find red edes
                    for (auto it = curr_ch_segment.begin(); it != curr_ch_segment.end(); it++) {
                        if(it->source().x() == p.x() || it->target().x() == p.x())
                            red_edges.push_back(*it);
                    }
                } else if (!this->init.compare("2a") || this->init.compare("2b")) {

                    // find red edes
                    for (auto it = curr_ch_segment.begin(); it != curr_ch_segment.end(); it++) {
                        if(it->source().y() == p.y() || it->target().y() == p.y())
                            red_edges.push_back(*it);
                    }
                }
            }

            // for every red find visible
            vis_edges = this->get_vis_edges(i, red_edges);

            Segment replaceable_edge;
            switch(edge_sel) {
                case 1:
                    // choose visible replaceable edge randomly
                    replaceable_edge = vis_edges[std::rand() % vis_edges.size()];
                    break;

                case 2:
                    // choose visible replaceable edge that adds min area
                    replaceable_edge = min_area(vis_edges, i);
                    break;

                case 3:
                    // choose visible replaceable edge that adds max area
                    replaceable_edge = max_area(vis_edges, i);
                    break;

                default:
                    throw "Error: Wrong edge_selection value!";
            }

            // remove point to add it to the right place
            this->pl_points.pop_back();

            //insert segment
            auto index = std::find(poly_line.begin(), poly_line.end(), replaceable_edge);
            this->poly_line.insert(index, Segment(replaceable_edge.source(), this->points[i]));
            index = std::find(poly_line.begin(), poly_line.end(), replaceable_edge);
            this->poly_line.insert(index, Segment(this->points[i], replaceable_edge.target()));
            index = std::find(poly_line.begin(), poly_line.end(), replaceable_edge);
            this->poly_line.erase(index);

            //insert point
            auto pindex = std::find(pl_points.begin(), pl_points.end(), replaceable_edge.target());
            if (pindex == this->pl_points.begin()) this->pl_points.push_back(this->points[i]);
            else this->pl_points.insert(pindex, this->points[i]);

            i++;
        }

        return;
    } catch (...) {
        throw;
    }
}

std::vector<Point> polyline::get_ch(std::vector<Point> points) {
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

std::vector<Segment> polyline::get_segment(std::vector<Point> points) {
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

std::vector<Segment> polyline::get_red_edges(std::vector<Segment> prev, std::vector<Segment> curr) {
    try {
        std::vector<Segment> seg;

        // red edges are the edges that are on the previous convex hull, but not the current one
        for (Segment line : prev) if ((*(std::find(curr.begin(), curr.end(), line)) == *curr.end()) && (line != *curr.end())) seg.push_back(line);

        return seg;
    } catch (...) {
        throw;
    }
}

std::vector<Segment> polyline::get_vis_edges(int i, std::vector<Segment> red_edges) {
    try {
        std::vector<Segment> seg;
        Point p = this->points[i];

        for (Segment red : red_edges) {
            // check if convex hull red edge is also polyline edge
            // if the condition is true the line is visible
            int flag = 1;
            for (Segment line : this->poly_line) {
                if (red == line) {
                    flag = 0;
                    seg.push_back(line);
                    break;
                }
            }

            // if red edge is part of the polyline there no more visible edges
            if(flag == 0) continue;

            // if red edge is not part of polyline
            for (auto it = this->poly_line.begin(); it != this->poly_line.end(); ++it) {
                // when the starting point of polyline is the same as the one of the red edge
                // possible visible edges exist
                if (red.source() == it->source() ) {

                    while(red.target() != it->target() && it != this->poly_line.end()) {
                        // create edges connecting the start and end point
                        // of the possible visible edge with the point
                        Segment red1(p, it->source()), red2(p, it->target());

                        // for each polyline edge check if it intersects with the edges created
                        // if at least one polyline intersects the possible visible edge is not visible
                        if (is_vis(red1, red2)) seg.push_back(Segment(it->source(), it->target()));

                        it++;
                        // stop when the end of a polyline edge is the same as of the red edge
                    }

                    Segment red1(p, it->source()), red2(p, it->target());
                    if (is_vis(red1, red2)) seg.push_back(*it);
                    // found all visible edges for this red edge
                    break;
                }
            }
        }

        // if size is 0 convex hull has opposite orientation 
        if (seg.size() == 0){
            int flag = 1;
            for (Segment red : red_edges) {

                for (Segment line : this->poly_line) {
                    if (red.source() == line.target() && red.target() == line.source() ) {
                        flag = 0;
                        seg.push_back(line);
                        break;
                    }
                }

                // if red edge is part of the polyline there no more visible edges
                if(flag == 0) continue;
                for (auto it = this->poly_line.begin(); it != this->poly_line.end(); ++it) {

                    if (red.target() == it->source()) {
                        while(red.source() != it->target()) {
                            // create edges connecting the start and end point
                            // of the possible visible edge with the point
                            Segment red1(p, it->source()), red2(p, it->target());

                            // for each polyline edge check if it intersects with the edges created
                            // if at least one polyline intersects the possible visible edge is not visible
                            if (is_vis(red1, red2)) seg.push_back(*it);
                            it++;
                            // stop when the end of a polyline edge is the same as of the red edge
                        }

                        Segment red1(p, it->source()), red2(p, it->target());
                        if (is_vis(red1, red2)) seg.push_back(*it);
                        // found all visible edges for this red edge
                        break;

                    }
                }
            }
        }

        return seg;
    } catch (...) {
        throw;
    }
}

bool polyline::is_vis(Segment red1, Segment red2) const {
    try {
        for(Segment line : this->poly_line) {
            if ((line.source() == red1.target()) && (line.target() == red2.target())) continue;
            if (!intersection(red1, line) && !intersection(red2, line)) continue;
            if (intersection(red1, line) && intersection(red2, line)) return false;
            CGAL::Object result1 = intersection(red1, line);
            CGAL::Object result2 = intersection(red2, line);
            Point ipoint;
            Point ipoint1;
            Segment iseg;

            // check if the only point that intersects is the common point of the egdes
            if (CGAL::assign(ipoint, result1) && !intersection(red2, line) && ipoint != red1.target())  return false;
            if (CGAL::assign(ipoint, result2) && !intersection(red1, line) && ipoint != red2.target()) return false;
            if (CGAL::assign(iseg, result1) || CGAL::assign(iseg, result2)) return false;
        }

        return true;
    } catch (...) {
        throw;
    }
}

Segment polyline::min_area(std::vector<Segment> vis_edges, int i) const {
    try {
        Segment replaceable;
        double min_area = std::numeric_limits<double>::max();
        for(auto it = vis_edges.begin(); it != vis_edges.end(); ++it) {
            double curr_area = std::abs(CGAL::area(it->source(), this->points[i], it->target()));
            if (curr_area < min_area) {
                min_area = curr_area;
                replaceable = *it;
            }
        }

        return replaceable;
    } catch (...) {
        throw;
    }
}

Segment polyline::max_area(std::vector<Segment> vis_edges, int i) const {
    try {
        Segment replaceable;
        double max_area = std::numeric_limits<double>::min();
        for(auto it = vis_edges.begin(); it != vis_edges.end(); ++it) {
            double curr_area = std::abs(CGAL::area(it->source(), this->points[i], it->target()));
            if (curr_area > max_area) {
                max_area = curr_area;
                replaceable = *it;
            }
        }

        return replaceable;
    } catch (...) {
        throw;
    }
}

void polyline::write_to_file(std::string alg, int time) const {
    try {
        std::ofstream file(this->out_file);
        file << "Polygonization" << std::endl;
        for (Point p : this->pl_points) file << p.x() << " " << p.y() << std::endl;
        for (Segment s : this->poly_line) file << s.source() << " " << s.target() << std::endl;
        file << "Algorithm: " << alg << "_" << this->edge_sel << std::endl;
        file << "Area: " << this->pl_area << std::endl;
        file << "Ratio: " << (this->pl_area / this->ch_area) << std::endl;
        file << "Construction time: " << time << " msec" << std::endl;
        file.close();
    } catch (...) {
        throw;
    }
    return;
}

polyline::polyline(std::vector<std::pair<float, float>> vec, std::string alg, std::string edge_sel, std::string init, std::string out_file): out_file(out_file), init(init) {
    try {
        //initialize points
        for(auto it = vec.begin(); it != vec.end(); ++it) this->points.push_back(Point(it->first, it->second));
        this->edge_sel = std::stoi(edge_sel);
        if (this->edge_sel != 1 && this->edge_sel != 2 && this->edge_sel != 3) throw std::invalid_argument("\'Edge selection\' must be \'1\', \'2\' or \'3\'");
        // run the correct algorithm
        if (!alg.compare("incremental")) {
            if (!init.compare("1a")) this->incremental(1);
            else if (!init.compare("1b")) this->incremental(2);
            else if (!init.compare("2a")) this->incremental(3);
            else if (!init.compare("2b")) this->incremental(4);
            else throw std::invalid_argument("\'Initialization\' must be \'1a\', \'1b\', \'2a\' or \'2b\'");
        } else if (!alg.compare("convex_hull")) {
            this->convex_hull();
        } else throw std::invalid_argument("\'Algorithm\' must be \'incremental\' or \'convex_hull\'");
    } catch (...) {
        throw;
    }
    return;
}


std::vector<Point> polyline::get_pl_points() const {
    return this->pl_points;
}

std::vector<Segment> polyline::get_poly_line() const {
    return this->poly_line;
}

double polyline::get_area(void) const {
    return this->pl_area;
}

double polyline::get_ch_area(void) const {
    return this->ch_area;
}

