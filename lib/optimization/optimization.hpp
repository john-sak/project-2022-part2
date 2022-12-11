#ifndef OPTIMIZATION_HPP
#define OPTIMIZATION_HPP

#include <string>
#include <vector>

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Polygon_2.h>

typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef K::Point_2 Point;
typedef CGAL::Polygon_2<K> Polygon;
typedef K::Segment_2 Segment;


class optimization {
    private:
        std::vector<Point> pl_points;
        std::vector<Segment> poly_line;
        int L;
        std::string out_file;
        std::string opt;
        double threshold;
        std::string annealing;
        double start_area;
        double end_area;
        double ch_area;
        void local_search(void);
        void simulated_annealing_local(void);
        std::vector<Point> simulated_annealing_global(std::vector<Point>);
        void simulated_annealing_subdivision(void);
        std::vector<Point> replace_edges(Segment, std::vector<Segment>);
        std::vector<Segment> get_segment(std::vector<Point>);
        std::vector<Point> get_ch(std::vector<Point>);
        void write_to_file(std::string, int) const;
    public:
        optimization(std::vector<Point>, std::vector<Segment>, std::string, std::string, std::string, std::string, std::string, double, double);

};

typedef struct update_node{
    Segment e;
    std::vector<Segment> V;
    double area_change;
} update_node;
#endif
