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
        std::string area;
        double threshold;
        std::string annealing;
        void local_search(void);
        void simulated_annealing(void);
        std::vector<Point> replace_edges(Segment, std::vector<Segment>);
    public:
        optimization(std::vector<Point>, std::vector<Segment>, std::string, std::string, std::string, std::string, std::string);

};

typedef struct update_node{
    Segment e;
    std::vector<Segment> V;
    double area_change;
} update_node;
#endif
