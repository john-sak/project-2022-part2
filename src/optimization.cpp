#include <optimization.hpp>

#include <string>

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
