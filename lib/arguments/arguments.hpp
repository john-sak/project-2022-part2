#ifndef ARGUMENTS_HPP
#define ARGUMENTS_HPP

#include <string>
#include <vector>

class arguments {
    private:
        std::string in_file;
        std::string out_file;
        std::string alg;
        std::string L;
        std::string opt;
        // std::string threshold;
        // std::string annealing;
        std::string alg_param;
        std::vector<std::pair<float, float>> points;
        void make_vector(void);
    public:
        arguments(int, char **);
        std::string get_in_file(void) const;
        std::string get_out_file(void) const;
        std::string get_alg(void) const;
        std::string get_L(void) const;
        // std::string get_threshold(void) const;
        // std::string get_annealing(void) const;
        std::vector<std::pair<float, float>> get_points(void) const;
        std::string get_opt(void) const;
        std::string get_alg_param(void) const;
};

#endif
