#ifndef ARGUMENTS_HPP
#define ARGUMENTS_HPP

#include <string>
#include <vector>

class arguments {
    private:
        std::string in_file;
        std::string out_file;
        std::string alg;
        std::string edge_sel;
        std::string init;
        std::vector<std::pair<float, float>> points;
        void make_vector(void);
    public:
        arguments(int, char **);
        std::string get_in_file(void) const;
        std::string get_out_file(void) const;
        std::string get_alg(void) const;
        std::string get_edge_sel(void) const;
        std::string get_init(void) const;
        std::vector<std::pair<float, float>> get_points(void) const;
};

#endif
