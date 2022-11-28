#include <string>
#include <vector>
#include <stdexcept>
#include <fstream>

#include <string.h>

#include <arguments.hpp>

void arguments::make_vector(void) {
    std::ifstream file;
    try {
        file.open(this->in_file);
        if (!file.is_open()) throw std::invalid_argument("Error opening file \'" + this->in_file + "\'");
        std::string line;
        while (std::getline(file, line)) {
            // ignore comments
            if (line.at(0) == '#') continue;
            std::string str = line.substr(line.find("\t") + 1, line.length() - 1);
            // save points as pair of 2 floating point numbers
            this->points.push_back({std::stof(str.substr(0, str.find("\t"))), std::stof(str.substr(str.find("\t") + 1, str.length() - 1))});
        }
    } catch (...) {
        file.close();
        throw;
    }
    file.close();
    return;
}

arguments::arguments(int argc, char *argv[]) {
    if (argc < 9) throw std::invalid_argument("Too few arguments");
    if (strcmp(argv[1], "-i") || strcmp(argv[3], "-o") || strcmp(argv[5], "-algorithm") || strcmp(argv[7], "-edge_selection")) throw std::invalid_argument("Wrong arguments");
    if (strcmp(argv[6], "incremental") && strcmp(argv[6], "convex_hull")) throw std::invalid_argument("\'Algorithm\' must be \'incremental\' or \'convex_hull\'");
    if (strcmp(argv[8], "1") && strcmp(argv[8], "2") && strcmp(argv[8], "3")) throw std::invalid_argument("\'Edge selection\' must be \'1\', \'2\' or \'3\'");
    if (!strcmp(argv[6], "incremental")) {
        if (argc != 11) throw std::invalid_argument("\'Incremental algorithm\' also needs \'initialization\' argument");
        if (strcmp(argv[9], "-initialization")) throw std::invalid_argument("Wrong arguments");
        if (strcmp(argv[10], "1a") && strcmp(argv[10], "1b") && strcmp(argv[10], "2a") && strcmp(argv[10], "2b")) throw std::invalid_argument("\'Initialization\' must be \'1a\', \'1b\', \'2a\' or \'2b\'");
        this->init = std::string(argv[10]);
    } else if (argc != 9) throw std::invalid_argument("Too many arguments");
    this->in_file = std::string(argv[2]);
    this->out_file = std::string(argv[4]);
    this->alg = std::string(argv[6]);
    this->edge_sel = std::string(argv[8]);
    try {
        make_vector();
    } catch (...) {
        throw;
    }
    return;
}

std::string arguments::get_in_file(void) const {
    return this->in_file;
}

std::string arguments::get_out_file(void) const {
    return this->out_file;
}

std::string arguments::get_alg(void) const {
    return this->alg;
}

std::string arguments::get_edge_sel(void) const {
    return this->edge_sel;
}

std::string arguments::get_init(void) const {
    return this->init;
}

std::vector<std::pair<float, float>> arguments::get_points(void) const {
    return this->points;
}
