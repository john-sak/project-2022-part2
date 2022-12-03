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
    if (argc != 12) throw std::invalid_argument("Wrong arguments");
    if (strcmp(argv[1], "-i") || strcmp(argv[3], "-o") || strcmp(argv[5], "-algorithm") || strcmp(argv[7], "-L") || ( strcmp(argv[9], "-max") && strcmp(argv[9], "-min")) || (strcmp(argv[10], "-threshold") && strcmp(argv[10], "-annealing")) ) throw std::invalid_argument("Wrong arguments");
    if (strcmp(argv[6], "local_search") && strcmp(argv[6], "simulated_annealing")) throw std::invalid_argument("\'Algorithm\' must be \'local_search\' or \'simulated_annealing\'");
    if (!strcmp(argv[6], "local_search")) {
        if (strcmp(argv[10], "-threshold")) throw std::invalid_argument("Wrong arguments");
        this->alg_param = std::string(argv[11]);
    } else if (!strcmp(argv[6], "simulated_annealing")) {
        if (strcmp(argv[10], "-annealing")) throw std::invalid_argument("Wrong arguments");
        if (strcmp(argv[11], "local") && strcmp(argv[11], "global") && strcmp(argv[11], "subdivision")) throw std::invalid_argument("\'Annealing\' must be \'local\', \'global\'or \'subdivision\'");
        this->alg_param = std::string(argv[11]);    
    }
    
    this->in_file = std::string(argv[2]);
    this->out_file = std::string(argv[4]);
    this->alg = std::string(argv[6]);
    this->L = std::string(argv[8]);
    this->opt = std::string(argv[9]);
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

std::string arguments::get_L(void) const {
    return this->L;
}

// std::string arguments::get_threshold(void) const {
//     return this->threshold;
// }

// std::string arguments::get_annealing(void) const {
//     return this->annealing;
// }


std::vector<std::pair<float, float>> arguments::get_points(void) const {
    return this->points;
}

std::string arguments::get_opt(void) const {
    return this->opt;
}

std::string arguments::get_alg_param(void) const {
    return this->alg_param;
}