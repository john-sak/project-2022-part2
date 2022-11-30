// to_polygon.cpp
#include <iostream>
#include <string>

#include <arguments.hpp>
#include <polyline.hpp>


int main(int argc, char *argv[]) {

    const std::string POLY_ALGO = "incremental";
    const std::string EDDGE_SEL = "1";
    const std::string INIT = "1a";

    try {
        // get arguments from command line
        arguments arg(argc, argv);
        // run the given algorithm, write results in given out_file
        polyline S(arg.get_points(), POLY_ALGO, EDDGE_SEL, INIT, arg.get_out_file());
        // S.print_points();

    } catch (std::invalid_argument const &ex) {
        std::cerr << ex.what() << std::endl;
        std::cerr << "Usage: ./optimal_polygon -i <input_file> -o <output_file> -algorithm <\'local_search\' OR \'simulated_annealing\'> -L  < L parameter >  -max [maximal area polygonization]  -min [minimal area polygonization]  -threshold < double > | with \'local_search\' algorithm only> -annealing < \'local\' OR \'global\' OR \'subdivision\' > | with \'simulated_annealing\' algorithm only" << std::endl;
        return -1;
    } catch (std::exception const &ex) {
        std::cerr << ex.what() << std::endl;
        return -1;
    }
    return 0;
}
