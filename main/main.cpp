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
        std::cerr << "Usage: ./to_polygon -i <input_file> -o <output_file> -algorithm <\'incremental\' OR \'convex_hull\'> -edge_selection <\'1\' OR \'2\' OR \'3\'> -initialization <\'1a\' OR \'1b\' OR \'2a\' OR \'2b\' | with \'incremental\' algorithm only>" << std::endl;
        return -1;
    } catch (std::exception const &ex) {
        std::cerr << ex.what() << std::endl;
        return -1;
    }
    return 0;
}
