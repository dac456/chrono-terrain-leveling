#ifndef __CONFIG_HPP
#define __CONFIG_HPP

#include "CtlCommon.hpp"

#include <boost/program_options.hpp>
namespace po = boost::program_options;

class Config{
private:
    po::variables_map _vm;

public:
    Config(std::string configFile){
        po::options_description desc("Configuration Options");
        desc.add_options()
            ("vehicle_start.x", po::value<double>())
            ("vehicle_start.y", po::value<double>())
            ("vehicle_start.z", po::value<double>())
            ("vehicle_start.r", po::value<double>())
            ("vehicle_start.p", po::value<double>())
            ("vehicle_start.h", po::value<double>())

            ("map.filename", po::value<std::string>())
            ("map.scale", po::value<double>())

            ("raygrid.resolution", po::value<double>())

            ("experiment.linear", po::value<double>())
            ("experiment.angular", po::value<double>())
        ;

        _vm = po::variables_map();

        std::ifstream cfg(configFile);

        po::store(po::parse_config_file(cfg , desc), _vm);
        po::notify(_vm);
    }

    po::variables_map getVariables(){
        return _vm;
    }

};

#endif
