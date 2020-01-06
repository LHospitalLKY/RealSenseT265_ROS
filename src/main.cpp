//
// Created by LHospital
//

#include <gtest/gtest.h>

#include "../include/RealSense/RealSenseT265_node.h"

// #define DEBUG

int main(int argc, char *argv[]) try {

    ros::init(argc, argv, "RST265_node");
    RealSenseT265 rsT265;
    rsT265.run();
    
}
catch (const rs2::error & e) {
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch (const std::exception& e) {
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}