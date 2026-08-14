#include "bridge/ControlledJointMarginBenchmark.hpp"

#include <exception>
#include <iostream>
#include <stdexcept>
#include <string>

#ifndef CBF_BUILD_SOURCE_SHA256
#define CBF_BUILD_SOURCE_SHA256 "missing"
#endif

namespace {

ControlledJointMarginPressure parsePressure(const std::string &value) {
    if (value == "low") {
        return ControlledJointMarginPressure::Low;
    }
    if (value == "critical") {
        return ControlledJointMarginPressure::Critical;
    }
    throw std::invalid_argument("pressure must be low or critical");
}

ControlledJointMarginController parseController(const std::string &value) {
    if (value == "hard-qp") {
        return ControlledJointMarginController::HardQp;
    }
    if (value == "margin-feedback") {
        return ControlledJointMarginController::MarginFeedback;
    }
    throw std::invalid_argument(
            "controller must be hard-qp or margin-feedback");
}

}  // namespace

int main(int argc, char **argv) {
    try {
        if (argc != 7
            || std::string(argv[1]) != "--pressure"
            || std::string(argv[3]) != "--controller"
            || std::string(argv[5]) != "--lateral-offset") {
            throw std::invalid_argument(
                    "usage: ControlledJointMarginBenchmark --pressure "
                    "{low,critical} --controller {hard-qp,margin-feedback} "
                    "--lateral-offset {-0.5,-0.25,0,0.25,0.5}");
        }
        const ControlledJointMarginSpec spec{
                parsePressure(argv[2]),
                parseController(argv[4]),
                std::stod(argv[6]),
        };
        const auto result = runControlledJointMarginCell(spec);
        std::cout << controlledJointMarginResultJson(
                result, CBF_BUILD_SOURCE_SHA256).dump() << '\n';
        return 0;
    } catch (const std::exception &error) {
        std::cerr << error.what() << '\n';
        return 2;
    }
}
