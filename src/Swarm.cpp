#include "utils.h"
#include "Swarm.hpp"

#include <fstream>
#include <ctime>
#include <iostream>
#include <string>

#ifndef CBF_BUILD_SOURCE_SHA256
#error "Swarm must be built with an embedded source fingerprint"
#endif


int main(int argc, char* argv[]) {
    if (argc == 2 && std::string(argv[1]) == "--build-source-fingerprint") {
        std::cout << CBF_BUILD_SOURCE_SHA256 << std::endl;
        return 0;
    }

    clock_t start = clock();

    std::string configPath = "../config/config.json";
    if (argc > 1) {
        configPath = argv[1];
    }

    json settings = json::parse(std::ifstream(configPath));
    seedRandomFromConfig(settings, static_cast<unsigned int>(time(NULL)));

    Swarm(settings).run();

    clock_t finish = clock();
    double duration = (double) (finish - start) / CLOCKS_PER_SEC;
    printf("%.4lf seconds passed!\n", duration);
    return 0;
}
