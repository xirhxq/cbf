#include "utils.h"
#include "Swarm.hpp"

#include <fstream>
#include <ctime>


int main(int argc, char* argv[]) {
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
