#include "utils.h"
#include "Swarm.hpp"

#include <fstream>
#include <ctime>


int main() {
    clock_t start = clock();

    json settings = json::parse(std::ifstream("../config/config.json"));

    Swarm(settings).run();

    clock_t finish = clock();
    double duration = (double) (finish - start) / CLOCKS_PER_SEC;
    printf("%.4lf seconds passed!\n", duration);
    return 0;
}