#include "utils.h"
#include "Swarm.hpp"

#include <fstream>
#include <ctime>


int main(int argc, char* argv[]) {
    clock_t start = clock();

    return runSimulationWithErrorGate([&]() {
        std::string configPath = "../config/config.json";
        if (argc > 1) {
            configPath = argv[1];
        }

        json settings = json::parse(std::ifstream(configPath));
        seedRandomFromConfig(settings, static_cast<unsigned int>(time(NULL)));
        const bool evidenceMode = settings.contains("evidence-stream")
            && settings.at("evidence-stream").is_object()
            && settings.at("evidence-stream").value("enabled", false);

        Swarm(settings).run();

        clock_t finish = clock();
        double duration = (double) (finish - start) / CLOCKS_PER_SEC;
        if (evidenceMode) {
            std::cerr << std::fixed << std::setprecision(4)
                      << duration << " seconds passed!\n";
        } else {
            printf("%.4lf seconds passed!\n", duration);
        }
    });
}
