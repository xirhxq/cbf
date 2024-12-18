#ifndef CBF_SWARM_HPP
#define CBF_SWARM_HPP

#include "Robot.hpp"


class Swarm {
public:
    int n;
    std::vector<std::unique_ptr<Robot>> robots;
    std::ofstream ofstream;
    json data;
    json stepData;
    json config;
    GridWorld gridWorldGroundTruth;
    json updatedGridWorldGroundTruth;
    std::string folderName;
    std::string filename;
    std::vector<int> all_ids;
public:
    Swarm(json &settings)
            : config(settings),
              n(settings["num"]),
              gridWorldGroundTruth(settings["world"]){
        std::generate_n(std::back_inserter(all_ids), n, [i = 1]() mutable {
            return i++;
        });
        settings["all"] = all_ids;
        for (int i = 0; i < n; i++) {
            auto robot = std::make_unique<Robot>(i + 1, settings);
            robots.push_back(std::move(robot));
        }
    }

    void output() {
        printf("An Swarm with %d robots @ time %.4lf: ---------\n", n, robots[0]->runtime);
        for (auto &robot: robots) {
            robot->output();
        }
        printf("--------------\n");
    }

    void endLog() {
        ofstream << std::fixed << std::setprecision(6) << data;
        ofstream.close();
    }

    void logParams() {
        data["para"] = robots[0]->getParams();
        data["config"] = config;
    }

    void logOnce() {
        stepData["runtime"] = robots[0]->runtime;
        {
            json robotsJson = json::array();
            for (auto &robot: robots) {
                robotsJson.emplace_back(robot->getState());
            }
            stepData["robots"] = robotsJson;
        }
        if (config["cbfs"]["without-slack"]["comm-fixed"]["on"] || config["cbfs"]["without-slack"]["comm-auto"]["on"]) {
            stepData["formation"] = json::array();
            for (auto &robot: robots) {
                stepData["formation"].push_back(robot->myFormation);
            }
        }
        stepData["update"] = updatedGridWorldGroundTruth;
        data["state"].push_back(stepData);
        stepData.clear();
    }

    void initLog() {
        mkdir("../data", 0777);
        time_t now = time(nullptr);
        tm *t = localtime(&now);
        std::ostringstream oss;
        oss << std::setfill('0')
            << std::setw(4) << t->tm_year + 1900 << "-"
            << std::setw(2) << t->tm_mon + 1 << "-"
            << std::setw(2) << t->tm_mday << "_"
            << std::setw(2) << t->tm_hour << "-"
            << std::setw(2) << t->tm_min << "-"
            << std::setw(2) << t->tm_sec;
        folderName = oss.str();
        if (mkdir(("../data/" + folderName).c_str(), 0777) == -1) {
            std::cerr << "Error :  " << strerror(errno) << std::endl;
        }
        filename = "../data/" + folderName + "/data.json";
        ofstream.open(filename, std::ios::app);
    }

    void exchangeData() {
        for (const auto &robot: robots) {
            Point pos2d = robot->model->xy();
            for (auto &otherRobot: robots) {
                if (robot->id == otherRobot->id) continue;
                otherRobot->comm->receivePosition2D(robot->id, pos2d);
            }
        };
    }

    void checkInformationExchange() {
        for (auto &robot: robots) {
            for (auto &other: robots) {
                if (robot->id == other->id) continue;
                assert(robot->model->xy().distance_to(other->comm->position2D[robot->id]) < 1e-3);
            }
        }
    }

    void updateGridWorld() {
        updatedGridWorldGroundTruth = json::array();
        double tol = 2;
        for (auto &robot: robots) {
            auto updatedFor1 = gridWorldGroundTruth.setValueInCircle(robot->model->xy(), tol, true, true);
            updatedGridWorldGroundTruth.insert(updatedGridWorldGroundTruth.end(), updatedFor1.begin(), updatedFor1.end());
        }
    }

    void run() {
        exchangeData();
        checkInformationExchange();
        initLog();
        logParams();
        output();

        for (auto &robot: robots) robot->presetCBF();

        auto settings = config["execute"];

        double tTotal = settings["time-total"], tStep = settings["time-step"];
        while (robots[0]->runtime < tTotal) {
            try {
                exchangeData();
                checkInformationExchange();
                for (auto &robot: robots) robot->checkRobotsInsideWorld();
                printf("\r%.2lf seconds elapsed... %.2lf%%", robots[0]->runtime, gridWorldGroundTruth.getPercentage() * 100);
                for (auto &robot: robots) robot->updateGridWorld();
                updateGridWorld();
                for (auto &robot: robots) robot->postsetCBF();
                for (auto &robot: robots) robot->optimise();
                logOnce();
                for (auto &robot: robots) robot->stepTimeForward(tStep);
            }
            catch (...) {
                for (auto &robot: robots) robot->updateGridWorld();
                logOnce();
                endLog();
                printf("Data so far has been saved in %s\n", filename.c_str());
                try {
                    throw;
                }
                catch (std::exception &e) {
                    std::cerr << e.what() << std::endl;
                }
                catch (...) {
                    std::cerr << "Unknown error" << std::endl;
                }
                break;
            }
        }

        printf("\nAfter %.4lf seconds\n", robots[0]->runtime);
        output();
        endLog();

        printf("Data saved in %s\n", filename.c_str());

    }
};


#endif //CBF_SWARM_HPP