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

    std::unique_ptr<CentralizedModel> centralizedModel;
    std::unique_ptr<Gurobi> optimizer;

    MultiCBF cbfNoSlack;
    std::unordered_map<std::string, CBF> cbfSlack;
    json opt;
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

        initializeCentralizedOptimization();
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
        stepData["formation"] = json::array();
        for (auto &robot: robots) {
            stepData["formation"].push_back(robot->myFormation);
        }
        stepData["update"] = updatedGridWorldGroundTruth;

        if (isCentralizedExecution()) {
            json centralizedData = opt;

            json cbfValues = json::object();

            for (const auto& [name, cbf] : cbfNoSlack.cbfs) {
                auto x = centralizedModel->getX();
                double value = cbf.h(x, robots[0]->runtime);
                cbfValues[name] = value;
            }

            for (const auto& [name, cbf] : cbfSlack) {
                auto x = centralizedModel->getX();
                double value = cbf.h(x, robots[0]->runtime);
                cbfValues[name] = value;
            }

            centralizedData["cbfs"] = cbfValues;
            stepData["centralized"] = centralizedData;
        }

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
            auto vel2d = robot->model->getVelocity();
            double yawRad = robot->model->getStateVariable("yawRad");
            double batteryLevel = robot->model->getStateVariable("battery");
            for (auto &otherRobot: robots) {
                if (robot->id == otherRobot->id) continue;
                otherRobot->comm->receivePosition2D(robot->id, pos2d);
                otherRobot->comm->receiveVelocity2D(robot->id, vel2d);
                otherRobot->comm->receiveYawRad(robot->id, yawRad);
                otherRobot->comm->receiveBatteryLevel(robot->id, batteryLevel);
            }
        };
    }

    void checkInformationExchange() {
        for (auto &robot: robots) {
            for (auto &other: robots) {
                if (robot->id == other->id) continue;
                assert(robot->model->xy().distance_to(other->comm->_othersPos[robot->id]) < 1e-3);
            }
        }
    }

    void updateGridWorld() {
        updatedGridWorldGroundTruth = json::array();
        json searchSettings = config["searching"];
        std::string method = searchSettings["method"];
        json params = searchSettings[method];
        for (auto &robot: robots) {
            auto updatedFor1 = json::array();
            if (method == "front-sector") {
                params["centerAngleRad"] = robot->model->getStateVariable("yawRad");
                updatedFor1 = gridWorldGroundTruth.setValueInSectorRing(
                    robot->model->xy(),
                    params,
                    true, true
                );
            }
            else if (method == "front-cone") {
                params["yaw-rad"] = robot->model->getStateVariable("yawRad");
                updatedFor1 = gridWorldGroundTruth.setValueInTiltedCone(
                    robot->model->xy(),
                    params,
                    true, true
                );
            }
            else if (method == "downward") {
                updatedFor1 = gridWorldGroundTruth.setValueInCircle(
                    robot->model->xy(),
                    params, true, true
                );
            }
            updatedGridWorldGroundTruth.insert(
                updatedGridWorldGroundTruth.end(),
                updatedFor1.begin(), updatedFor1.end()
            );
        }
    }

    void run() {
        exchangeData();
        checkInformationExchange();
        initLog();
        logParams();
        output();

        if (isCentralizedExecution()) {
            initializeCentralizedOptimization();
            presetCBFs();
        } else {
            for (auto &robot: robots) robot->presetCBF();
        }

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

                if (isCentralizedExecution()) {
                    postsetCBFs();
                    centralizedOptimise();
                } else {
                    for (auto &robot: robots) robot->optimise();
                }

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

private:
    void presetCBFs() {
        if (config["cbfs"]["without-slack"]["comm-fixed"]["on"]) setupCommCBF();
    }

    void postsetCBFs() {
        if (config["cbfs"]["with-slack"]["cvt"]["on"]) setupCVTCBF();
    }
    
    void setupCommCBF() {
        auto commConfig = config["cbfs"]["without-slack"]["comm-fixed"];
        if (!commConfig["on"]) return;

        double maxRange = commConfig["max-range"];
        double k = commConfig["k"];
        int maxOffset = commConfig["max-neighbour-id-offset"];
        int minOffset = commConfig["min-neighbour-id-offset"];

        for (auto &robot : robots) {
            robot->postsetCBF();
            std::vector<int> anchorIds;
            if (robot->myFormation.contains("anchorIds")) {
                for (auto &id : robot->myFormation["anchorIds"]) {
                    anchorIds.push_back(id);
                }
            }

            std::vector<Point> formationPoints;
            if (robot->myFormation.contains("anchorPoints")) {
                for (auto &point : robot->myFormation["anchorPoints"]) {
                    formationPoints.emplace_back(point[0], point[1]);
                }
            }

            for (int anchorId : anchorIds) {
                if (anchorId >= robot->id) continue;

                auto &other = robots[anchorId - 1];
                CBF commCBF;
                commCBF.name = "commCBF_" + std::to_string(robot->id) + "_" + std::to_string(other->id);

                double alpha_coe = commConfig["alpha"]["coe"];
                int alpha_pow = commConfig["alpha"]["pow"];
                commCBF.setAlphaClassK(alpha_coe, alpha_pow);

                auto commFunc = [this, &robot, &other, maxRange, k](VectorXd x, double t) -> double {
                    Eigen::VectorXd robot1State = centralizedModel->getRobotStateFromX(x, robot->id);
                    Eigen::VectorXd robot2State = centralizedModel->getRobotStateFromX(x, other->id);

                    Point pos1 = robot->model->extractXYFromVector(robot1State);
                    Point pos2 = other->model->extractXYFromVector(robot2State);

                    double distance = pos1.distance_to(pos2);
                    double h = k * (maxRange - distance);

                    // Robust CBF: ĥ = h - lε, where l = k for comm CBF
                    double robust_h = h - k * (robot->uncertainty + other->uncertainty);

                    return robust_h;
                };
                commCBF.h = commFunc;

                cbfNoSlack.cbfs[commCBF.name] = commCBF;
            }

            for (int i = 0; i < formationPoints.size(); ++i) {
                Point anchorPoint = formationPoints[i];
                CBF anchorCBF;
                anchorCBF.name = "anchorCBF_" + std::to_string(robot->id) + "_" + std::to_string(i);

                double alpha_coe = commConfig["alpha"]["coe"];
                int alpha_pow = commConfig["alpha"]["pow"];
                anchorCBF.setAlphaClassK(alpha_coe, alpha_pow);

                auto anchorFunc = [this, &robot, anchorPoint, maxRange, k](VectorXd x, double t) -> double {
                    Eigen::VectorXd robotState = centralizedModel->getRobotStateFromX(x, robot->id);

                    Point robotPos = robot->model->extractXYFromVector(robotState);

                    double distance = robotPos.distance_to(anchorPoint);
                    double h = k * (maxRange - distance);

                    // Robust CBF: ĥ = h - lε, where l = k for anchor CBF
                    // Anchor points have no uncertainty, only robot uncertainty
                    double robust_h = h - k * robot->uncertainty;

                    return robust_h;
                };
                anchorCBF.h = anchorFunc;

                cbfNoSlack.cbfs[anchorCBF.name] = anchorCBF;
            }
        }
    }

    void setupCVTCBF() {
        auto cvtConfig = config["cbfs"]["with-slack"]["cvt"];
        if (!cvtConfig["on"]) return;

        CVT tempCVT = CVT(n, robots[0]->world.boundary);
        for (auto &robot : robots) {
            tempCVT.pt[robot->id] = robot->model->xy();
        }
        tempCVT.cal_poly();
        for (int i = 1; i <= tempCVT.n; i++) {
            tempCVT.ct[i] = robots[0]->gridWorld.getCentroidInPolygon(tempCVT.pl[i]);
        }

        double alpha_coe = cvtConfig["alpha"]["coe"];
        int alpha_pow = cvtConfig["alpha"]["pow"];
        double kp = cvtConfig["kp"];

        CBF cvtCBF;
        cvtCBF.name = "cvtCBF";
        cvtCBF.setAlphaClassK(alpha_coe, alpha_pow);

        auto cvtFunc = [this, tempCVT, kp](VectorXd x, double t) -> double {
            double totalDistance = 0.0;
            for (auto &robot : robots) {
                int robotIdx = robot->id - 1;
                int stateOffset = centralizedModel->getStateOffset(robotIdx);
                int stateSize = centralizedModel->stateSizes[robotIdx];
                Eigen::VectorXd robotState = x.segment(stateOffset, stateSize);

                Point robotPos = robot->model->extractXYFromVector(robotState);

                Point cvtCenter = tempCVT.ct[robot->id];
                totalDistance += cvtCenter.distance_to(robotPos);
            }
            return -kp * totalDistance;
        };
        cvtCBF.h = cvtFunc;

        cbfSlack[cvtCBF.name] = cvtCBF;
    }


    void initializeCentralizedOptimization() {
        centralizedModel = std::make_unique<CentralizedModel>();
        for (auto &robot : robots) {
            centralizedModel->addRobot(robot->model.get());
        }

        if (config.contains("cbfs") && config["cbfs"].contains("objective-function")) {
            optimizer = std::make_unique<Gurobi>(config["cbfs"]["objective-function"]);
        } else {
            printf("Warning: missing objective-function config\n");
        }
    }

    void centralizedOptimise() {
        if (!optimizer || !centralizedModel) {
            return;
        }

        centralizedModel->updateConcatenatedStates();

        Eigen::VectorXd uNominal(centralizedModel->getTotalControlSize());
        uNominal.setZero();

        opt = {
            {"nominal",    json::array()},
            {"result",     json::array()},
            {"cbfNoSlack", json::array()},
            {"cbfSlack",   json::array()}
        };
        json jsonCBFNoSlack = json::array(), jsonCBFSlack = json::array();

        optimizer->clear();

        auto f = centralizedModel->f();
        auto g = centralizedModel->g();
        auto x = centralizedModel->getX();

        int uSize = centralizedModel->getTotalControlSize();
        int slackSize = cbfSlack.size();
        int totalSize = uSize + slackSize;

        optimizer->start(totalSize, uSize);
        optimizer->setObjective(uNominal);

        opt["nominal"] = convertCentralizedControlToJson(uNominal);

        std::string cbf_method = config["cbfs"]["without-slack"].value("method", "all");
        if (cbf_method == "all") {
            for (auto &[name, cbf] : cbfNoSlack.cbfs) {
                VectorXd uCoe = cbf.constraintUCoe(f, g, x, robots[0]->runtime);
                double constraintConstWithTime = cbf.constraintConstWithTime(f, g, x, robots[0]->runtime);
                optimizer->addLinearConstraint(uCoe, -constraintConstWithTime);

                jsonCBFNoSlack.emplace_back(json{
                    {"name",  cbf.name},
                    {"coe",   convertCentralizedControlToJson(uCoe)},
                    {"const", constraintConstWithTime}
                });
            }
        }

        int cnt = 0;
        for (auto &[name, cbf] : cbfSlack) {
            VectorXd uCoe = cbf.constraintUCoe(f, g, x, robots[0]->runtime);
            Eigen::VectorXd sCoe = Eigen::VectorXd::Zero(slackSize);
            sCoe(cnt) = 1.0;
            Eigen::VectorXd coe(totalSize);
            coe << uCoe, sCoe;
            double constraintConst = cbf.constraintConstWithoutTime(f, g, x, robots[0]->runtime);

            optimizer->addLinearConstraint(coe, -constraintConst);

            jsonCBFSlack.emplace_back(json{
                {"name",  cbf.name},
                {"coe",   convertCentralizedControlToJson(uCoe)},
                {"const", constraintConst}
            });
            ++cnt;
        }

        opt["cbfNoSlack"] = jsonCBFNoSlack;
        opt["cbfSlack"] = jsonCBFSlack;

        auto result = optimizer->solve();

        optimizer->write("centralized_optimization.lp");

        auto u = result.head(uSize);
        opt["result"] = convertCentralizedControlToJson(u);
        opt["slacks"] = convertCentralizedSlackToJson(result.tail(slackSize));

        for (int i = 0; i < n; ++i) {
            centralizedModel->setRobotControl(i, result);
        }
    }

    bool isCentralizedExecution() const {
        auto executeConfig = config["execute"];
        if (executeConfig.contains("execution-mode")) {
            std::string mode = executeConfig["execution-mode"];
            return mode == "centralized";
        }
        return false;
    }

    json convertCentralizedControlToJson(const VectorXd& u) {
        json result = json::array();
        for (int i = 0; i < robots.size(); ++i) {
            int controlOffset = centralizedModel->getControlOffset(i);
            int controlSize = centralizedModel->controlSizes[i];
            Eigen::VectorXd robotControl = u.segment(controlOffset, controlSize);

            json robotControlJson = json::array();
            for (int j = 0; j < robotControl.size(); ++j) {
                robotControlJson.push_back(robotControl(j));
            }
            result.push_back(robotControlJson);
        }
        return result;
    }

    json convertCentralizedSlackToJson(const VectorXd& slack) {
        json result = json::array();
        for (int i = 0; i < slack.size(); ++i) {
            result.push_back(slack(i));
        }
        return result;
    }
};


#endif //CBF_SWARM_HPP