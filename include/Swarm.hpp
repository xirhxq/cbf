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
    std::string customOutputPath;
    std::string outputDir;
    std::vector<int> all_ids;

    std::unique_ptr<CentralizedModel> centralizedModel;
    std::unique_ptr<OptimiserBase> optimizer;

    MultiCBF cbfNoSlack;
    std::unordered_map<std::string, CBF> cbfSlack;
    json opt;
public:
    Swarm(json &settings)
            : config(settings),
              n(settings["num"]),
              gridWorldGroundTruth(settings["world"]){
        if (settings.contains("output_path")) {
            customOutputPath = settings["output_path"];
        } else {
            customOutputPath = "";
        }

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
        stepData["covariance_formation"] = json::array();
        for (auto &robot: robots) {
            stepData["formation"].push_back(robot->myFormation);
            stepData["covariance_formation"].push_back(robot->myCovarianceFormation);
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
        std::string dataDir;
        std::string runSuffix;

        if (!customOutputPath.empty()) {
            dataDir = customOutputPath;
        } else {
            dataDir = PROJECT_ROOT "/data";
        }

        if (config.contains("run_suffix")) {
            runSuffix = config["run_suffix"].get<std::string>();
        }

        if (mkdir(dataDir.c_str(), 0777) == -1 && errno != EEXIST) {
            std::cerr << "[Swarm::initLog] Warning: Could not create data directory: " << strerror(errno) << std::endl;
        }

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

        std::string outputDirName = folderName + runSuffix;
        outputDir = dataDir + "/" + outputDirName;

        if (mkdir(outputDir.c_str(), 0777) == -1) {
            std::cerr << "Error :  " << strerror(errno) << std::endl;
        }
        filename = outputDir + "/data.json";
        ofstream.open(filename, std::ios::app);
    }

    void exchangeData() {
        for (const auto &robot: robots) {
            Point pos2d = robot->model->xy();
            auto vel2d = robot->model->getVelocity();
            double yawRad = robot->model->getStateVariable("yawRad");
            double batteryLevel = robot->model->getStateVariable("battery");
            Eigen::Matrix2d positionCovariance = robot->positionCovariance;
            double uncertaintyRate = robot->uncertaintyRate;

            for (auto &otherRobot: robots) {
                otherRobot->comm->receivePosition2D(robot->id, pos2d);
                otherRobot->comm->receiveVelocity2D(robot->id, vel2d);
                otherRobot->comm->receiveYawRad(robot->id, yawRad);
                otherRobot->comm->receiveBatteryLevel(robot->id, batteryLevel);
                otherRobot->comm->receivePositionCovariance(robot->id, positionCovariance);
                otherRobot->comm->receiveUncertaintyRate(robot->id, uncertaintyRate);
            }
        };
    }

    // External data injection interface (for simulation environment)
    void injectExternalVelocities(const std::vector<std::pair<double, double>>& velocities) {
        for (size_t i = 0; i < robots.size() && i < velocities.size(); ++i) {
            robots[i]->setExternalVelocity(velocities[i].first, velocities[i].second);
        }
    }

    void checkInformationExchange() {
        for (auto &robot: robots) {
            for (auto &other: robots) {
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

    void checkUpdatedGridWorld() {
        for (auto &robot: robots) {
            if (updatedGridWorldGroundTruth.size() != robot->updatedGridWorld.size()) {
                throw std::runtime_error("updatedGridWorldGroundTruth size mismatch, robot id: " + std::to_string(robot->id));
            }
        }
    }

    void run() {
        auto settings = config["execute"];
        double tTotal = settings["time-total"], tStep = settings["time-step"];

        exchangeData();
        for (auto &robot: robots) robot->updateCovarianceAndRate(tStep);
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

        while (robots[0]->runtime < tTotal) {
            try {
                exchangeData();
                for (auto &robot: robots) robot->updateCovarianceAndRate(tStep);
                exchangeData();
                checkInformationExchange();
                for (auto &robot: robots) robot->checkRobotsInsideWorld();
                printf("\r%.2lf seconds elapsed... %.2lf%%", robots[0]->runtime, gridWorldGroundTruth.getPercentage() * 100);
                for (auto &robot: robots) robot->updateGridWorld();
                updateGridWorld();
                checkUpdatedGridWorld();
                for (auto &robot: robots) robot->postsetCBF();

                if (isCentralizedExecution()) {
                    postsetCBFs();
                    centralizedOptimise();
                } else {
                    for (auto &robot: robots) robot->optimise();
                }

                // if (checkConstraintViolation()) {
                //     logOnce();
                //     std::cout << "\n[Simulation Terminated] Constraint violation detected at t=" << robots[0]->runtime << "s" << std::endl;
                //     break;
                // }
                if (settings.value("check-constraint-violation", false)) {
                    if (checkConstraintViolation()) {
                        logOnce();
                        std::cout << "\n[Simulation Terminated] Constraint violation detected at t=" << robots[0]->runtime << "s" << std::endl;
                        break;
                    }
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
        std::cout << "[OUTPUT_DIR] " << outputDir << std::endl;

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
                    // Use distance from origin for uncertainty calculation
                    Point origin(0, 0);
                    double robotDistFromOrigin = pos1.distance_to(origin);
                    double otherDistFromOrigin = pos2.distance_to(origin);
                    double uncertainty1 = robot->getUncertaintyAtDistance(robotDistFromOrigin);
                    double uncertainty2 = other->getUncertaintyAtDistance(otherDistFromOrigin);
                    double robust_h = h - k * (uncertainty1 + uncertainty2);

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
                    // Use distance from origin for uncertainty calculation
                    Point origin(0, 0);
                    double robotDistFromOrigin = robotPos.distance_to(origin);
                    double uncertainty = robot->getUncertaintyAtDistance(robotDistFromOrigin);
                    double robust_h = h - k * uncertainty;

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
            optimizer = createOptimiser(config["optimiser"], config["cbfs"]["objective-function"]);
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

    bool checkConstraintViolation() {
        // Check h_loc constraints for violation
        // Constraint: distance + uncertainty <= max_range
        // Violation occurs when: distance + uncertainty1 + uncertainty2 > max_range
        // Add buffer for numerical stability

        // Get max_range from config
        auto commConfig = config["cbfs"]["without-slack"]["comm-fixed"];
        if (!commConfig.contains("max-range")) {
            return false;  // No max-range specified, cannot check
        }
        double maxRange = commConfig["max-range"];
        double buffer = maxRange * 0.1;  // 1% buffer for numerical stability
        double violationThreshold = maxRange + buffer;

        // Check all robots and their constrained pairs
        for (auto &robot : robots) {
            // Debug: Check if myFormation is properly set
            if (!robot->myFormation.contains("anchorIds") && !robot->myFormation.contains("baseIds")) {
                throw std::runtime_error("Robot " + std::to_string(robot->id) + " myFormation is empty! anchorIds and baseIds not found.");
            }
            if (robot->myFormation.contains("anchorIds") && robot->myFormation["anchorIds"].size() == 0 &&
                robot->myFormation.contains("baseIds") && robot->myFormation["baseIds"].size() == 0) {
                throw std::runtime_error("Robot " + std::to_string(robot->id) + " myFormation has empty anchorIds and baseIds!");
            }

            Point robotPos = robot->model->xy();
            double robotUncertainty = robot->uncertaintyFromCovarianceFunction(robot->positionCovariance);

            // Check robot-robot constraints (from anchorIds)
            if (robot->myFormation.contains("anchorIds")) {
                for (auto &anchorId : robot->myFormation["anchorIds"]) {
                    int otherId = anchorId.get<int>();
                    if (otherId <= 0 || otherId > n) continue;

                    auto &otherRobot = robots[otherId - 1];
                    Point otherPos = otherRobot->model->xy();
                    double otherUncertainty = otherRobot->uncertaintyFromCovarianceFunction(otherRobot->positionCovariance);

                    double distance = robotPos.distance_to(otherPos);
                    double totalDistance = distance + robotUncertainty + otherUncertainty;

                    if (totalDistance > violationThreshold) {
                        std::cout << "\n[Constraint Violation] Robot " << robot->id
                                  << " - Robot " << otherId
                                  << ": distance=" << distance
                                  << ", uncertainty=" << robotUncertainty << "+" << otherUncertainty
                                  << ", total=" << totalDistance
                                  << " > threshold=" << violationThreshold << " (maxRange=" << maxRange << " + buffer=" << buffer << ")"
                                  << " at t=" << robot->runtime << std::endl;
                        return true;
                    }
                }
            }

            // Check robot-base constraints (from baseIds)
            if (robot->myFormation.contains("baseIds")) {
                json bases = config["bases"];
                for (auto &baseIdx : robot->myFormation["baseIds"]) {
                    int baseId = baseIdx.get<int>();
                    if (baseId < 0 || baseId >= bases.size()) continue;

                    Point basePos(bases[baseId][0], bases[baseId][1]);
                    double distance = robotPos.distance_to(basePos);

                    // Base has no uncertainty, only robot uncertainty
                    double totalDistance = distance + robotUncertainty;

                    if (totalDistance > violationThreshold) {
                        std::cout << "\n[Constraint Violation] Robot " << robot->id
                                  << " - Base " << baseId
                                  << ": distance=" << distance
                                  << ", uncertainty=" << robotUncertainty
                                  << ", total=" << totalDistance
                                  << " > threshold=" << violationThreshold << " (maxRange=" << maxRange << " + buffer=" << buffer << ")"
                                  << " at t=" << robot->runtime << std::endl;
                        return true;
                    }
                }
            }
        }

        return false;
    }
};


#endif //CBF_SWARM_HPP
