#include "utils.h"
//#define TIMER_ON
#include "Swarm.hpp"

#include <fstream>
#include <ctime>


std::vector<Point> getPointVectorFromJson(json j) {
    std::vector<Point> res;
    for (auto a: j) {
        res.emplace_back(Point(a["x"], a["y"]));
    }
    return res;
}


int main() {
    clock_t start = clock();

    std::ifstream f("../launch/main.json");
    json data = json::parse(f);
    time_t tt = time(NULL);
    tm *t = localtime(&tt);

    mkdir("../data", 0777);

    char folderName[256];
    snprintf(folderName, 256, R"(../data/%02d-%02d_%02d-%02d)",
            t->tm_mon + 1, t->tm_mday, t->tm_hour, t->tm_min);

    if (mkdir(folderName, 0777) == -1) {
        std::cerr << "Error :  " << strerror(errno) << std::endl;
    } else {
        std::cout << "Directory " << folderName << " created" << std::endl;
    }

    char fileName[256];
    snprintf(fileName, 256, R"(%s/%s)", folderName, "data.json");


    Polygon worldBoundary(Polygon(getPointVectorFromJson(data["world"])));
    auto c = getPointVectorFromJson(data["charge"]);


    World world(worldBoundary, c);
//    world.targets.emplace_back(Target::makeStaticTarget({10, 5}));
//    world.targets[0].visibleTimeRanges = {{0, 15}, {30, 40}};
//    world.targets.emplace_back(
//            Target::makeLoopRectangleTarget(
//                    {4, 3},
//                    {16, 7},
//                    0.5, 32
//            )
//    );
//    world.targets[1].visibleTimeRanges = {{10, 22}};
//    world.targets.emplace_back(
//            Target::makeLoopRectangleTarget(
//                    {4, 3},
//                    {16, 7},
//                    0.5
//            )
//    );

    Swarm s = Swarm(data["swarm"]["num"], world);

    if (data["swarm"]["initial_position"] == "random_in_poly") {
        s.randomInitialPosition(Polygon(getPointVectorFromJson(data["swarm"]["random_poly"])));
    } else if (data["swarm"]["initial_position"] == "specified") {
        auto v = getPointVectorFromJson(data["swarm"]["specified_pos"]);
        s.setInitialPosition(v);
    }
    else if (data["swarm"]["initial_position"] == "random_all"){
        s.randomInitialPosition();
    }


    if (data["cbfs"]["energy_cbf"] == "on") {
        s.setEnergyCBF();
    }
    if (data["cbfs"]["camera_cbf"] == "on") {
        s.setYawCBF();
    }
    s.initLogPath(fileName);
    s.logParams();

    s.output();
#ifdef TIMER_ON
    clock_t begin, end;
#endif

    double tTotal = data["execute"]["time_total"], tStep = data["execute"]["step_time"];
    for (int iter = 1; iter <= tTotal / tStep; iter++) {
#ifdef TIMER_ON
        double gridWorldUpdatedTime, cvtForwardTime, logTime, setCVTTime = 0.0, calCVTTime = 0.0;
#endif
        if (data["execute"]["output_grid"] == "off") {
            printf("\r%.2lf seconds elapsed...", tStep * (iter - 1));
        } else {
            printf("%.2lf seconds elapsed...\n", tStep * (iter - 1));
        }
        if (data["cbfs"]["comm_cbf"] == "on"){
            s.setCommCBF();
        }
#ifdef TIMER_ON
        begin = clock();
#endif
        s.updateGridWorld();
#ifdef TIMER_ON
        end = clock();
        gridWorldUpdatedTime = (double) (end - begin) / CLOCKS_PER_SEC;
#endif
        if (data["cbfs"]["cvt_cbf"] == "on") {
#ifdef TIMER_ON
            begin = clock();
#endif
            s.calCVT();
#ifdef TIMER_ON
            end = clock();
            calCVTTime += (double) (end - begin) / CLOCKS_PER_SEC;
            begin = clock();
#endif
            s.setCVTCBF();
//            s.set_cvt_cbf();
#ifdef TIMER_ON
            end = clock();
            setCVTTime += (double) (end - begin) / CLOCKS_PER_SEC;
#endif
        }
        if (data["cbfs"]["safety_cbf"] == "on") {
            s.setSafetyCBF();
        }
#ifdef TIMER_ON
        begin = clock();
#endif
        s.logOnce();
#ifdef TIMER_ON
        end = clock();
        logTime = (double) (end - begin) / CLOCKS_PER_SEC;
        begin = clock();
#endif
        s.cvtForward(tStep);
#ifdef TIMER_ON
        end = clock();
        cvtForwardTime = (double) (end - begin) / CLOCKS_PER_SEC;
        printf("GridWorldUpdatedTime: %.4lf, CVTForwardTime: %.4lf, LogTime: %.4lf, SetCVTTime: %.4lf, CalCVTTime: %.4lf\n",
               gridWorldUpdatedTime, cvtForwardTime, logTime, setCVTTime, calCVTTime);
#endif
        if (data["execute"]["output_grid"] == "on") {
            s.gridWorldOutput();
        }
    }

    printf("\nAfter %.4lf seconds\n", tTotal);
//    s.gw.output();
    s.output();
    s.endLog();
    printf("Data saved in %s\n", fileName);

    clock_t finish = clock();
    double duration = (double) (finish - start) / CLOCKS_PER_SEC;
    printf("%.4lf seconds passed!\n", duration);
    return 0;
}