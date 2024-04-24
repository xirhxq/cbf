#include "utils.h"
#include "Swarm.hpp"

#include <fstream>
#include <ctime>

std::vector<Point> get_point_vector_from_json(json j) {
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


    Polygon world_poly(Polygon(get_point_vector_from_json(data["world"])));
    auto c = get_point_vector_from_json(data["charge"]);


    World wd(world_poly, c);
//    wd.charge_place[0].second = 101.0;
//    wd.target.emplace_back(
//            Target::make_static_target(
//                    {10, 5}
//                    )
//    );
//    wd.target[0].vis_time = {{0, 15}, {30, 40}};
//    wd.target.emplace_back(
//            Target::make_loop_rectangle_target(
//                    {4, 3},
//                    {16, 7},
//                    0.5, 32
//            )
//    );
//    wd.target[1].vis_time = {{10, 22}};
//    wd.target.emplace_back(
//            Target::make_loop_rectangle_target(
//                    {4, 3},
//                    {16, 7},
//                    0.5
//            )
//    );

    Swarm s = Swarm(data["swarm"]["num"], wd);

    if (data["swarm"]["initial_position"] == "random_in_poly") {
        s.random_initial_position(Polygon(get_point_vector_from_json(data["swarm"]["random_poly"])));
    } else if (data["swarm"]["initial_position"] == "specified") {
        auto v = get_point_vector_from_json(data["swarm"]["specified_pos"]);
        s.set_initial_position(v);
    }
    else if (data["swarm"]["initial_position"] == "random_all"){
        s.random_initial_position();
    }


    if (data["cbfs"]["energy_cbf"] == "on") {
        s.set_energy_cbf();
    }
    if (data["cbfs"]["camera_cbf"] == "on") {
        s.set_camera_cbf();
    }
    s.init_log_path(fileName);
    s.para_log_once();

    s.output();

    double t_total = data["execute"]["time_total"], t_gap = data["execute"]["step_time"];
    for (int iter = 1; iter <= t_total / t_gap; iter++) {
        if (data["execute"]["output_grid"] == "off") {
            printf("\r%.2lf seconds elapsed...", t_gap * (iter - 1));
        } else {
            printf("%.2lf seconds elapsed...\n", t_gap * (iter - 1));
        }
        if (data["cbfs"]["comm_cbf"] == "on"){
            s.set_comm_cbf();
        }
        s.update_vis();
        if (data["cbfs"]["cvt_cbf"] == "on") {
            s.cal_cvt();
            s.set_cvt_cbf();
        }
        if (data["cbfs"]["safety_cbf"] == "on") {
            s.set_safety_cbf();
        }
        s.log_once();
        s.cvt_forward(t_gap);
        if (data["execute"]["output_grid"] == "on") {
            s.grid_world_output();
        }
    }

    printf("\nAfter %.4lf seconds\n", t_total);
//    s.gw.output();
    s.output();
    s.end_log();
    printf("Data saved in %s\n", fileName);

    clock_t finish = clock();
    double duration = (double) (finish - start) / CLOCKS_PER_SEC;
    printf("%.4lf seconds passed!\n", duration);
    return 0;
}