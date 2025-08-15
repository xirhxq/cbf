#ifndef CBF_GRIDWORLD_HPP
#define CBF_GRIDWORLD_HPP

#include "utils.h"

class GridWorld {
public:
    std::pair<double, double> xLim, yLim;
    int xNum{}, yNum{};
    std::vector<bool> vis;
    double trueWeight = 0.0, falseWeight = 1.0;

public:
    GridWorld() {

    }
    GridWorld(pd xLim, int xNum, pd yLim, int yNum): xLim(xLim), yLim(yLim), xNum(xNum), yNum(yNum) {
        vis.resize(xNum * yNum);
        reset();
    }

    explicit GridWorld(const json &worldSettings) {
        World tmpWorld(worldSettings);
        xLim = tmpWorld.boundary.get_x_limit(1.0), yLim = tmpWorld.boundary.get_y_limit(1.0);
        xNum = (xLim.second - xLim.first) / double(worldSettings["spacing"]);
        yNum = (yLim.second - yLim.first) / double(worldSettings["spacing"]);
        vis.resize(xNum * yNum);
        reset();
    }

    void reset(bool value = false) {
        for (auto a: vis) {
            a = value;
        }
    }

    void output(char c = '*') {
        printf("< Gridworld of (%.2lf-%.2lf, %.2lf-%.2lf) -> (%d, %d) grids",
               xLim.first, xLim.second, yLim.first, yLim.second, xNum, yNum);
        printf("\n");
        for (int j = yNum - 1; j >= 0; j--) {
            for (int i = 0; i < xNum; i++) {
                printf("%c", (getValue(i, j) == true) ? c : '.');
            }
            printf("\n");
        }
        for (int i = 0; i < xNum; i++) {
            printf("-");
        }
        printf(">\n");
    }

    double getPercentage() {
        // count the number of true values
        int num = 0;
        for (auto a: vis) {
            num += a;
        }
        return 1.0 * num / vis.size();
    }

    int getNumInLim(double coordinate, pd lim, int size, std::string mode = "round") {
        double ratio = (coordinate - lim.first) / (lim.second - lim.first);
        int ret = lround(ratio * size);
        if (mode == "ceil") {
            ret = ceil(ratio * size);
        } else if (mode == "floor") {
            ret = floor(ratio * size);
        }
        ret = std::max(std::min(ret, size - 1), 0);
//    printf("ratio = %.2lf ret = %d\n", ratio, ret);
        return ret;
    }

    int getNumInXLim(double x, std::string mode = "round") {
        return getNumInLim(x, xLim, xNum, mode);
    }

    int getNumInYLim(double y, std::string mode = "round") {
        return getNumInLim(y, yLim, yNum, mode);
    }

    int getIndex(int xIndex, int yIndex) const {
        int ind = xIndex * yNum + yIndex;
        return ind;
    }

    int getXIndex(int index) const {
        int xIndex = index / yNum;
        return xIndex;
    }

    int getYIndex(int index) const {
        int yIndex = index % yNum;
        return yIndex;
    }

    bool getValue(Point point) {
        int xIndex = getNumInXLim(point.x);
        int yIndex = getNumInYLim(point.y);
        int index = getIndex(xIndex, yIndex);
//    printf("(%d, %d) == (%d) -> %d\n", x_ind, y_ind, ind, bool(vis[ind]));
        return vis[index] == true;
    }

    bool getValue(int index) {
        return vis[index] == true;
    }

    bool getValue(int xIndex, int yIndex) {
        xIndex = std::max(0, std::min(xIndex, xNum - 1));
        yIndex = std::max(0, std::min(yIndex, yNum - 1));
        int ind = getIndex(xIndex, yIndex);
        return vis[ind] == true;
    }

    void setValue(Point point, bool value) {
        int xIndex = getNumInXLim(point.x);
        int yIndex = getNumInYLim(point.y);
        vis[getIndex(xIndex, yIndex)] = value;
    }

    void setValue(int xIndex, int yIndex, bool value) {
        vis[getIndex(xIndex, yIndex)] = value;
    }

    void setValue(int index, bool value) {
        vis[index] = value;
    }

    double getPositionInLimit(int index, pd limit, int size) {
        double ratio = 1.0 * index / size;
        double pos = limit.first * (1.0 - ratio) + limit.second * ratio;
        return pos;
    }

    double getXPositionInXLimit(int xIndex) {
        return getPositionInLimit(xIndex, xLim, xNum);
    }

    double getYPositionInYLimit(int yIndex) {
        return getPositionInLimit(yIndex, yLim, yNum);
    }

    Point getPointInArea(int xIndex, int yIndex) {
        return {getXPositionInXLimit(xIndex), getYPositionInYLimit(yIndex)};
    }

    Point getPointInArea(int index) {
        return {getXPositionInXLimit(getXIndex(index)), getYPositionInYLimit(getYIndex(index))};
    }

    double getValueInPolygon(Polygon poly) {
        double res = 0;
        pd xLimit = poly.get_x_limit(1.0), yLimit;
        pd xIndexes, yIndexes;
        xIndexes.first = getNumInXLim(xLimit.first, "ceil");
        xIndexes.second = getNumInXLim(xLimit.second, "floor");
        for (int xIndex = xIndexes.first; xIndex <= xIndexes.second; xIndex++) {
            double x = getXPositionInXLimit(xIndex);
            yLimit = poly.get_y_lim_at_certain_x(x);
            yIndexes.first = getNumInYLim(yLimit.first, "ceil");
            yIndexes.second = getNumInYLim(yLimit.second, "floor");
            for (int y_ind = yIndexes.first; y_ind <= yIndexes.second; y_ind++) {
                res += (getValue(xIndex, y_ind) == true) ? trueWeight : falseWeight;
            }
        }
        return res;
    }

    json setValueInPolygon(Polygon poly, bool value, bool updateJson = false) {
        json ret = json::array();
        pd xLimit = poly.get_x_limit(1.0), yLimit;
        pd xIndexes, yIndexes;
        xIndexes.first = getNumInXLim(xLimit.first, "ceil");
        xIndexes.second = getNumInXLim(xLimit.second, "floor");
//    printf("x_lim: (%.12lf, %.12lf)\tx_ind: (%lf, %lf)\n", x_lim_pd.first, x_lim_pd.second, x_ind_pd.first, x_ind_pd.second);
        for (int xIndex = xIndexes.first; xIndex <= xIndexes.second; xIndex++) {
            double x = getXPositionInXLimit(xIndex);
            if (x > xLimit.second || x < xLimit.first) continue;
//        printf("x_ind = %d, x_pos = %.12lf\n", x_ind, x_pos);
            yLimit = poly.get_y_lim_at_certain_x(x);
            yIndexes.first = getNumInYLim(yLimit.first, "ceil");
            yIndexes.second = getNumInYLim(yLimit.second, "floor");
            for (int yIndex = yIndexes.first; yIndex <= yIndexes.second; yIndex++) {
                if (updateJson && getValue(xIndex, yIndex) != value) {
                    ret.push_back({xIndex, yIndex});
                }
                setValue(xIndex, yIndex, value);
            }
        }
        return ret;
    }

    json setValueInCircle(Point center, json params, bool value, bool updateJson = false) {
        double radius = params["radius"];
        json ret = json::array();
        pd xLimit = {center.x - radius, center.x + radius};
        pd yLimit = {center.y - radius, center.y + radius};
        pd xIndexes, yIndexes;
        xIndexes.first = getNumInXLim(xLimit.first, "ceil");
        xIndexes.second = getNumInXLim(xLimit.second, "floor");
        for (int i = xIndexes.first; i <= xIndexes.second; i++) {
            double x = getXPositionInXLimit(i);
            if (x > xLimit.second || x < xLimit.first) continue;
            yIndexes.first = getNumInYLim(yLimit.first, "ceil");
            yIndexes.second = getNumInYLim(yLimit.second, "floor");
            for (int j = yIndexes.first; j <= yIndexes.second; j++) {
                Point p = getPointInArea(i, j);
                if (p.distance_to(center) > radius) continue;
                if (updateJson && getValue(i, j) != value) {
                    ret.push_back({i, j});
                }
                setValue(i, j, value);
            }
        }
        return ret;
    }

    json setValueInTiltedCone(Point center, json params, bool value, bool updateJson = false) {
        double h = params["height"];
        double r = params["downward-radius"];
        double yawRad = params["yaw-rad"];
        double pitchRad = static_cast<double>(params["camera-pitch-deg"]) * M_PI / 180.0;

        json ret = json::array();

        // Apex of the cone A: (center.x, center.y, height)
        // Projection of cone central line B:
        // (center.x + height / tan(pitchRad) * cos(yawRad), center.y + height / tan(pitchRad) * sin(yawRad), 0)
        Eigen::Vector3d A(center.x, center.y, h);
        Eigen::Vector3d B(center.x + h / tan(pitchRad) * cos(yawRad),
                          center.y + h / tan(pitchRad) * sin(yawRad),
                          0.0);

        double alpha = atan(r / h);
        double theta = pitchRad;

        double l = h / tan(theta - alpha) - h / tan(theta + alpha);

        pd xLimit = {B.x() - l, B.x() + l};
        pd yLimit = {B.y() - l, B.y() + l};

        pd xIndexes, yIndexes;
        xIndexes.first = getNumInXLim(xLimit.first, "ceil");
        xIndexes.second = getNumInXLim(xLimit.second, "floor");

        double angleThreshold = alpha;

        for (int i = xIndexes.first; i <= xIndexes.second; i++) {
            double x = getXPositionInXLimit(i);
            if (x > xLimit.second || x < xLimit.first) continue;

            yIndexes.first = getNumInYLim(yLimit.first, "ceil");
            yIndexes.second = getNumInYLim(yLimit.second, "floor");

            for (int j = yIndexes.first; j <= yIndexes.second; j++) {
                Point p = getPointInArea(i, j);
                Eigen::Vector3d P(p.x, p.y, 0.0);

                Eigen::Vector3d PA = A - P;
                Eigen::Vector3d BA = A - B;

                double dotProduct = PA.dot(BA);
                double magnitudePA = PA.norm();
                double magnitudeBA = BA.norm();

                if (magnitudePA == 0 || magnitudeBA == 0) continue;

                double angle = acos(dotProduct / (magnitudePA * magnitudeBA));

                if (angle <= angleThreshold) {
                    if (updateJson && getValue(i, j) != value) {
                        ret.push_back({i, j});
                    }
                    setValue(i, j, value);
                }
            }
        }

        return ret;
    }

    json setValueInSectorRing(Point center, json params, bool value, bool updateJson = false) {
        double outerRadius = params["outer-radius"];
        double innerRadius = params["inner-radius"];
        double halfAngleRad = static_cast<double>(params["half-angle-deg"]) * M_PI / 180.0;
        double centerAngleRad = params["centerAngleRad"];
        pd xLimit = {center.x - outerRadius, center.x + outerRadius};
        pd yLimit = {center.y - outerRadius, center.y + outerRadius};
        pd xIndexes, yIndexes;
        xIndexes.first = getNumInXLim(xLimit.first, "ceil");
        xIndexes.second = getNumInXLim(xLimit.second, "floor");

        json ret = json::array();

        double startAngle = centerAngleRad - halfAngleRad;
        double endAngle = centerAngleRad + halfAngleRad;

        for (int i = xIndexes.first; i <= xIndexes.second; i++) {
            double x = getXPositionInXLimit(i);
            if (x > xLimit.second || x < xLimit.first) continue;

            yLimit.first = center.y - outerRadius;
            yLimit.second = center.y + outerRadius;
            yIndexes.first = getNumInYLim(yLimit.first, "ceil");
            yIndexes.second = getNumInYLim(yLimit.second, "floor");

            for (int j = yIndexes.first; j <= yIndexes.second; j++) {
                Point p = getPointInArea(i, j);

                double distance = p.distance_to(center);
                if (distance < innerRadius || distance > outerRadius) continue;

                double dx = p.x - center.x;
                double dy = p.y - center.y;
                double theta = atan2(dy, dx);
                if (theta < 0) theta += 2 * M_PI;

                double start = fmod(startAngle, 2 * M_PI);
                double end = fmod(endAngle, 2 * M_PI);
                bool inSector = false;

                if (start < end) {
                    inSector = (theta >= start && theta <= end);
                } else {
                    inSector = (theta >= start || theta <= end);
                }

                if (!inSector) continue;

                if (updateJson && getValue(i, j) != value) {
                    ret.push_back({i, j});
                }
                setValue(i, j, value);
            }
        }
        return ret;
}


    Point getCentroidInPolygon(Polygon poly) {
        int cntTotal = 0, cntTrue = 0;
        double sumX = 0.0, sumY = 0.0, totalWeight = getValueInPolygon(poly);
        double sumWeightedX = 0.0, sumWeightedY = 0.0;
        pd xLimit = poly.get_x_limit(1.0), yLimit;
        pd xIndexes, yIndexes;
        xIndexes.first = getNumInXLim(xLimit.first, "ceil");
        xIndexes.second = getNumInXLim(xLimit.second, "floor");
        for (int i = xIndexes.first; i <= xIndexes.second; i++) {
            double x = getXPositionInXLimit(i);
            yLimit = poly.get_y_lim_at_certain_x(x);
            yIndexes.first = getNumInYLim(yLimit.first, "ceil");
            yIndexes.second = getNumInYLim(yLimit.second, "floor");
            for (int j = yIndexes.first; j <= yIndexes.second; j++) {
                double y = getYPositionInYLimit(j);
                cntTotal++;
                sumX += x;
                sumY += y;
                if (getValue(i, j)) {
                    cntTrue++;
                    sumWeightedX += x * trueWeight;
                    sumWeightedY += y * trueWeight;
                } else {
                    sumWeightedX += x * falseWeight;
                    sumWeightedY += y * falseWeight;
                }
            }
        }
        if (totalWeight > 0) {
            return Point(sumWeightedX / totalWeight, sumWeightedY / totalWeight);
        } else {
            return Point(sumX / cntTotal, sumY / cntTotal);
        }
    }

    void outputCentroidInPolygon(Polygon poly) {
        auto tempVis = vis;
        for (auto a: tempVis) {
            a = false;
        }
        pd xLimit = poly.get_x_limit(1.0), yLimit;
        pd iLimit, jLimit;
        iLimit.first = getNumInXLim(xLimit.first, "ceil");
        iLimit.second = getNumInXLim(xLimit.second, "floor");
        for (int i = iLimit.first; i <= iLimit.second; i++) {
            double x = getXPositionInXLimit(i);
            yLimit = poly.get_y_lim_at_certain_x(x);
            jLimit.first = getNumInYLim(yLimit.first, "ceil");
            jLimit.second = getNumInYLim(yLimit.second, "floor");
            for (int j = jLimit.first; j <= jLimit.second; j++) {
                tempVis[getIndex(i, j)] = true;
            }
        }
    }

};


#endif //CBF_GRIDWORLD_HPP
