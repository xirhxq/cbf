#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include "doctest/doctest.h"
#include "Utils.h"
#include "ComputingGeometry/ComputingGeometry"
#include <vector>
#include <random>
#include <cmath>

TEST_CASE("Polygon Constructors") {
    SUBCASE("Default Constructor") {
        Polygon poly;
        CHECK(poly.n == 0);
    }

    SUBCASE("Array Constructor") {
        Point points[] = {Point(0, 0), Point(1, 0), Point(0, 1)};
        Polygon poly(3, points);
        CHECK(poly.n == 3);
    }

    SUBCASE("Vector Constructor") {
        std::vector<Point> points = {Point(0, 0), Point(1, 0), Point(0, 1)};
        Polygon poly(points);
        CHECK(poly.n == 3);
    }

    SUBCASE("Square Constructor") {
        std::vector<Point> square = {Point(0, 0), Point(1, 0), Point(1, 1), Point(0, 1)};
        Polygon poly(square);
        CHECK(poly.n == 4);
        CHECK(poly.area() == doctest::Approx(1.0));
        CHECK(poly.circumference() == doctest::Approx(4.0));
    }
}

TEST_CASE("Polygon Basic Operations") {
    std::vector<Point> triangle = {Point(0, 0), Point(3, 0), Point(0, 4)};
    Polygon poly(triangle);

    SUBCASE("Area Calculation") {
        CHECK(poly.area() == doctest::Approx(6.0));
    }

    SUBCASE("Circumference Calculation") {
        CHECK(poly.circumference() == doctest::Approx(12.0));
    }

    SUBCASE("Diameter Calculation") {
        CHECK(poly.get_diameter() == doctest::Approx(5.0));
    }

    SUBCASE("Inner Point") {
        poly.get_inner_point();
        Point center = poly.in;
        CHECK(center.x > 0);
        CHECK(center.y > 0);
        CHECK(center.x < 3);
        CHECK(center.y < 4);
    }
}

TEST_CASE("Polygon Limit and Boundary Functions") {
    std::vector<Point> rectangle = {Point(0, 0), Point(4, 0), Point(4, 3), Point(0, 3)};
    Polygon poly(rectangle);

    SUBCASE("X Limits") {
        pd x_limits = poly.get_x_limit(1.0);
        CHECK(x_limits.first == doctest::Approx(0.0));
        CHECK(x_limits.second == doctest::Approx(4.0));

        x_limits = poly.get_x_limit(1.2);
        CHECK(x_limits.first < 0.0);
        CHECK(x_limits.second > 4.0);
    }

    SUBCASE("Y Limits") {
        pd y_limits = poly.get_y_limit(1.0);
        CHECK(y_limits.first == doctest::Approx(0.0));
        CHECK(y_limits.second == doctest::Approx(3.0));

        y_limits = poly.get_y_limit(1.1);
        CHECK(y_limits.first < 0.0);
        CHECK(y_limits.second > 3.0);
    }

    SUBCASE("Y Limits at Certain X") {
        pd y_at_x2 = poly.get_y_lim_at_certain_x(2.0);
        CHECK(y_at_x2.first == doctest::Approx(0.0));
        CHECK(y_at_x2.second == doctest::Approx(3.0));

        pd y_at_x1 = poly.get_y_lim_at_certain_x(1.0);
        CHECK(y_at_x1.first == doctest::Approx(0.0));
        CHECK(y_at_x1.second == doctest::Approx(3.0));
    }
}

TEST_CASE("Polygon Point Management") {
    std::vector<Point> square = {Point(0, 0), Point(1, 0), Point(1, 1), Point(0, 1)};
    Polygon poly(square);

    SUBCASE("Add Point") {
        Point new_point(0.5, 2.5);
        poly.add_one_point(new_point);
        CHECK(poly.n >= 4);
    }

    SUBCASE("Delete Point by Position") {
        Point to_remove(1, 1);
        poly.delete_one_point_at_position(to_remove);
        CHECK(poly.n == 3);
    }

    SUBCASE("Delete Point by Index") {
        // Use fresh polygon for each test
        Polygon fresh_poly(square);
        int index = 2;
        fresh_poly.delete_one_point_at_index(index);
        CHECK(fresh_poly.n == 3);
    }

    SUBCASE("Direct Add") {
        Polygon fresh_poly(square);
        fresh_poly.direct_add(Point(0.5, 0.5));
        CHECK(fresh_poly.n == 5);
    }

    SUBCASE("Direct Delete") {
        Polygon fresh_poly(square);
        fresh_poly.direct_delete_at_position(Point(1, 1));
        CHECK(fresh_poly.n == 3);
    }
}

TEST_CASE("Polygon Geometric Operations") {
    std::vector<Point> triangle = {Point(0, 0), Point(4, 0), Point(2, 3)};
    Polygon poly(triangle);

    SUBCASE("Random Point Generation") {
        Point random_pt = poly.get_random_point();
        bool is_inside = true;

        CHECK(random_pt.x >= 0);
        CHECK(random_pt.x <= 4);
        CHECK(random_pt.y >= 0);
        CHECK(random_pt.y <= 3);
    }

    SUBCASE("Half-plane Intersection") {
        Line vertical_line(Point(2, -1), Point(2, 4));
        poly.intersect_with_halfplane(vertical_line, 1);

        CHECK(poly.n <= 3);
        for (int i = 1; i <= poly.n; i++) {
            CHECK(poly.p[i].x <= 2.0 + 1e-6);
        }
    }

    SUBCASE("Polygon Intersection") {
        std::vector<Point> small_square = {Point(1, 0.5), Point(3, 0.5), Point(3, 2.5), Point(1, 2.5)};
        Polygon square_poly(small_square);

        poly.intersect_with_polygon(square_poly);

        CHECK(poly.n > 0);
        CHECK(poly.area() <= 6.0);
    }
}

TEST_CASE("Polygon Special Cases") {
    SUBCASE("Collinear Points") {
        std::vector<Point> line = {Point(0, 0), Point(1, 0), Point(2, 0), Point(3, 0)};
        Polygon poly(line);

        CHECK(poly.n <= 2);
    }

    SUBCASE("Duplicate Points") {
        std::vector<Point> duplicate_square = {
            Point(0, 0), Point(1, 0), Point(1, 1),
            Point(1, 1), Point(0, 1), Point(0, 0)
        };
        Polygon poly(duplicate_square);

        CHECK(poly.n <= 4);
    }

    SUBCASE("Regular Pentagon") {
        std::vector<Point> pentagon;
        for (int i = 0; i < 5; i++) {
            double angle = 2 * M_PI * i / 5;
            pentagon.emplace_back(cos(angle), sin(angle));
        }
        Polygon poly(pentagon);

        CHECK(poly.n == 5);
        CHECK(poly.area() > 0);
        CHECK(poly.circumference() > 0);
        CHECK(poly.get_diameter() > 0);
    }
}

TEST_CASE("Polygon Diameter Function") {
    SUBCASE("Unit Square") {
        std::vector<Point> square = {Point(0, 0), Point(1, 0), Point(1, 1), Point(0, 1)};
        Polygon poly(square);
        CHECK(poly.get_diameter() == doctest::Approx(std::sqrt(2)));
    }

    SUBCASE("Rectangle") {
        std::vector<Point> rect = {Point(0, 0), Point(4, 0), Point(4, 2), Point(0, 2)};
        Polygon poly(rect);
        CHECK(poly.get_diameter() == doctest::Approx(std::sqrt(20)));
    }

    SUBCASE("Right Triangle") {
        std::vector<Point> triangle = {Point(0, 0), Point(3, 0), Point(0, 4)};
        Polygon poly(triangle);
        CHECK(poly.get_diameter() == doctest::Approx(5.0));
    }

    SUBCASE("Equilateral Triangle") {
        double height = std::sqrt(3) / 2;
        std::vector<Point> triangle = {
            Point(0, 0), Point(1, 0), Point(0.5, height)
        };
        Polygon poly(triangle);
        CHECK(poly.get_diameter() == doctest::Approx(1.0));
    }

    SUBCASE("Regular Hexagon") {
        std::vector<Point> hexagon;
        for (int i = 0; i < 6; i++) {
            double angle = 2 * M_PI * i / 6;
            hexagon.emplace_back(cos(angle), sin(angle));
        }
        Polygon poly(hexagon);
        CHECK(poly.get_diameter() == doctest::Approx(2.0));
    }
}

TEST_CASE("Polygon Area with Function") {
    std::vector<Point> square = {Point(0, 0), Point(1, 0), Point(1, 1), Point(0, 1)};
    Polygon poly(square);

    SUBCASE("Constant Function") {
        auto constant_func = [](Point p) { return 2.0; };
        double result = poly.area_with_function(constant_func, 0.1);
        CHECK(result == doctest::Approx(2.0));
    }

    SUBCASE("Linear Function") {
        auto linear_func = [](Point p) { return p.x + p.y; };
        double result = poly.area_with_function(linear_func, 0.05);
        CHECK(result > 0);
    }

    SUBCASE("Quadratic Function") {
        auto quadratic_func = [](Point p) { return p.x * p.x + p.y * p.y; };
        double result = poly.area_with_function(quadratic_func, 0.1);
        CHECK(result > 0);
    }
}

TEST_CASE("Polygon Convexity Check") {
    SUBCASE("Convex Square") {
        std::vector<Point> square = {Point(0, 0), Point(1, 0), Point(1, 1), Point(0, 1)};
        Polygon poly(square);
        CHECK(poly.n == 4);
    }

    SUBCASE("Convex Triangle") {
        std::vector<Point> triangle = {Point(0, 0), Point(2, 0), Point(1, 3)};
        Polygon poly(triangle);
        CHECK(poly.n == 3);
    }
}

TEST_CASE("Polygon Stress Test") {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> angle_dis(0.0, 2 * M_PI);
    std::uniform_real_distribution<> radius_dis(2.0, 5.0);

    SUBCASE("Random Convex Polygon") {
        std::vector<Point> random_points;
        int num_points = 8;

        // Generate points on concentric circles to ensure convexity
        for (int i = 0; i < num_points; i++) {
            double angle = 2 * M_PI * i / num_points;
            double radius = 3.0 + (i % 3) * 0.5; // Use 3 different radii
            random_points.emplace_back(radius * cos(angle), radius * sin(angle));
        }

        Polygon poly(random_points);

        CHECK(poly.n > 0);
        CHECK(poly.n <= num_points);

        if (poly.n >= 3) {
            CHECK(poly.area() >= 0);
            CHECK(poly.circumference() > 0);
            CHECK(poly.get_diameter() > 0);
        }
    }

    SUBCASE("Large Regular Polygon") {
        std::vector<Point> large_polygon;
        int n = 20;
        double radius = 10.0;

        for (int i = 0; i < n; i++) {
            double angle = 2 * M_PI * i / n;
            large_polygon.emplace_back(radius * cos(angle), radius * sin(angle));
        }

        Polygon poly(large_polygon);

        CHECK(poly.n == n);
        CHECK(poly.get_diameter() == doctest::Approx(2 * radius));
        CHECK(poly.area() > 0);
        CHECK(poly.circumference() > 0);
    }
}

TEST_CASE("Polygon Contains Method") {
      SUBCASE("Basic Square - Internal Points") {
          std::vector<Point> square = {Point(0, 0), Point(2, 0), Point(2, 2), Point(0, 2)};
          Polygon poly(square);

          // Test points clearly inside the square (convex polygon)
          CHECK(poly.contains(Point(1, 1)) == true);    // Center
          CHECK(poly.contains(Point(0.5, 0.5)) == true);  // Bottom-left quadrant
          CHECK(poly.contains(Point(1.5, 1.5)) == true); // Top-right quadrant

          // Test points clearly outside the square
          CHECK(poly.contains(Point(-1, 1)) == false);   // Left
          CHECK(poly.contains(Point(3, 1)) == false);    // Right
          CHECK(poly.contains(Point(1, -1)) == false);   // Bottom
          CHECK(poly.contains(Point(1, 3)) == false);    // Top
      }

      SUBCASE("Triangle - Internal Points") {
          std::vector<Point> triangle = {Point(0, 0), Point(3, 0), Point(0, 4)};
          Polygon poly(triangle);

          // Test points well inside the right triangle (convex polygon)
          CHECK(poly.contains(Point(1, 1)) == true);    // Well inside
          CHECK(poly.contains(Point(0.5, 0.5)) == true); // Near corner
          CHECK(poly.contains(Point(1, 0.5)) == true);  // Bottom area

          // Test points clearly outside the triangle
          CHECK(poly.contains(Point(-1, 1)) == false);   // Left
          CHECK(poly.contains(Point(4, 1)) == false);    // Right
          CHECK(poly.contains(Point(1, 3)) == false);    // Near hypotenuse but outside
      }

      SUBCASE("Edge and Vertex Tests") {
          std::vector<Point> square = {Point(0, 0), Point(1, 0), Point(1, 1), Point(0, 1)};
          Polygon poly(square);

          // Test vertices (should be true for boundary points)
          CHECK(poly.contains(Point(0, 0)) == true);     // Bottom-left vertex
          CHECK(poly.contains(Point(1, 1)) == true);     // Top-right vertex

          // Test edge midpoints (should be true for boundary points)
          CHECK(poly.contains(Point(0.5, 0)) == true);   // Bottom edge midpoint
          CHECK(poly.contains(Point(0, 0.5)) == true);   // Left edge midpoint

          // Test points just outside
          CHECK(poly.contains(Point(-1e-10, 0.5)) == false); // Just left of left edge
          CHECK(poly.contains(Point(0.5, 1+1e-10)) == false); // Just above top edge
      }
  }