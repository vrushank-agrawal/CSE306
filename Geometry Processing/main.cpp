#define _CRT_SECURE_NO_WARNINGS 1
#include <vector>
#include <cmath>
#include <limits>
#include <random>
#include <cfloat>
#include <tuple>
#include <omp.h>
#include <list>
#include <chrono>
#include <iostream>

#include "svg.h"

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "../stb/stb_image_write.h"

static std::default_random_engine engine(10);
static std::uniform_real_distribution<double> uniform(0.0, 1.0);

/* ------------------------ POLYGON --------------------------------- */

Vector polygonIntersection(const Vector& prev, const Vector& cur, const std::vector<Vector>& edges) {
    Vector normal = (Vector(edges[1][1] - edges[0][1], edges[0][0] - edges[1][0], 0)).normalize();
    double distance = dot(normal, edges[0] - prev);
    double t = dot(normal, edges[0] - prev) / dot(normal, cur - prev);
    if (0 <= t && t <= 1)
        return prev + t * (cur - prev);
    return Vector();
}

bool isInside(const Vector& point, const std::vector<Vector>& edges) {
    Vector normal = (Vector(edges[1][1] - edges[0][1], edges[0][0] - edges[1][0], 0)).normalize();
    return dot(normal, edges[0] - point) <= 0;
}

/* Polygon clipping */
Polygon clipPolygon(Polygon& subjectPolygon, Polygon& convexPolygon) {
    Polygon outputPolygon;
    for (int i=0; i < convexPolygon.vertices.size()-1; i++) {

        std::vector<Vector> clipEdge = {convexPolygon.vertices[i],
                                        convexPolygon.vertices[(i > 0) ? (i - 1):(convexPolygon.vertices.size() - 1)]};
        // outputPolygon = Polygon();

        for (int j=0; j < subjectPolygon.vertices.size(); j++) {

            Vector currVertex = subjectPolygon.vertices[i];
            Vector prevVertex = subjectPolygon.vertices[(j>0)?j-1:subjectPolygon.vertices.size()-1];
            Vector intersection = polygonIntersection(prevVertex, currVertex, clipEdge);

            if (isInside(currVertex, clipEdge)) {
                if (!isInside(prevVertex, clipEdge))
                    outputPolygon.addVertex(intersection);
                outputPolygon.addVertex(currVertex);
            }
            else if (isInside(prevVertex, clipEdge)) {
                outputPolygon.addVertex(intersection);
            }
        }
        subjectPolygon = outputPolygon;
    }
    return outputPolygon;
}


/* ------------------------ VORONOI --------------------------------- */



/* ------------------------ FLUID DYNAMICS --------------------------------- */




/* ------------------------ MAIN --------------------------------- */

int main() {

    Polygon subjectPolygon({
        Vector(0.1, 0.2), Vector(0.3, 0.8), Vector(0.6, 0.5),
        Vector(0.8, 0.9), Vector(0.9, 0.1), Vector(0.5, 0.4)
    });

    Polygon convexPolygon({
        Vector(0.3, 0.3), Vector(0.3, 0.7),
        Vector(0.7, 0.7), Vector(0.7, 0.3)
    });

    save_svg({subjectPolygon, convexPolygon}, "images/initial.svg", "none");
    save_svg({clipPolygon(subjectPolygon, convexPolygon)}, "images/clipped.svg", "none");
}