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

Vector polygonIntersection(const Vector& A, const Vector& B, const std::vector<Vector>& edges) {
    Vector normal = (Vector(edges[1][1] - edges[0][1], edges[0][0] - edges[1][0], 0)).normalize();
    double t = dot(normal, edges[0] - A) / dot(normal, B - A);
    if (0 <= t && t <= 1)
        return A + t*(B - A);
    return Vector();
}

bool isInside(const Vector& point, const std::vector<Vector>& edges) {
    Vector normal = (Vector(edges[1][1] - edges[0][1], edges[0][0] - edges[1][0], 0));
    return dot(normal, point - edges[0]) <= 0;
}

/* Polygon clipping Sutherland-Hodgman */
Polygon clipPolygon(Polygon& subjectPolygon, Polygon& clipPolygon) {
    Polygon outputPolygon;
    for (int i=0; i < clipPolygon.vertices.size(); i++) {

        std::vector<Vector> clipEdge = {clipPolygon.vertices[i],
                                        clipPolygon.vertices[(i>0) ? (i-1):(clipPolygon.vertices.size() - 1)]};
        outputPolygon = Polygon();

        for (int j=0; j < subjectPolygon.vertices.size(); j++) {

            Vector currVertex = subjectPolygon.vertices[j];
            Vector prevVertex = subjectPolygon.vertices[(j>0) ? (j-1):(subjectPolygon.vertices.size()-1)];
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

Vector calcNormal(const Vector& A, const Vector& B, double wA, double wB) {
    return (A+B)/2 + (wA-wB)*(B-A)/(2*std::pow((A-B).norm2(), 2));
}

/* Power diagram intersection */
Vector voronoiIntersect(
    const Vector& A,
    const Vector& B,
    const std::vector<Vector>& edges,
    const std::vector<double>& weights
) {
    Vector normal = calcNormal(edges[0], edges[1], weights[0], weights[1]);
    double t = dot(normal - A, edges[0] - edges[1]) / dot(B - A, edges[0] - edges[1]);
    if (0 <= t && t <= 1)
        return A + t*(B - A);
    return Vector();
}

bool isInsideVoronoi(const Vector& point, const std::vector<Vector>& edges, const std::vector<double>& weights) {
    Vector normal = calcNormal(edges[0], edges[1], weights[0], weights[1]);
    return dot(point - normal, edges[1] - edges[0]) < 0;
}

/* Voronoi Parallel Linear Enumeration */
std::vector<Polygon> voronoiPLE(
    const std::vector<Vector>& points,
    const Polygon& bounds,
    const std::vector<double>& weights
) {
    std::vector<Polygon> outputPolygons(points.size());

    #pragma omp parallel for
    for (int i=0; i < points.size(); i++) {
        Vector P_i = points[i];
        Polygon currEdges = bounds;

        for (int j=0; j < points.size(); j++)
        {
            if (i != j)
            {
                Vector P_j = points[j];
                std::vector<Vector> edge = {P_i, P_j};
                std::vector<double> weight = {weights[i], weights[j]};
                Polygon outputPolygon;
                for (int k=0; k < currEdges.vertices.size(); k++)
                {
                    Vector currVertex = currEdges.vertices[k];
                    Vector prevVertex = currEdges.vertices[(k>0) ? (k-1):(currEdges.vertices.size()-1)];
                    Vector intersection = voronoiIntersect(prevVertex, currVertex, edge, weight);

                    if (isInsideVoronoi(currVertex, edge, weight)) {
                        if (!isInsideVoronoi(prevVertex, edge, weight))
                            outputPolygon.addVertex(intersection);
                        outputPolygon.addVertex(currVertex);
                    }
                    else if (isInsideVoronoi(prevVertex, edge, weight)) {
                        outputPolygon.addVertex(intersection);
                    }
                }
                currEdges = outputPolygon;
            }
        }
        outputPolygons[i] = currEdges;
    }
    return outputPolygons;
}


/* ------------------------ FLUID DYNAMICS --------------------------------- */




/* ------------------------ MAIN --------------------------------- */

int main() {

    #ifdef POLYGON_TEST
    Polygon subjectPolygon({
        Vector(0.1, 0.2), Vector(0.3, 0.8), Vector(0.6, 0.5),
        Vector(0.8, 0.9), Vector(0.9, 0.1), Vector(0.5, 0.4)
    });

    Polygon convexClipPolygon({
        Vector(0.3, 0.3), Vector(0.3, 0.7),
        Vector(0.7, 0.7), Vector(0.7, 0.3)
    });

    save_svg({subjectPolygon, convexClipPolygon}, "images/initial.svg", "none");
    save_svg({clipPolygon(subjectPolygon, convexClipPolygon)}, "images/clipped.svg", "none");
    #endif

    
    int n = 10;
    Polygon bounds({
        Vector(0., 0.), Vector(0., 1.),
        Vector(1., 1.), Vector(1., 0.)
    });

    std::vector<Vector> points(n);
    for (int i=0; i < n; i++) {
        points[i] = Vector(uniform(engine), uniform(engine));
    }
    save_svg(voronoiPLE(points, bounds, std::vector<double>(n, 1)), "images/voronoi.svg", "none");
}