#define _CRT_SECURE_NO_WARNINGS 1
#include <vector>
#include <random>
#include <omp.h>
#include <iostream>

#include "svg.h"

#define POLYGON_TEST

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
    return (A+B)/2 + (wA-wB)*(B-A)/(2*((A-B).norm2()*(A-B).norm2()));
}

/* Power diagram intersection */
Vector voronoiIntersect(const Vector& A,
                        const Vector& B,
                        const std::vector<Vector>& edges,
                        const std::vector<double>& weights)
{
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
std::vector<Polygon> voronoiPLE(const std::vector<Vector>& points,
                                const Polygon& bounds,
                                const std::vector<double>& weights)
{
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


/* ------------------------ L-BFGS --------------------------------- */




/* ------------------------ FLUID DYNAMICS --------------------------------- */

/* Gallouet Merigot Scheme */
void gallouet_merigot(std::vector<Vector>& points,
                    std::vector<Vector>& velocities,
                    const std::vector<double>& masses,
                    const Polygon& bounds,
                    int niter,
                    int N, int M)
{

    for (int i=0; i<niter; i++) {

    }

}


/* ------------------------ Tutte's Mapping - Lab 9 ------------------------ */

std::vector<Vector> tutte(std::vector<Vector>& vertices,
                            const std::vector<std::vector<int>>& adjacencies,
                            const std::vector<int>& fixedIndices,
                            int niter)
{
    std::vector<Vector> outputVertices(vertices.size());
    double s = 0;
    for (int i=0; i < vertices.size(); i++)
        s += (vertices[fixedIndices[i]] - vertices[fixedIndices[(i+1)%fixedIndices.size()]]).norm();

    double cs = 0;
    for (int i=0; i < vertices.size(); i++) {
        double theta = 2*M_PI*cs/s;
        vertices[fixedIndices[i]] = Vector(cos(theta), sin(theta), 0);
        cs += (vertices[fixedIndices[i]] - vertices[fixedIndices[(i+1)%fixedIndices.size()]]).norm();
    }

    for (int i=0; i < niter; i++) {
        for (int j=0; j < vertices.size(); j++) {
            if (std::find(fixedIndices.begin(), fixedIndices.end(), j) == fixedIndices.end()) {
                Vector sum(0, 0, 0);
                for (int k=0; k < adjacencies[j].size(); k++)
                    sum = sum + vertices[adjacencies[j][k]];
                outputVertices[j] = sum / adjacencies[j].size();
            }
        }
        vertices = outputVertices;
    }
    return vertices;
}


/* ------------------------ MAIN --------------------------------- */

int main() {

    #ifdef POLYGON_TEST
    Polygon subjectPolygon({
        Vector(0.1, 0.1), Vector(0.2, 0.3), Vector(0.3, 0.4),
        Vector(0.4, 0.4), Vector(0.5, 0.9), Vector(0.6, 0.4),
        Vector(0.7, 0.4), Vector(0.2, 0.1), Vector(0.1, 0.1),
    });

    Polygon convexClipPolygon({
        Vector(0.3, 0.3), Vector(0.3, 0.7),
        Vector(0.4, 0.8), Vector(0.5, 0.9),
        Vector(0.6, 0.6), Vector(0.7, 0.2),
        Vector(0.3, 0.3)
    });

    save_svg({subjectPolygon, convexClipPolygon}, "images/initial.svg", "none");
    save_svg({clipPolygon(subjectPolygon, convexClipPolygon)}, "images/clipped.svg", "none");

    #else

    int n = 1000;
    Polygon bounds({
        Vector(0., 0.), Vector(0., 1.),
        Vector(1., 1.), Vector(1., 0.)
    });
    std::vector<Vector> points(n);
    for (int i=0; i < n; i++)
        points[i] = Vector(uniform(engine), uniform(engine));
    std::vector<double> weights(n, 1);
    save_svg(voronoiPLE(points, bounds, weights), "images/voronoi_1000.svg", "none");

    for (int i=0; i < n; i++) {
        if (points[i][0] < 0.2 || points[i][0] > 0.8 || points[i][1] < 0.2 || points[i][1] > 0.8) {
            weights[i] = 0;
        } else {
            weights[i] = 1;
        }
    }
    save_svg(voronoiPLE(points, bounds, weights), "images/power_1000.svg", "none");
    #endif

}