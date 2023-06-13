#pragma once
#include <vector>
#include "vector.h"
#include "svg.h"
#include "lbfgs/lbfgs.h"

std::vector<Polygon> voronoiPLE(
    const std::vector<Vector>& points,
    const Polygon& edges,
    const std::vector<double>& weights);

class OT {

public:

    std::vector<Vector> points;
    std::vector<double> lambdas;
    Polygon edges;
    std::vector<Polygon> polygons;
    std::vector<double> weights = std::vector<double>(points.size(), 1.0);

    OT( const std::vector<Vector>& points,
        const std::vector<double>& lambdas,
        const Polygon& edges
    ) : points(points),
        lambdas(lambdas),
        edges(edges) {}

    ~OT() {}

    void solve(int n) {
        std::cout<< "entering solve" << std::endl;
        double fx = 0.0;
        lbfgs(n, &weights[0], &fx, _evaluate, _progress, this, NULL);
        polygons = voronoiPLE(points, edges, weights);
    }

    static lbfgsfloatval_t _evaluate(
        void *instance,
        const lbfgsfloatval_t *x,
        lbfgsfloatval_t *g,
        const int n,
        const lbfgsfloatval_t step
    ) {
        return reinterpret_cast<OT*>(instance)->evaluate(x, g, n, step);
    }

    lbfgsfloatval_t evaluate(
        const lbfgsfloatval_t *x,
        lbfgsfloatval_t *g,
        const int n,
        const lbfgsfloatval_t step
    ) {
        lbfgsfloatval_t fx = 0.0;

        for (int i=0; i<n; i++)
            weights[i] = x[i];

        polygons = voronoiPLE(points, edges, weights);
        // std::cout<< "entering evaluate" << std::endl;
        double t1=0, t2=0, t3=0;
        for (int i=0; i<n; i++) {
            g[i] = polygons[i].area() - lambdas[i];
            t1 += polygons[i].integrateSqDist(points[i]);
            t2 -= x[i] * polygons[i].area();
            t3 += x[i] * lambdas[i];
        }
        fx = -(t1 + t2 + t3);
        // std::cout << "fx: " << fx << std::endl;
        return fx;
    }

    static int _progress(
        void *instance,
        const lbfgsfloatval_t *x,
        const lbfgsfloatval_t *g,
        const lbfgsfloatval_t fx,
        const lbfgsfloatval_t xnorm,
        const lbfgsfloatval_t gnorm,
        const lbfgsfloatval_t step,
        int n,
        int k,
        int ls
    ) {
        return reinterpret_cast<OT*>(instance)->progress(x, g, fx, xnorm, gnorm, step, n, k, ls);
    }

    int progress(
        const lbfgsfloatval_t *x,
        const lbfgsfloatval_t *g,
        const lbfgsfloatval_t fx,
        const lbfgsfloatval_t xnorm,
        const lbfgsfloatval_t gnorm,
        const lbfgsfloatval_t step,
        int n,
        int k,
        int ls
    ) {
		return 0;
    }

};