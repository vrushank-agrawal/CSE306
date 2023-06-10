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
    std::vector<double> weights;
    Polygon edges;
    std::vector<Polygon> polygons;

    OT( const std::vector<Vector>& points,
        const std::vector<double>& weights,
        const Polygon& edges
    ) : points(points),
        weights(weights),
        edges(edges) {}

    ~OT() {}

    void solve() {
        double fx = 0.0;
        lbfgs(
            points.size(),
            &(weights[0]),
            &fx,
            _evaluate,
            _progress,
            this,
            NULL
        );
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

        std::vector<double> weights(x, x + n);
        polygons = voronoiPLE(points, edges, weights);

        for (int i=0; i<n; i++) {
            g[i] = polygons[i].area() - weights[i];
            fx += g[i] * g[i];
            fx -= polygons[i].integrateSqDist(points[i]);
        }

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
        /* std::cout << "Iteration " << k << ":\n";
         * std::cout << "  fx = " << fx << "\n";
         * std::cout << "  xnorm = " << xnorm << "\n";
         * std::cout << "  gnorm = " << gnorm << "\n";
         * std::cout << "  step = " << step << "\n";
         * std::cout << "  ls = " << ls << "\n";
         * std::cout << "\n";
         */
        return 0;
    }

};