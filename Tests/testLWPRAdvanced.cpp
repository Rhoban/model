#include <iostream>
#include <lwpr_eigen.hpp>
#include "Utils/LWPRUtils.h"
#include "Plot/Plot.hpp"
#include "Spline/CubicSpline.hpp"

std::default_random_engine generator;      
std::uniform_real_distribution<double> uniform(-1.0, 1.0);

double dRand()
{
    return uniform(generator);
}

double nonConvexFunction(double t, int mode)
{
    Leph::CubicSpline spline1;
    spline1.addPoint(0.0, 0.0);
    spline1.addPoint(0.1, 0.5);
    spline1.addPoint(0.3, -0.5);
    spline1.addPoint(0.5, 0.0);
    spline1.addPoint(0.6, -0.4);
    spline1.addPoint(0.8, 0.5);
    spline1.addPoint(1.0, 0.0);
    Leph::CubicSpline spline2;
    spline2.addPoint(0.0, 0.0);
    spline2.addPoint(0.1, 1.0);
    spline2.addPoint(0.3, -0.5);
    spline2.addPoint(0.5, 0.0);
    spline2.addPoint(0.6, 0.4);
    spline2.addPoint(0.8, -0.2);
    spline2.addPoint(1.0, 0.0);
    if (t < 0.5) {
        return spline1.pos(t);
    } else if (mode == 1) {
        return spline1.pos(t);
    } else if (mode == 2) {
        return spline2.pos(t);
    } else {
        return 0.0;
    }
}

/**
 * Try to learn non convex function
 */
void testNonConvex()
{
    size_t inputDim = 2;
    Eigen::VectorXd params = Leph::LWPRInitParameters(inputDim);
    std::vector<Eigen::VectorXd> trainInputs;
    std::vector<double> trainOutputs;

    Leph::Plot plot;
    for (size_t k=0;k<10;k++) {
        int mode = rand()%2 + 1;
        for (double t=0;t<=1.0;t+=0.01) {
            double y = nonConvexFunction(t, mode) + 0.01*dRand();
            double y1 = nonConvexFunction(t-0.02, mode) + 0.01*dRand();
            plot.add(Leph::VectorLabel(
                "t", t, 
                "target", y,
                "mode", mode
            ));
            Eigen::VectorXd x(inputDim);
            x << t, y1;
            trainInputs.push_back(x);
            trainOutputs.push_back(y);
        }
    }
    plot.plot("t", "target", Leph::Plot::Points, "mode").render();

    params = Leph::LWPROptimizeParameters(inputDim,
        params, trainInputs, trainOutputs, trainInputs, trainOutputs,
        500, false);

    LWPR_Object model = Leph::LWPRInit(inputDim, params);
    for (size_t i=0;i<trainInputs.size();i++) {
        Eigen::VectorXd y(1);
        y << trainOutputs[i];
        model.update(trainInputs[i], y);
    }

    plot.clear();
    for (int mode=1;mode<=2;mode++) {
        for (double t=0;t<=1.0;t+=0.01) {
            double y = nonConvexFunction(t, mode);
            Eigen::VectorXd confidence(1);
            Eigen::VectorXd x(inputDim);
            x << t, nonConvexFunction(t-0.02, mode);
            Eigen::VectorXd yp = model.predict(x, confidence);
            if (confidence(0) > 2.0) confidence(0) = 2.0;
            plot.add(Leph::VectorLabel(
                "t", t, 
                "diff", t-nonConvexFunction(t-0.02, mode),
                "real", y,
                "mode", mode,
                "fitted", yp(0),
                "confidence", confidence(0) 
            ));
        }
    }
    plot
        .plot("t", "real", Leph::Plot::Points)
        .plot("t", "fitted", Leph::Plot::Points)
        .plot("t", "confidence", Leph::Plot::Points)
        .render();
    plot.plot("t", "diff", "fitted", 
        Leph::Plot::Points).render();
}

int main()
{
    testNonConvex();
    return 0;
}

