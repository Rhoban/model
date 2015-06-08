#include <iostream>
#include <cmath>
#include <vector>
#include <libcmaes/cmaes.h>
#include <lwpr.hh>
#include "Plot/Plot.hpp"

std::default_random_engine generator;      
std::uniform_real_distribution<double> uniform(-1.0, 1.0);

double dRand()
{
    return uniform(generator);
}

double cross(double x1,double x2) 
{
    double a = exp(-10*x1*x1);
    double b = exp(-50*x2*x2);
    double c = 1.25*exp(-5*(x1*x1 + x2*x2));

    if (a>b) {
        return (a>c) ? a:c;
    } else {
        return (b>c) ? b:c;
    }
}

double testLWPR(
    double paramNormIn1, double paramNormIn2,
    double paramInitAlpha,
    double paramMetaRate,
    double paramPenalty,
    double paramInitD1, double paramInitD2,
    double paramWGen,
    double paramWPrune,
    bool verbose)
{
    //InitializeLWPR model with 
    //input and output dimensions
    LWPR_Object model(2, 1);

    //Normalisation value for each input dimension (>0)
    model.normIn({paramNormIn1, paramNormIn2});
    //Use only diagonal matrix. Big speed up in high dimension
    //but lower performance in complex learning.
    model.diagOnly(true);
    //Learning rate for gradient descente (>0)
    //(meta optimized)
    model.setInitAlpha(paramInitAlpha);
    //Automatic tunning of distance metric 
    model.useMeta(true);
    //Meta tunning learning rate (>0)
    model.metaRate(paramMetaRate);
    //Larger value enforce wider receptive field (>0)
    model.penalty(paramPenalty);
    //Set diagonal (input_dim) or complet (input_dim*input_dim)
    //initial distance matrix (>0)
    model.setInitD({paramInitD1, paramInitD2});
    //Receptive field activation threshold (>0)
    model.wGen(paramWGen);
    //Receptive field remove threshold
    model.wPrune(paramWPrune);
    
    std::vector<double> x(2);
    std::vector<double> y(1);
    Leph::Plot plot;
    size_t count = 500;
    for (size_t i=0;i<count;i++) {
        //Generate a data point
        x[0] = dRand();
        x[1] = dRand();
        y[0] = cross(x[0],x[1]) + 0.2*dRand();
        // Update the model with one sample
        model.update(x,y);
        //Plot
        if (verbose) {
            plot.add(Leph::VectorLabel(
                "x0", x[0],
                "x1", x[1],
                "target", y[0]
            ));
        }
    }
    
    double mse = 0.0; 
    size_t numTest = 0;  
    for (x[1]=-1.0;x[1]<=1.0;x[1]+=0.05) {   
        for (x[0]=-1.0;x[0]<=1.0;x[0]+=0.05) {
            y[0] = cross(x[0], x[1]);
            // Use the model for predicting an output
            std::vector<double> yp = model.predict(x);
            //Compute prediction error
            mse += (y[0]-yp[0])*(y[0]-yp[0]);
            numTest++;
            //Plot
            if (verbose) {
                plot.add(Leph::VectorLabel(
                    "x0", x[0],
                    "x1", x[1],
                    "fitted", yp[0],
                    "real", cross(x[0], x[1])
                ));
            }
        }
    }
    if (verbose) {
        plot
            .plot("x0", "x1", "target")
            .plot("x0", "x1", "fitted")
            //.plot("x0", "x1", "real")
            .render();
    }

    mse /= (double)numTest;
    std::cout << "MSE=" << mse << std::endl;
    if (verbose) {
        std::cout << "normIn: " << paramNormIn1 << " " << paramNormIn2 << std::endl;
        std::cout << "initAlpha: " << paramInitAlpha << std::endl;
        std::cout << "metaRate: " << paramMetaRate << std::endl;
        std::cout << "penalty: " << paramPenalty << std::endl;
        std::cout << "initD: " << paramInitD1 << " " << paramInitD2 << std::endl;
        std::cout << "wGen: " << paramWGen << std::endl;
        std::cout << "wPrune: " << paramWPrune << std::endl;
    }
    return mse;
}

int main()
{
    //Starting point
    std::vector<double> x0(9, 0.0);
    x0[0] = 1.0; //normIn1
    x0[1] = 1.0; //normIn2
    x0[2] = 50.0; //initAlpha
    x0[3] = 250.0; //metaRate
    x0[4] = 1e-6; //penalty
    x0[5] = 100.0; //initD1
    x0[6] = 100.0; //initD2
    x0[7] = 0.2; //wGen
    x0[8] = 0.9; //WPrune
    //Optimization init
    libcmaes::CMAParameters<> cmaparams(x0, 0.1);
    cmaparams.set_quiet(false);
    cmaparams.set_mt_feval(false);
    cmaparams.set_str_algo("acmaes");
    cmaparams.set_max_iter(100);

    //Fitness function
    libcmaes::FitFunc fitness = []
        (const double *x, const int N) 
    {
        return testLWPR(
            x[0], x[1], x[2], 
            x[3], x[4], x[5], 
            x[6], x[7], x[8], 
            false);
    };
    
    //Run optimization
    libcmaes::CMASolutions cmasols = libcmaes::cmaes<>(fitness, cmaparams);
    Eigen::VectorXd params = cmasols.best_candidate().get_x_dvec();

    //Run with best parameters
    testLWPR(
        params(0), params(1), params(2), 
        params(3), params(4), params(5), 
        params(6), params(7), params(8), 
        true);

    return 0;
}

