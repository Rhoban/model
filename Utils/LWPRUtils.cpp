#include <iostream>
#include <libcmaes/cmaes.h>
#include <stdexcept>
#include "Utils/LWPRUtils.h"

namespace Leph {

Eigen::VectorXd LWPRInitParameters(size_t inputDim)
{
    size_t paramDim = 2*inputDim + 5;
    Eigen::VectorXd params(paramDim);

    //NormIn
    for (size_t i=0;i<inputDim;i++) {
        params(i) = 0.05;
    }
    //InitAlpha
    params(inputDim) = 50.0;
    //MetaRate
    params(inputDim + 1) = 250.0;
    //Penalty
    params(inputDim + 2) = 1e-6;
    //InitD
    for (size_t i=0;i<inputDim;i++) {
        params(inputDim + 3 + i) = 1.0;
    }
    //WGen
    params(2*inputDim + 3) = 0.1;
    //WPrune
    params(2*inputDim + 4) = 0.9;

    return params;
}

LWPR_Object LWPRInit(size_t inputDim, 
    const Eigen::VectorXd& rawParams)
{
    if ((size_t)rawParams.size() != 2*inputDim+5) {
        throw std::logic_error("LWPRUtils invalid params size");
    }

    //Trim all values to positive
    Eigen::VectorXd params = rawParams;
    for (size_t i=0;i<(size_t)params.size();i++) {
        if (params(i) <= 0.0) {
            params(i) = 0.01;
        }
    }
    //Trim wGen
    if (params(params.size()-2) >= 1.0) {
        params(params.size()-2) = 0.999;
    }

    //InitializeLWPR model with 
    //input dimension
    LWPR_Object model(inputDim);
    
    //Normalisation value for each input dimension (>0)
    model.normIn(params.segment(0, inputDim));
    //Use only diagonal matrix. Big speed up in high dimension
    //but lower performance in complex learning.
    model.diagOnly(true);
    //Learning rate for gradient descente (>0)
    //(meta optimized)
model.setInitAlpha(params(inputDim));
    //Automatic tunning of distance metric 
    model.useMeta(true);
    //Meta tunning learning rate (>0)
model.metaRate(params(inputDim + 1));
    //Larger value enforce wider receptive field (>0)
model.penalty(params(inputDim + 2));
    //Set diagonal (input_dim) or complet (input_dim*input_dim)
    //initial distance matrix (>0)
    model.setInitD(params.segment(inputDim + 3, inputDim));
    //Receptive field activation threshold (>0)
//model.wGen(params(2*inputDim + 3));
    //Receptive field remove threshold
//model.wPrune(params(2*inputDim + 4));

    return model;
}

Eigen::VectorXd LWPROptimizeParameters(size_t inputDim,
    const Eigen::VectorXd& params, 
    const std::vector<Eigen::VectorXd> trainInputs,
    const std::vector<double> trainOutputs,
    const std::vector<Eigen::VectorXd> testInputs,
    const std::vector<double> testOutputs,
    unsigned int maxIteration,
    bool isQuiet)
{
    //Checks
    if (trainInputs.size() == 0 || testInputs.size() == 0) {
        throw std::logic_error("LWPRUtils empty train or test");
    }
    if (
        trainInputs.size() != trainOutputs.size() || 
        testInputs.size() != testOutputs.size()
    ) {
        throw std::logic_error("LWPRUtils invalid train or test size");
    }
    if (
        (size_t)trainInputs.front().size() != inputDim ||
        (size_t)testInputs.front().size() != inputDim
    ) {
        throw std::logic_error("LWPRUtils invalid train or test dim");
    }
    if ((size_t)params.size() != 2*inputDim+5) {
        throw std::logic_error("LWPRUtils invalid params dim");
    }

    //Starting point conversion to vector
    std::vector<double> x0(params.size());
    for (size_t i=0;i<(size_t)params.size();i++) {
        x0[i] = params(i);
    }
    //Optimization init
    libcmaes::CMAParameters<> cmaparams(x0, 0.1);
    cmaparams.set_quiet(isQuiet);
    cmaparams.set_mt_feval(true);
    cmaparams.set_str_algo("acmaes");
    cmaparams.set_max_iter(maxIteration);
    
    //Fitness function
    libcmaes::FitFunc fitness = 
        [&inputDim, &trainInputs, &trainOutputs, &testInputs, &testOutputs]
        (const double* x, const int N) 
    {
        //Conversion to Eigen
        Eigen::VectorXd params(N);
        for (size_t i=0;i<(size_t)N;i++) {
            params(i) = x[i];
        }
        //Init LWPR model
        LWPR_Object model = LWPRInit(inputDim, params);
        //Train LWMR
        for (size_t i=0;i<trainInputs.size();i++) {
            model.update(trainInputs[i], trainOutputs[i]);
        }
        //Test LWPR
        double mse = 0.0;
        for (size_t i=0;i<testInputs.size();i++) {
            Eigen::VectorXd yp = model.predict(testInputs[i]);
            mse += pow(yp(0)-testOutputs[i], 2);
        }
        mse /= (double)testInputs.size();

        //Penalize negative parameters
        double cost = mse;
        for (size_t i=0;i<(size_t)params.size();i++) {
            if (params(i) <= 0.0) {
                cost += 100.0 + 1000.0*fabs(params(i));
            }
        }
        //Penalize >= 1 wGen
        if (params(params.size()-2) >= 1.0) {
            cost += 100.0 + 1000.0*fabs(params(params.size()-2)-1.0);
        }

        return cost;
    };
    
    //Run optimization
    libcmaes::CMASolutions cmasols = libcmaes::cmaes<>(fitness, cmaparams);
    Eigen::VectorXd bestParams = cmasols.best_candidate().get_x_dvec();

    return bestParams;
}

void LWPRPrint(const LWPR_Object& model)
{
    //Print model general information
    std::cout << "LWPR model:" << std::endl;
    std::cout << "nIn: " << model.nIn() << std::endl;
    std::cout << "nOut: " << model.nOut() << std::endl;
    std::cout << "nData: " << model.nData() << std::endl;
    std::cout << "numRFS: " << model.numRFS() << std::endl;
    
    //Compute and print receptive field information
    size_t countRFTrustworthy = 0;
    size_t sumRFReg = 0;
    size_t minRFReg = 0;
    size_t maxRFReg = 0;
    Eigen::VectorXd sumRFVIP(model.nIn());
    sumRFVIP.setZero();
    Eigen::VectorXd minRFVIP(model.nIn());
    Eigen::VectorXd maxRFVIP(model.nIn());

    bool isInit = false;
    for (size_t i=0;i<(size_t)model.numRFS();i++) {
        const LWPR_ReceptiveFieldObject& rf = model.getRF(i);
        if (!rf.trustworthy()) {
            continue;
        }
        if (!isInit) {
            minRFReg = rf.nReg();
            maxRFReg = rf.nReg();
            minRFVIP = rf.vip();
            maxRFVIP = rf.vip();
            isInit = true;
        }
        countRFTrustworthy++;
        sumRFReg += rf.nReg();
        if (minRFReg > (size_t)rf.nReg()) minRFReg = rf.nReg();
        if (maxRFReg < (size_t)rf.nReg()) maxRFReg = rf.nReg();
        sumRFVIP += rf.vip();
        for (size_t i=0;i<(size_t)model.nIn();i++) {
            Eigen::VectorXd v = rf.vip();
            if (minRFVIP(i) > v(i)) minRFVIP(i) = v(i);
            if (maxRFVIP(i) < v(i)) maxRFVIP(i) = v(i);
        }
        /*
        if (model.numRFS() < 5) {
            std::cout << "RF:" << i << std::endl;
            std::cout << "center=" << std::endl;
            std::cout << rf.center().transpose() << std::endl;
            std::cout << "D=" << std::endl;
            std::cout << rf.D() << std::endl;
        }
        */
    }
    std::cout << "RF Trustworthy: " << countRFTrustworthy << std::endl;
    std::cout << "RF min nReg: " << minRFReg << std::endl;
    std::cout << "RF max nReg: " << maxRFReg << std::endl;
    std::cout << "RF mean nReg: " << 
        (double)sumRFReg/(double)countRFTrustworthy << std::endl;
    for (size_t i=0;i<(size_t)model.nIn();i++) {
        std::cout << "RF VIP In=" << i << ":";
        std::cout << " mean=" << sumRFVIP(i)/(double)countRFTrustworthy;
        std::cout << " min=" << minRFVIP(i);
        std::cout << " max=" << maxRFVIP(i);
        std::cout << std::endl;
    }
}

size_t LWPRRFTrustworthy(const LWPR_Object& model)
{
    size_t countRFTrustworthy = 0;
    for (size_t i=0;i<(size_t)model.numRFS();i++) {
        const LWPR_ReceptiveFieldObject& rf = model.getRF(i);
        if (rf.trustworthy()) {
            countRFTrustworthy++;
        }
    }
    return countRFTrustworthy;
}
size_t LWPRMinRFReg(const LWPR_Object& model)
{
    size_t minRFReg = (size_t)-1;
    for (size_t i=0;i<(size_t)model.numRFS();i++) {
        const LWPR_ReceptiveFieldObject& rf = model.getRF(i);
        if (!rf.trustworthy()) {
            continue;
        }
        if (minRFReg == (size_t)-1 || 
            minRFReg > (size_t)rf.nReg()
        ) {
            minRFReg = rf.nReg();
        }
    }
    return minRFReg;
}
size_t LWPRMaxRFReg(const LWPR_Object& model)
{
    size_t maxRFReg = (size_t)-1;
    for (size_t i=0;i<(size_t)model.numRFS();i++) {
        const LWPR_ReceptiveFieldObject& rf = model.getRF(i);
        if (!rf.trustworthy()) {
            continue;
        }
        if (maxRFReg == (size_t)-1 || 
            maxRFReg < (size_t)rf.nReg()
        ) {
            maxRFReg = rf.nReg();
        }
    }
    return maxRFReg;
}

void LWPRPrintParameters(size_t inputDim, 
    const Eigen::VectorXd& params)
{
    if ((size_t)params.size() != 2*inputDim + 5) {
        throw std::logic_error("LWPRUtils invalid dim");
    }

    std::cout << "LWPR parameters:" << std::endl;
    std::cout << "NormIn (nIn): " 
        << params.segment(0, inputDim).transpose() << std::endl;
    std::cout << "InitAlpha: " << params(inputDim) << std::endl;
    std::cout << "MetaRate: " << params(inputDim + 1) << std::endl;
    std::cout << "Penalty: " << params(inputDim + 2) << std::endl;
    std::cout << "InitD (nIn): " 
        << params.segment(inputDim + 3, inputDim).transpose() << std::endl;
    std::cout << "WGen: " << params(2*inputDim + 3) << std::endl;
    std::cout << "WPrune: " << params(2*inputDim + 4) << std::endl;
}

}

