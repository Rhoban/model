#ifndef LEPH_REGRESSIONLWPR_HPP
#define LEPH_REGRESSIONLWPR_HPP

#include <Eigen/Dense>
#include <lwpr_eigen.hpp>
#include "TimeSeries/Regression.hpp"

namespace Leph
{
/**
 * RegressionLWPR
 *
 * Implementation of Regression class using
 * LWPR non linear non parametric regression
 * algorithm.
 */
class RegressionLWPR : public Regression
{
public:
  /**
   * Initialization
   */
  RegressionLWPR() : Regression(), _model(nullptr)
  {
  }

  /**
   * Inherit Optimizable
   */
  virtual inline size_t parameterSize() const override
  {
    return Regression::inputSize() + 2;
  }
  virtual inline MetaParameter defaultParameter(size_t index) const override
  {
    size_t size = Regression::inputSize();
    MetaParameter param;
    if (index < size)
    {
      param = MetaParameter("NormIn", 1.0);
      param.setMinimum(0.01);
    }
    if (index == size)
    {
      param = MetaParameter("Penalty", 1e-6);
      param.setMinimum(0.0);
    }
    if (index == size + 1)
    {
      param = MetaParameter("InitD", 1.0);
      param.setMinimum(0.01);
    }

    return param;
  }

  /**
   * Inherit Regression
   */
  virtual inline bool learn(double time) override
  {
    Eigen::VectorXd in = retrieveInputs(time);
    if ((size_t)in.size() == inputSize() && Regression::getOutput()->isTimeValid(time))
    {
      _model->update(in, Regression::getOutput()->get(time));
      return true;
    }
    else
    {
      return false;
    }
  }
  virtual inline double predict(double time) const override
  {
    Eigen::VectorXd in = retrieveInputs(time);
    if ((size_t)in.size() == inputSize())
    {
      if (!isRegressionValid())
      {
        std::cout << "WARNING prediction with invalid regression" << std::endl;
        return in(0);  // TODO
      }
      /*
      Eigen::VectorXd confidence;
      Eigen::VectorXd maxW;
      Eigen::VectorXd yp = _model->predict(in, confidence, maxW, 0.000);
      return yp(0);
      */
      double yp = _model->predict(in, 0.0)(0);
      return yp;
    }
    else
    {
      throw std::runtime_error("RegressionLWPR invalid time");
    }
  }

  /**
   * Initialize an new LWPR model
   * object using current meta parameters
   * Inherit Regression
   */
  inline virtual void resetRegression() override
  {
    size_t size = Regression::inputSize();

    try
    {
      // InitializeLWPR model with
      // input dimension
      if (_model != nullptr)
      {
        delete _model;
      }
      _model = new LWPR_Object(size);

      // Normalisation value for each input dimension (>0)
      Eigen::VectorXd vect(size);
      for (size_t i = 0; i < size; i++)
      {
        vect(i) = Optimizable::getParameter(i).value();
      }
      _model->normIn(vect);
      // Use only diagonal matrix. Big speed up in high dimension
      // but lower performance in complex learning.
      _model->diagOnly(false);
      // Automatic tunning of distance metric
      _model->useMeta(true);
      // Larger value enforce wider receptive field (>0)
      _model->penalty(Optimizable::getParameter(size).value());
      // Set diagonal (input_dim) or complet (input_dim*input_dim)
      // initial distance matrix (>0)
      _model->setInitD(Optimizable::getParameter(size + 1).value());
    }
    catch (const LWPR_Exception& e)
    {
      Optimizable::parameterPrint();
      throw std::logic_error("RegressionLWPR parameters exception: " + std::string(e.getString()));
    }
  }

  /**
   * Inherit Regression
   */
  inline virtual bool isRegressionValid() const override
  {
    // Check if there is at least one
    // valid receptive field
    size_t countRFTrustworthy = 0;
    for (size_t i = 0; i < (size_t)_model->numRFS(); i++)
    {
      const LWPR_ReceptiveFieldObject& rf = _model->getRF(i);
      if (rf.trustworthy())
      {
        countRFTrustworthy++;
      }
    }

    return (countRFTrustworthy > 0);
  }

  /**
   * Access to internal LWPR model
   * for LWPR related specific query
   */
  inline const LWPR_Object& model() const
  {
    return *_model;
  }
  inline LWPR_Object& model()
  {
    return *_model;
  }

  /**
   * Returned as an Eigen Vector the input
   * at given time with declared lag offset.
   * A zero sized vector is returned if
   * inputs are not all available.
   */
  inline Eigen::VectorXd retrieveInputs(double time) const
  {
    size_t size = Regression::inputSize();
    Eigen::VectorXd vect(size);

    // Parse all registered inputs
    for (size_t i = 0; i < size; i++)
    {
      const TimeSeries* series = Regression::getInput(i).series;
      if (Regression::getInput(i).isDeltaTime)
      {
        double delta = Regression::getInput(i).deltaTime;
        if (series->isTimeValid(time - delta))
        {
          vect(i) = series->get(time - delta);
        }
        else
        {
          // No result if asked lagged time is not available
          return Eigen::VectorXd();
        }
      }
      else
      {
        size_t delta = Regression::getInput(i).deltaIndex;
        if (!series->isTimeValid(time))
        {
          return Eigen::VectorXd();
        }
        size_t index = series->getClosestIndex(time);
        /*
        double t = series->at(index).time;
        if (fabs(time-t) < TIME_EPSILON && index+delta < series->size()) {
           vect(i) = series->at(index+delta).value;
        } else {
            std::cout << "WARNING regression input no near value" << std::endl; //TODO
            return Eigen::VectorXd();
        }
        */
        vect(i) = series->at(index + delta).value;
      }
    }

    return vect;
  }

  /**
   * Write and read in given file path
   * to save and load LWPR model.
   * Note that meta parameters are not yet updated
   * on load.
   */
  inline void save(const std::string& filepath) const
  {
    if (inputSize() == 0 || Regression::getOutput() == nullptr)
    {
      throw std::logic_error("Regression not initialized");
    }

    try
    {
      _model->writeBinary(filepath.c_str());
    }
    catch (const LWPR_Exception& e)
    {
      throw std::logic_error("RegressionLWPR exception save: " + std::string(e.getString()));
    }
  }
  inline void load(const std::string& filepath)
  {
    if (inputSize() == 0 || Regression::getOutput() == nullptr)
    {
      throw std::logic_error("Regression not initialized");
    }

    // InitializeLWPR model with
    // input dimension
    if (_model != nullptr)
    {
      delete _model;
    }
    try
    {
      _model = new LWPR_Object(filepath.c_str());
    }
    catch (const LWPR_Exception& e)
    {
      throw std::logic_error("RegressionLWPR exception load: " + std::string(e.getString()));
    }

    // Check loaded model consistancy
    if ((size_t)_model->nIn() != inputSize() || (size_t)_model->nOut() != 1)
    {
      throw std::runtime_error("RegressionLWPR load invalid model");
    }
  }

protected:
  virtual inline void onAddInput() override
  {
    // Reset meta parameters
    Optimizable::resetParameters();
    // Reset the internal LWPR model
    resetRegression();
  }

private:
  /**
   * LWPR learned model
   */
  mutable LWPR_Object* _model;
};

}  // namespace Leph

#endif
