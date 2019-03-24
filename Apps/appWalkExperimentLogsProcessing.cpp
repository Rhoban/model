#include <iostream>
#include <stdexcept>
#include <string>
#include <fstream>
#include <vector>
#include "Types/VectorLabel.hpp"
#include "Types/MatrixLabel.hpp"
#include "Plot/Plot.hpp"
#include "Gradient/FiniteDifferenceGradient.hpp"
#include "CartWalk/CartWalkProxy.hpp"

/**
 * Time serie range structure
 */
struct Range
{
  // Bound
  size_t begin;
  size_t end;

  // Constructor
  Range(size_t b, size_t e) : begin(b), end(e)
  {
    if (e < b)
    {
      throw std::logic_error("Range invalid bound");
    }
  }
};

/**
 * Compute the standart deviation of given range
 * for given variable
 */
double computeFitness(size_t begin, size_t end, const std::string& sensorName, const Leph::MatrixLabel& container)
{
  double sum = 0.0;
  double sum2 = 0.0;
  double count = 0.0;
  for (size_t j = begin; j < end; j++)
  {
    sum += container[j](sensorName);
    sum2 += container[j](sensorName) * container[j](sensorName);
    count += 1.0;
  }
  double mean = sum / count;
  double variance = sum2 / count - mean * mean;

  return sqrt(variance);
}

int main(int argc, char** argv)
{
  // Usage
  if (argc != 2)
  {
    std::cout << "Usage: ./app <log-file.csv>" << std::endl;
    return 1;
  }

  // Data loading
  Leph::MatrixLabel container;
  std::cout << "Loading data..." << std::endl;
  container.load(argv[1]);
  std::cout << container.size() << " points (~" << container.size() / 50 / 60 << " mins)" << std::endl;

  // Split parameters rollout
  std::vector<Range> paramsSeq;
  for (size_t i = 0; i < container.size();)
  {
    size_t indexBegin = i;
    paramsSeq.push_back(Range(i, container.size() - 1));
    while (i < container.size())
    {
      if (!Leph::VectorLabel::isEqualInter(container[i], container[indexBegin], "static"))
      {
        paramsSeq.back().end = i - 1;
        break;
      }
      else
      {
        i++;
      }
    }
  }
  std::cout << paramsSeq.size() << " parameters sequences" << std::endl;

  // Split stable rollout
  std::vector<Range> stablesSeq;
  for (size_t i = 0; i < container.size();)
  {
    while (i < container.size() && container[i]("fitness:isStable") == 0.0)
    {
      i++;
    }
    if (i == container.size())
      break;
    stablesSeq.push_back(Range(i, container.size() - 1));
    while (i < container.size() && container[i]("fitness:isStable") == 1.0)
    {
      i++;
    }
    stablesSeq.back().end = i - 1;
  }
  std::cout << stablesSeq.size() << " stable sequences" << std::endl;

  // Filter stable and non stable walk sequences
  std::vector<Range> stableParamsSeq;
  std::vector<Range> nonStableParamsSeq;
  for (size_t i = 0; i < paramsSeq.size(); i++)
  {
    bool isStable = false;
    for (size_t j = 0; j < stablesSeq.size(); j++)
    {
      if (stablesSeq[j].begin >= paramsSeq[i].begin && stablesSeq[j].end <= paramsSeq[i].end &&
          stablesSeq[j].end - stablesSeq[i].begin + 1 >= 20 * 50)
      {
        isStable = true;
      }
    }
    if (isStable)
    {
      stableParamsSeq.push_back(paramsSeq[i]);
    }
    else
    {
      nonStableParamsSeq.push_back(paramsSeq[i]);
    }
  }
  std::cout << stableParamsSeq.size() << " stable params Seq" << std::endl;
  std::cout << nonStableParamsSeq.size() << " non stable params Seq" << std::endl;

  /*
  Leph::Plot plot2;
  long countSeq = 0;
  long countPts = 0;
  for (size_t i=0;i<stablesSeq.size();i+=5) {
      if (stablesSeq[i].end - stablesSeq[i].begin > 50*15) {
          countSeq++;
          std::cout << "Seq: " << stablesSeq[i].begin << " -- " << stablesSeq[i].end << std::endl;
          for (size_t j=stablesSeq[i].begin;j<stablesSeq[i].end;j++) {
                  container[j].append("seq", i);
                  plot2.add(container[j]);
                  countPts++;
          }
      }
  }
  std::cout << "CountSeq: " << countSeq << std::endl;
  std::cout << "CountPts: " << countPts << std::endl;
  plot2.plot("info:phase", "sensor:AccY", Leph::Plot::Points, "seq").render();
  plot2.plot("info:phase", "sensor:AccX", Leph::Plot::Points, "seq").render();
  plot2.plot("info:phase", "sensor:GyroX", Leph::Plot::Points, "seq").render();
  plot2.plot("info:phase", "sensor:GyroY", Leph::Plot::Points, "seq").render();
  plot2.plot("info:phase", "error:lateral", Leph::Plot::Points, "seq").render();
  plot2.plot("info:phase", "error:step", Leph::Plot::Points, "seq").render();
  */

  /*
  Leph::Plot plot2;
  long countSeq = 0;
  long countPts = 0;
  for (size_t i=0;i<stablesSeq.size();i++) {
      if (stablesSeq[i].end - stablesSeq[i].begin > 50*15) {
          countSeq++;
          std::cout << "Seq: " << stablesSeq[i].begin << " -- " << stablesSeq[i].end << std::endl;
          Leph::MatrixLabel tmp;
          for (size_t j=stablesSeq[i].begin;j<stablesSeq[i].end;j++) {
              if (container[j]("info:phase") > 0.60 && container[j]("info:phase") < 0.63) {
                  container[j].append("seq", i);
                  tmp.append(container[j]);
                  countPts++;
              }
          }
          Leph::VectorLabel tmpMean = tmp.mean();
          Leph::VectorLabel tmpVar = tmp.stdDev();
          tmpMean.append("mean", tmpMean("sensor:AccY"));
          plot2.add(tmpMean);
          tmpMean.addOp(tmpVar);
          tmpMean.append("varsup", tmpMean("sensor:AccY"));
          plot2.add(tmpMean);
          tmpMean.subOp(tmpVar);
          tmpMean.subOp(tmpVar);
          tmpMean.append("varinf", tmpMean("sensor:AccY"));
          plot2.add(tmpMean);
      }
  }
  std::cout << "CountSeq: " << countSeq << std::endl;
  std::cout << "CountPts: " << countPts << std::endl;
  plot2
      .plot("seq", "mean", Leph::Plot::LinesPoints)
      .plot("seq", "varsup", Leph::Plot::LinesPoints)
      .plot("seq", "varinf", Leph::Plot::LinesPoints)
      //.plot("seq", "mean", Leph::Plot::LinesPoints, "static:swingGain")
      //.plot("seq", "varsup", Leph::Plot::LinesPoints, "static:swingGain")
      //.plot("seq", "varinf", Leph::Plot::LinesPoints, "static:swingGain")
      .render();
  //plot2.plot("info:phase", "sensor:AccY", Leph::Plot::Points, "seq").render();
  container.range(stablesSeq[0].begin, stablesSeq[0].end).plot().plot("info:phase", "sensor:AccY",
  Leph::Plot::Points).render(); container.range(stablesSeq[15].begin, stablesSeq[15].end).plot().plot("info:phase",
  "sensor:AccY", Leph::Plot::Points).render(); container.range(stablesSeq[2].begin,
  stablesSeq[2].end).plot().plot("info:phase", "sensor:AccY", Leph::Plot::Points).render();
  */

  // Try to find stable/unstable simple axis aligned
  // partition
  /*
  Leph::VectorLabel countStaticParamsStableUp = tmpStaticParamsBegin;
  Leph::VectorLabel countStaticParamsStableDown = tmpStaticParamsBegin;
  Leph::VectorLabel countStaticParamsUnstableUp = tmpStaticParamsBegin;
  Leph::VectorLabel countStaticParamsUnstableDown = tmpStaticParamsBegin;
  countStaticParamsStableUp.vect() = Leph::Vector::Zero(tmpStaticParamsBegin.size());
  countStaticParamsStableDown.vect() = Leph::Vector::Zero(tmpStaticParamsBegin.size());
  countStaticParamsUnstableUp.vect() = Leph::Vector::Zero(tmpStaticParamsBegin.size());
  countStaticParamsUnstableDown.vect() = Leph::Vector::Zero(tmpStaticParamsBegin.size());
  for (size_t i=0;i<stableParamsSeq.size();i++) {
      for (size_t j=0;j<tmpStaticParamsBegin.size();j++) {
          const std::string& label = tmpStaticParamsBegin.getLabel(j);
          if (container[stableParamsSeq[i].begin](label) < container.front()(label)-0.0001) {
              countStaticParamsStableDown(label)++;
          }
          if (container[stableParamsSeq[i].begin](label) > container.front()(label)+0.0001) {
              countStaticParamsStableUp(label)++;
          }
      }
  }
  for (size_t i=0;i<nonStableParamsSeq.size();i++) {
      for (size_t j=0;j<tmpStaticParamsBegin.size();j++) {
          const std::string& label = tmpStaticParamsBegin.getLabel(j);
          if (container[stableParamsSeq[i].begin](label) < container.front()(label)-0.0001) {
              countStaticParamsUnstableDown(label)++;
          }
          if (container[stableParamsSeq[i].begin](label) > container.front()(label)+0.0001) {
              countStaticParamsUnstableUp(label)++;
          }
      }
  }
  std::cout << countStaticParamsStableDown << std::endl;
  std::cout << countStaticParamsStableUp << std::endl;
  std::cout << countStaticParamsUnstableDown << std::endl;
  std::cout << countStaticParamsUnstableUp << std::endl;
  */

  //
  /*
  for (size_t i=0;i<stableParamsSeq.size();i++) {
      Leph::Plot plot;
      for (size_t j=stableParamsSeq[i].begin;j<stableParamsSeq[i].end;j++) {
          plot.add(container[j]);
      }
      std::cout << "index: " << i << std::endl;
      plot.plot("index", "mocap:isValid").render();
  }
  */

  // Range of fitness candidate serie for each stable rollout
  /*
  for (size_t i=0;i<stableParamsSeq.size();i++) {
      for (size_t j=stableParamsSeq[i].end-(50*30);j<stableParamsSeq[i].end-(50*5);j++) {
          plot.add(
              Leph::VectorLabel::mergeUnion(container[j], Leph::VectorLabel("seq", i + (i%2)*30)));
      }
  }
  plot
      //.plot("index", "fitness:mocap lateral", Leph::Plot::LinesPoints, "seq")
      //.plot("index", "fitness:mocap turn", Leph::Plot::LinesPoints, "seq")
      //.plot("index", "fitness:gyro x", Leph::Plot::LinesPoints, "seq")
      //.plot("index", "fitness:gyro z", Leph::Plot::LinesPoints, "seq")
      //.plot("index", "fitness:pitch", Leph::Plot::LinesPoints, "seq")
      //.plot("index", "fitness:roll", Leph::Plot::LinesPoints, "seq")
      //.plot("index", "fitness:*", Leph::Plot::LinesPoints, "seq")
      //.plot("index", "error:lateral", Leph::Plot::LinesPoints, "seq")
      .plot("index", "error:step", Leph::Plot::LinesPoints, "seq")
      .render();
  plot
      .plot("index", "error:lateral", Leph::Plot::LinesPoints, "seq")
      .plot("index", "fitness:mocap lateral", Leph::Plot::LinesPoints, "seq")
      .render();
  */

  /*
  stableParamsSeq.erase(stableParamsSeq.begin() + 23);
  stableParamsSeq.erase(stableParamsSeq.begin() + 20);
  stableParamsSeq.erase(stableParamsSeq.begin() + 11);
  stableParamsSeq.erase(stableParamsSeq.begin() + 8);
  for (size_t i=0;i<stableParamsSeq.size();i++) {
      size_t index = stableParamsSeq[i].end-(5*50);
      std::cout << i << " --> " << container[index]("fitness:mocap turn") << std::endl;
  }
  */

  // Scalar fitness candidate for each stable rollout
  /*
  plot.clear();
  for (size_t i=0;i<stableParamsSeq.size();i++) {
      size_t indexRef = stableParamsSeq[0].end-(5*50);
      size_t index = stableParamsSeq[i].end-(5*50);
      Leph::VectorLabel tmp = container[index];
      tmp.divOp(container[indexRef]);
      tmp.append("one", 1.0);
      plot.add(tmp);
  }
  plot
      .plot("index", "fitness:*")
      .plot("index", "fitness:isValid", Leph::Plot::None)
      .plot("index", "one", Leph::Plot::Lines)
      .render();
  */

  // Compute gradient for all fitness candidate
  /*
  Leph::VectorLabel subsetStaticParams({
      "static:hipOffset",
      "static:riseGain",
      "static:swingGain",
      "static:swingPhase",
      "static:timeGain",
      "static:xOffset",
      "static:yOffset",
      "static:zOffset"});
  Leph::VectorLabel lengthScale = subsetStaticParams;
  lengthScale.mergeInter(Leph::CartWalkProxy().buildParamsDelta());
  lengthScale.mulOp(2.0);
  Leph::Vector oldGradient;

  {
  oldGradient = Leph::Vector::Zero(subsetStaticParams.size());
  plot.clear();
  std::cout << "GYRO X" << std::endl;
  Leph::FiniteDifferenceGradient gradient;
  for (size_t i=0;i<stableParamsSeq.size();i++) {
      size_t index = stableParamsSeq[i].end - (5*50);
      double fitness = container[index]("fitness:gyro x");
      subsetStaticParams.mergeInter(container[index]);
      subsetStaticParams.subOp(container[0]);
      subsetStaticParams.divOp(lengthScale);
      gradient.addExperiment(subsetStaticParams.vect(), fitness);
      if (i > 5) {
      plot.add(Leph::VectorLabel("deltaNorm",
          (gradient.gradient()-oldGradient).norm()));
      }
      oldGradient = gradient.gradient();
  }
  subsetStaticParams.vect() = oldGradient;
  std::cout << subsetStaticParams << std::endl;
  plot.plot("index", "deltaNorm").render();
  }

  {
  oldGradient = Leph::Vector::Zero(subsetStaticParams.size());
  plot.clear();
  std::cout << "GYRO Z" << std::endl;
  Leph::FiniteDifferenceGradient gradient;
  for (size_t i=0;i<stableParamsSeq.size();i++) {
      size_t index = stableParamsSeq[i].end - (5*50);
      double fitness = container[index]("fitness:gyro z");
      subsetStaticParams.mergeInter(container[index]);
      subsetStaticParams.subOp(container[0]);
      subsetStaticParams.divOp(lengthScale);
      gradient.addExperiment(subsetStaticParams.vect(), fitness);
      if (i > 5) {
      plot.add(Leph::VectorLabel("deltaNorm",
          (gradient.gradient()-oldGradient).norm()));
      }
      oldGradient = gradient.gradient();
  }
  subsetStaticParams.vect() = oldGradient;
  std::cout << subsetStaticParams << std::endl;
  plot.plot("index", "deltaNorm").render();
  }

  {
  oldGradient = Leph::Vector::Zero(subsetStaticParams.size());
  plot.clear();
  std::cout << "PITCH" << std::endl;
  Leph::FiniteDifferenceGradient gradient;
  for (size_t i=0;i<stableParamsSeq.size();i++) {
      size_t index = stableParamsSeq[i].end - (5*50);
      double fitness = container[index]("fitness:pitch");
      subsetStaticParams.mergeInter(container[index]);
      subsetStaticParams.subOp(container[0]);
      subsetStaticParams.divOp(lengthScale);
      gradient.addExperiment(subsetStaticParams.vect(), fitness);
      if (i > 5) {
      plot.add(Leph::VectorLabel("deltaNorm",
          (gradient.gradient()-oldGradient).norm()));
      }
      oldGradient = gradient.gradient();
  }
  subsetStaticParams.vect() = oldGradient;
  std::cout << subsetStaticParams << std::endl;
  plot.plot("index", "deltaNorm").render();
  }

  {
  oldGradient = Leph::Vector::Zero(subsetStaticParams.size());
  plot.clear();
  std::cout << "ROLL" << std::endl;
  Leph::FiniteDifferenceGradient gradient;
  for (size_t i=0;i<stableParamsSeq.size();i++) {
      size_t index = stableParamsSeq[i].end - (5*50);
      double fitness = container[index]("fitness:roll");
      subsetStaticParams.mergeInter(container[index]);
      subsetStaticParams.subOp(container[0]);
      subsetStaticParams.divOp(lengthScale);
      gradient.addExperiment(subsetStaticParams.vect(), fitness);
      if (i > 5) {
      plot.add(Leph::VectorLabel("deltaNorm",
          (gradient.gradient()-oldGradient).norm()));
      }
      oldGradient = gradient.gradient();
  }
  subsetStaticParams.vect() = oldGradient;
  std::cout << subsetStaticParams << std::endl;
  plot.plot("index", "deltaNorm").render();
  }

  {
  oldGradient = Leph::Vector::Zero(subsetStaticParams.size());
  plot.clear();
  std::cout << "MOCAP LATERAL" << std::endl;
  Leph::FiniteDifferenceGradient gradient;
  for (size_t i=0;i<stableParamsSeq.size();i++) {
      size_t index = stableParamsSeq[i].end - (5*50);
      double fitness = container[index]("fitness:mocap lateral");
      subsetStaticParams.mergeInter(container[index]);
      subsetStaticParams.subOp(container[0]);
      subsetStaticParams.divOp(lengthScale);
      gradient.addExperiment(subsetStaticParams.vect(), fitness);
      if (i > 5) {
      plot.add(Leph::VectorLabel("deltaNorm",
          (gradient.gradient()-oldGradient).norm()));
      }
      oldGradient = gradient.gradient();
  }
  subsetStaticParams.vect() = oldGradient;
  std::cout << subsetStaticParams << std::endl;
  plot.plot("index", "deltaNorm").render();
  }
  */

  /*
  for (size_t i=0;i<stableParamsSeq.size();i++) {
      container.range(stableParamsSeq[i].begin, stableParamsSeq[i].end-5*50)
          .plot().plot("index", "error:lateral").render();
  }
  */

  std::vector<std::string> sensorsNames = {
    "error:lateral", "error:step",   "error:turn",   "sensor:AccX", "sensor:AccY",
    "sensor:AccZ",   "sensor:GyroX", "sensor:GyroY", "sensor:Roll", "sensor:Pitch",
  };
  Leph::MatrixLabel fitnesses(stableParamsSeq.size());
  for (const std::string& sensorName : sensorsNames)
  {
    for (size_t i = 0; i < stableParamsSeq.size(); i++)
    {
      double fitness = computeFitness(stableParamsSeq[i].begin, stableParamsSeq[i].end, sensorName, container);
      fitnesses[i].append(sensorName, fitness);
    }
  }

  Leph::VectorLabel initFitness = fitnesses[0];
  fitnesses.subOp(initFitness);
  Leph::VectorLabel stddevFitness = fitnesses.stdDev();
  fitnesses.divOp(stddevFitness);
  /*
  for (size_t i=0;i<fitnesses.size();i++) {
      fitnesses[i]("error:lateral") /= 0.00874;
      fitnesses[i]("error:step") /= 0.0452;
      fitnesses[i]("error:turn") /= 4.24;
  }
  */

  for (size_t i = 0; i < fitnesses.size(); i++)
  {
    std::cout << "[" << i << "] ";
    for (const std::string& sensorName : sensorsNames)
    {
      if (fitnesses[i](sensorName) < fitnesses[0](sensorName))
      {
        std::cout << sensorName << " ";
      }
    }
    std::cout << std::endl;
  }
  for (const std::string& sensorName : sensorsNames)
  {
    std::cout << "[" << sensorName << "] ";
    for (size_t i = 0; i < fitnesses.size(); i++)
    {
      if (fitnesses[i](sensorName) < fitnesses[0](sensorName))
      {
        std::cout << i << " ";
      }
    }
    std::cout << std::endl;
  }

  fitnesses.plot().plot("index", "all").render();

  for (size_t i = 0; i < fitnesses.size(); i++)
  {
    fitnesses[i].append("sum:error", 0.0);
    fitnesses[i].append("sum:acc", 0.0);
    fitnesses[i].append("sum:gyro", 0.0);
    fitnesses[i].append("sum:angle", 0.0);
    fitnesses[i].append("sum:all", 0.0);
    for (const std::string& sensorName : { "error:lateral" })
    {
      fitnesses[i]("sum:error") += fitnesses[i](sensorName);
    }
    for (const std::string& sensorName : { "sensor:AccX", "sensor:AccY", "sensor:AccZ" })
    {
      fitnesses[i]("sum:acc") += fitnesses[i](sensorName);
    }
    for (const std::string& sensorName : { "sensor:GyroX", "sensor:GyroY" })
    {
      fitnesses[i]("sum:gyro") += fitnesses[i](sensorName);
    }
    for (const std::string& sensorName : { "sensor:Roll", "sensor:Pitch" })
    {
      fitnesses[i]("sum:angle") += fitnesses[i](sensorName);
    }
    for (const std::string& sensorName : sensorsNames)
    {
      fitnesses[i]("sum:all") += fitnesses[i](sensorName) / sensorsNames.size();
    }
  }

  fitnesses.plot().plot("index", "sum:*").render();
  fitnesses.plot().plot("index", "sum:all").render();

  Leph::VectorLabel subsetStaticParams({ "static:hipOffset", "static:riseGain", "static:swingGain", "static:swingPhase",
                                         "static:timeGain", "static:xOffset", "static:yOffset", "static:zOffset" });
  Leph::VectorLabel lengthScale = subsetStaticParams;
  lengthScale.mergeInter(Leph::CartWalkProxy().buildParamsDelta());
  lengthScale.mulOp(2.0);
  Leph::Vector oldGradient;

  {
    Leph::Plot plot;
    oldGradient = Leph::Vector::Zero(subsetStaticParams.size());
    plot.clear();
    Leph::FiniteDifferenceGradient gradient;
    for (size_t i = 0; i < stableParamsSeq.size(); i++)
    {
      double fitness = fitnesses[i]("sum:all");
      subsetStaticParams.mergeInter(container[stableParamsSeq[i].begin]);
      subsetStaticParams.subOp(container[0]);
      subsetStaticParams.divOp(lengthScale);
      gradient.addExperiment(subsetStaticParams.vect(), fitness);
      if (i > 5)
      {
        plot.add(Leph::VectorLabel("deltaNorm", (gradient.gradient() - oldGradient).norm()));
      }
      oldGradient = gradient.gradient();
    }
    subsetStaticParams.vect() = oldGradient;
    std::cout << subsetStaticParams << std::endl;

    for (size_t i = 0; i < nonStableParamsSeq.size(); i++)
    {
      double fitness = 4.0;
      subsetStaticParams.mergeInter(container[nonStableParamsSeq[i].begin]);
      subsetStaticParams.subOp(container[0]);
      subsetStaticParams.divOp(lengthScale);
      gradient.addExperiment(subsetStaticParams.vect(), fitness);
      if (i > 5)
      {
        plot.add(Leph::VectorLabel("deltaNorm", (gradient.gradient() - oldGradient).norm()));
      }
      oldGradient = gradient.gradient();
    }
    subsetStaticParams.vect() = oldGradient;
    std::cout << subsetStaticParams << std::endl;

    plot.plot("index", "deltaNorm").render();
  }
}
