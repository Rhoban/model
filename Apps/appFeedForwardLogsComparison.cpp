#include <iostream>
#include <string>
#include "Types/MapSeries.hpp"
#include "Plot/Plot.hpp"
#include "Model/NamesModel.h"

/**
 * Processing data and comparison between
 * feed forward and correction by optimization
 * of position tracking
 */
int main()
{
  std::string fileOriginal = "../../These/Data/logs-2016-12-19_feedforward/logOriginal";
  std::string fileFeedForward = "../../These/Data/logs-2016-12-19_feedforward/logFeedForward";
  std::string fileCorrected = "../../These/Data/logs-2016-12-19_feedforward/logCorrected";

  Leph::MapSeries seriesOriginal;
  Leph::MapSeries seriesFeedForward;
  Leph::MapSeries seriesCorrected;

  seriesOriginal.importData(fileOriginal);
  seriesFeedForward.importData(fileFeedForward);
  seriesCorrected.importData(fileCorrected);

  double timeBeginOriginal = seriesOriginal.timeMin();
  double timeBeginFeedForward = seriesFeedForward.timeMin();
  double timeBeginCorrected = seriesCorrected.timeMin();
  double timeEndOriginal = seriesOriginal.timeMax();
  double timeLength = timeEndOriginal - timeBeginOriginal;

  Leph::Plot plot;
  double errorOriginalSum = 0.0;
  double errorOriginalCount = 0.0;
  double errorOriginalMax = -1.0;
  std::string errorOriginalName = "";
  double errorOriginalTime = 0.0;
  double errorFeedSum = 0.0;
  double errorFeedCount = 0.0;
  double errorFeedMax = -1.0;
  double errorFeedTime = 0.0;
  std::string errorFeedName = "";
  double errorCorrectedSum = 0.0;
  double errorCorrectedCount = 0.0;
  double errorCorrectedMax = 0.0;
  double errorCorrectedTime = 0.0;
  std::string errorCorrectedName = "";

  for (double t = 0.0; t <= timeLength; t += 0.01)
  {
    double tOriginal = timeBeginOriginal + t;
    double tFeedForward = timeBeginFeedForward + t;
    double tCorrected = timeBeginCorrected + t;
    for (const std::string& name : Leph::NamesDOFLeg)
    {
      double target = seriesOriginal.get("target:" + name, tOriginal);
      double original = seriesOriginal.get("read:" + name, tOriginal);
      double feedforward = seriesFeedForward.get("read:" + name, tFeedForward);
      double corrected = seriesCorrected.get("read:" + name, tCorrected);
      plot.add({
          "t",
          t,
          "target:" + name,
          180.0 / M_PI * target,
          "original:" + name,
          180.0 / M_PI * original,
          "feedforward:" + name,
          180.0 / M_PI * feedforward,
          "corrected:" + name,
          180.0 / M_PI * corrected,
      });
      double errorOriginal = 180.0 / M_PI * fabs(target - original);
      double errorFeed = 180.0 / M_PI * fabs(target - feedforward);
      double errorCorrected = 180.0 / M_PI * fabs(target - corrected);
      errorOriginalSum += errorOriginal;
      errorFeedSum += errorFeed;
      errorCorrectedSum += errorCorrected;
      errorOriginalCount += 1.0;
      errorFeedCount += 1.0;
      errorCorrectedCount += 1.0;
      if (errorOriginalMax < errorOriginal)
      {
        errorOriginalMax = errorOriginal;
        errorOriginalName = name;
        errorOriginalTime = t;
      }
      if (errorFeedMax < errorFeed)
      {
        errorFeedMax = errorFeed;
        errorFeedName = name;
        errorFeedTime = t;
      }
      if (errorCorrectedMax < errorCorrected)
      {
        errorCorrectedMax = errorCorrected;
        errorCorrectedName = name;
        errorCorrectedTime = t;
      }
    }
  }

  std::cout << "==== Original ====" << std::endl;
  std::cout << "MeanError: " << errorOriginalSum / errorOriginalCount << std::endl;
  std::cout << "MaxError:  " << errorOriginalMax << std::endl;
  std::cout << "MaxName:   " << errorOriginalName << std::endl;
  std::cout << "MaxTime:   " << errorOriginalTime << std::endl;
  std::cout << "==== Feed ====" << std::endl;
  std::cout << "MeanError: " << errorFeedSum / errorFeedCount << std::endl;
  std::cout << "MaxError:  " << errorFeedMax << std::endl;
  std::cout << "MaxName:   " << errorFeedName << std::endl;
  std::cout << "MaxTime:   " << errorFeedTime << std::endl;
  std::cout << "==== Corrected ====" << std::endl;
  std::cout << "MeanError: " << errorCorrectedSum / errorCorrectedCount << std::endl;
  std::cout << "MaxError:  " << errorCorrectedMax << std::endl;
  std::cout << "MaxName:   " << errorCorrectedName << std::endl;
  std::cout << "MaxTime:   " << errorCorrectedTime << std::endl;

  for (const std::string& name : Leph::NamesDOFLeg)
  {
    plot.plot("t", "target:" + name)
        .plot("t", "original:" + name)
        .plot("t", "feedforward:" + name)
        .plot("t", "corrected:" + name)
        .render();
  }

  return 0;
}
