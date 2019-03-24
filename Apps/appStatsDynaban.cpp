#include <iostream>
#include <vector>
#include <string>
#include <map>
#include <fstream>
#include <cmath>
#include "Types/MatrixLabel.hpp"
#include "Utils/AxisAngle.h"
#include "Model/HumanoidFixedModel.hpp"

struct Point
{
  double time;
  double goal;
  double pos;
};

double getValue(double t, std::vector<Point> data, std::string elt)
{
  for (size_t i = 0; i < data.size() - 1; i++)
  {
    double t1 = data[i].time;
    double t2 = data[i + 1].time;
    double v1;
    double v2;
    if (elt == "pos")
    {
      v1 = data[i].pos;
      v2 = data[i + 1].pos;
    }
    else if (elt == "goal")
    {
      v1 = data[i].goal;
      v2 = data[i + 1].goal;
    }
    else
    {
      std::cout << "ERROR ELT" << std::endl;
      exit(1);
    }
    if (t1 <= t && t2 >= t)
    {
      return v1 + (v2 - v1) / (t2 - t1) * (t - t1);
    }
  }
  if (t <= 0.2 && elt == "pos")
  {
    return data.front().pos;
  }
  else if (t > 2.8 && elt == "pos")
  {
    return data.back().pos;
  }
  if (t <= 0.2 && elt == "goal")
  {
    return data.front().goal;
  }
  else if (t > 2.8 && elt == "goal")
  {
    return data.back().goal;
  }
  return 0.0;
}

Leph::MatrixLabel interpolate(std::map<std::string, std::vector<Point>>& container, double step, double delta)
{
  Leph::MatrixLabel matrix;
  for (double t = 0.0; t <= 3.0; t += step)
  {
    matrix.append(Leph::VectorLabel());
  }

  size_t index = 0;
  for (double t = 0.0; t <= 3.0; t += step)
  {
    matrix[index].append("t", t);
    for (auto& it : container)
    {
      matrix[index].append("pos:" + it.first.substr(4), getValue(t + delta, it.second, "pos"));
      matrix[index].append("goal:" + it.first.substr(4), getValue(t, it.second, "goal"));
    }
    index++;
  }

  return matrix;
}

void computeModel(Leph::MatrixLabel& logs)
{
  Leph::HumanoidFixedModel modelGoal(Leph::SigmabanModel);
  Leph::HumanoidFixedModel modelPos(Leph::SigmabanModel);
  for (size_t i = 0; i < logs.size(); i++)
  {
    // Assign DOF
    Leph::VectorLabel vectDOFGoal = logs[i].extract("goal").rename("goal", "");
    modelGoal.get().setDOF(vectDOFGoal, false);
    Leph::VectorLabel vectDOFPos = logs[i].extract("pos").rename("pos", "");
    modelPos.get().setDOF(vectDOFPos, false);
    // Contraint the model on the ground
    modelGoal.updateBase();
    modelPos.updateBase();
    // Compute model goal
    std::string supportGoalName = "left";
    std::string supportPosName = "left";
    logs[i].append("goal_model:trunk_pos_x", modelGoal.get().position("trunk", supportGoalName + "_foot_tip").x());
    logs[i].append("goal_model:trunk_pos_y", modelGoal.get().position("trunk", supportGoalName + "_foot_tip").y());
    logs[i].append("goal_model:trunk_pos_z", modelGoal.get().position("trunk", supportGoalName + "_foot_tip").z());
    logs[i].append("goal_model:foot_pos_x",
                   modelGoal.get().position("right_foot_tip", supportGoalName + "_foot_tip").x());
    logs[i].append("goal_model:foot_pos_y",
                   modelGoal.get().position("right_foot_tip", supportGoalName + "_foot_tip").y());
    logs[i].append("goal_model:foot_pos_z",
                   modelGoal.get().position("right_foot_tip", supportGoalName + "_foot_tip").z());
    logs[i].append(
        "goal_model:trunk_axis_x",
        Leph::MatrixToAxis(modelGoal.get().orientation("trunk", supportGoalName + "_foot_tip").transpose()).x());
    logs[i].append(
        "goal_model:trunk_axis_y",
        Leph::MatrixToAxis(modelGoal.get().orientation("trunk", supportGoalName + "_foot_tip").transpose()).y());
    logs[i].append(
        "goal_model:trunk_axis_z",
        Leph::MatrixToAxis(modelGoal.get().orientation("trunk", supportGoalName + "_foot_tip").transpose()).z());
    logs[i].append(
        "goal_model:foot_axis_x",
        Leph::MatrixToAxis(modelGoal.get().orientation("right_foot_tip", supportGoalName + "_foot_tip").transpose())
            .x());
    logs[i].append(
        "goal_model:foot_axis_y",
        Leph::MatrixToAxis(modelGoal.get().orientation("right_foot_tip", supportGoalName + "_foot_tip").transpose())
            .y());
    logs[i].append(
        "goal_model:foot_axis_z",
        Leph::MatrixToAxis(modelGoal.get().orientation("right_foot_tip", supportGoalName + "_foot_tip").transpose())
            .z());
    // Compute model pos
    logs[i].append("model:trunk_pos_x", modelPos.get().position("trunk", supportPosName + "_foot_tip").x());
    logs[i].append("model:trunk_pos_y", modelPos.get().position("trunk", supportPosName + "_foot_tip").y());
    logs[i].append("model:trunk_pos_z", modelPos.get().position("trunk", supportPosName + "_foot_tip").z());
    logs[i].append("model:foot_pos_x", modelPos.get().position("right_foot_tip", supportPosName + "_foot_tip").x());
    logs[i].append("model:foot_pos_y", modelPos.get().position("right_foot_tip", supportPosName + "_foot_tip").y());
    logs[i].append("model:foot_pos_z", modelPos.get().position("right_foot_tip", supportPosName + "_foot_tip").z());
    logs[i].append(
        "model:trunk_axis_x",
        Leph::MatrixToAxis(modelPos.get().orientation("trunk", supportPosName + "_foot_tip").transpose()).x());
    logs[i].append(
        "model:trunk_axis_y",
        Leph::MatrixToAxis(modelPos.get().orientation("trunk", supportPosName + "_foot_tip").transpose()).y());
    logs[i].append(
        "model:trunk_axis_z",
        Leph::MatrixToAxis(modelPos.get().orientation("trunk", supportPosName + "_foot_tip").transpose()).z());
    logs[i].append(
        "model:foot_axis_x",
        Leph::MatrixToAxis(modelPos.get().orientation("right_foot_tip", supportPosName + "_foot_tip").transpose()).x());
    logs[i].append(
        "model:foot_axis_y",
        Leph::MatrixToAxis(modelPos.get().orientation("right_foot_tip", supportPosName + "_foot_tip").transpose()).y());
    logs[i].append(
        "model:foot_axis_z",
        Leph::MatrixToAxis(modelPos.get().orientation("right_foot_tip", supportPosName + "_foot_tip").transpose()).z());
    // Compute model error
    logs[i].append("error_model:trunk_pos_x", fabs(logs[i]("goal_model:trunk_pos_x") - logs[i]("model:trunk_pos_x")));
    logs[i].append("error_model:trunk_pos_y", fabs(logs[i]("goal_model:trunk_pos_y") - logs[i]("model:trunk_pos_y")));
    logs[i].append("error_model:trunk_pos_z", fabs(logs[i]("goal_model:trunk_pos_z") - logs[i]("model:trunk_pos_z")));
    logs[i].append("error_model:foot_pos_x", fabs(logs[i]("goal_model:foot_pos_x") - logs[i]("model:foot_pos_x")));
    logs[i].append("error_model:foot_pos_y", fabs(logs[i]("goal_model:foot_pos_y") - logs[i]("model:foot_pos_y")));
    logs[i].append("error_model:foot_pos_z", fabs(logs[i]("goal_model:foot_pos_z") - logs[i]("model:foot_pos_z")));
    logs[i].append("error_model:trunk_axis_x", fabs(logs[i]("goal_model:trunk_axis_x") - logs[i]("model:trunk_axis_"
                                                                                                 "x")));
    logs[i].append("error_model:trunk_axis_y", fabs(logs[i]("goal_model:trunk_axis_y") - logs[i]("model:trunk_axis_"
                                                                                                 "y")));
    logs[i].append("error_model:trunk_axis_z", fabs(logs[i]("goal_model:trunk_axis_z") - logs[i]("model:trunk_axis_"
                                                                                                 "z")));
    logs[i].append("error_model:foot_axis_x", fabs(logs[i]("goal_model:foot_axis_x") - logs[i]("model:foot_axis_x")));
    logs[i].append("error_model:foot_axis_y", fabs(logs[i]("goal_model:foot_axis_y") - logs[i]("model:foot_axis_y")));
    logs[i].append("error_model:foot_axis_z", fabs(logs[i]("goal_model:foot_axis_z") - logs[i]("model:foot_axis_z")));
  }
}

int main(int argc, char** argv)
{
  std::map<size_t, std::map<size_t, std::map<std::string, std::vector<Point>>>> container;
  size_t experiment = 0;
  size_t index = 0;
  for (size_t i = 1; i < (size_t)argc; i++)
  {
    std::string filename = argv[i];
    std::cout << filename << std::endl;
    if (filename == "NEXT")
    {
      experiment++;
      index = 0;
      continue;
    }
    std::ifstream file(filename);
    if (!file.is_open())
    {
      return 0;
    }
    std::string content = "";
    while (file.good())
    {
      int c = file.peek();
      if (c == EOF)
      {
        continue;
      }
      content += c;
      file.ignore();
    }
    file.close();

    size_t pos1 = 0;
    std::string name = "";
    while (true)
    {
      if (pos1 >= content.size() - 1)
      {
        break;
      }
      std::string line = "";
      while (pos1 < content.length() && content[pos1] != '\n')
      {
        line += content[pos1];
        pos1++;
      }
      pos1++;
      if (line.length() == 0 || line[0] == '\n' || line[0] == ' ')
      {
        continue;
      }
      if (line[0] == '#')
      {
        name = line.substr(1);
        continue;
      }
      else
      {
        std::string num1 = "";
        std::string num2 = "";
        std::string num3 = "";
        size_t pos2 = 0;
        size_t pos3 = 0;
        while (pos3 < line.length() && line[pos3] != ' ')
          pos3++;
        pos3++;
        num1 = line.substr(pos2, pos3 - pos2 - 1);
        pos2 = pos3;
        while (pos3 < line.length() && line[pos3] != ' ')
          pos3++;
        pos3++;
        num2 = line.substr(pos2, pos3 - pos2 - 1);
        pos2 = pos3;
        while (pos3 < line.length() && line[pos3] != ' ')
          pos3++;
        pos3++;
        num3 = line.substr(pos2, pos3 - pos2 - 1);
        pos2 = pos3;
        double n1 = std::stod(num1);
        double n2 = std::stod(num2);
        double n3 = std::stod(num3);
        if (n3 > M_PI)
          n3 -= 2.0 * M_PI;
        if (n3 < -M_PI)
          n3 += 2.0 * M_PI;
        container[experiment][index][name].push_back({ n1, n2, n3 });
      }
    }
    index++;
  }

  for (double delta = -0.2; delta <= 0.2; delta += 0.005)
  {
    delta = 0.0;
    // delta = 0.08;

    std::map<size_t, std::map<size_t, Leph::MatrixLabel>> container2;
    for (auto& it : container)
    {
      for (auto& it2 : it.second)
      {
        container2[it.first].insert({ it2.first, interpolate(it2.second, 0.01, delta) });
        computeModel(container2[it.first][it2.first]);
        // std::cout << "Compute model: " << it.first << " " << it2.first << std::endl;
      }
    }

    std::map<size_t, Leph::MatrixLabel> containerMean;
    std::map<size_t, Leph::MatrixLabel> containerVar;
    for (auto& it : container2)
    {
      containerMean.insert({ it.first, container2[0][0].copy() });
      containerVar.insert({ it.first, container2[0][0].copy() });
    }

    Leph::Plot plot;
    for (size_t i = 0; i < container2[0][0].size(); i++)
    {
      double t = container2[0][0][i]("t");
      std::cout << t << " ";
      for (auto& it : container2)
      {
        Leph::VectorLabel sum;
        sum.mergeUnion(container2[0][0][0]);
        Leph::VectorLabel sum2;
        sum2.mergeUnion(container2[0][0][0]);
        sum.zeroOp();
        sum2.zeroOp();
        double count = 0.0;
        for (auto& it2 : it.second)
        {
          Leph::VectorLabel tmp = it2.second[i];
          sum.addOp(tmp);
          tmp.squareOp();
          sum2.addOp(tmp);
          count++;
        }
        sum.divOp(count);
        sum2.divOp(count);
        containerMean[it.first][i].assignOp(sum);
        sum.squareOp();
        sum2.subOp(sum);
        sum2.minOp(0.0);
        sum2.sqrtOp();
        containerVar[it.first][i].assignOp(sum2);
        /*
        Leph::VectorLabel error = containerMean[it.first][i].extract("model");
        error.subOp(containerMean[it.first][i].extract("goal"), "goal", "model");
        error.rename("model", "error");
        error.squareOp();
        std::cout << error << std::endl;
        std::cout << containerMean[it.first][i] << std::endl;
        containerMean[it.first][i].mergeUnion(error);
        */

        std::cout << containerMean[it.first][i]("goal_model:trunk_axis_x") << " ";
        std::cout << containerMean[it.first][i]("model:trunk_axis_x") << " ";
        std::cout << containerMean[it.first][i]("goal_model:foot_pos_x") << " ";
        std::cout << containerMean[it.first][i]("model:foot_pos_x") << " ";
        Leph::VectorLabel vect("t", t);
        Leph::VectorLabel tmpp = containerMean[it.first][i];
        tmpp = tmpp.rename("model", std::to_string(it.first) + "_mean_model");
        tmpp = tmpp.rename("goal_model", std::to_string(it.first) + "_mean_goal_model");
        vect.mergeUnion(tmpp);
        plot.add(vect);
      }
      std::cout << std::endl;
    }
    // return 0;

    plot.plot("t", "0_mean_model:trunk_axis_x")
        .plot("t", "1_mean_model:trunk_axis_x")
        .plot("t", "0_mean_goal_model:trunk_axis_x")
        .render();

    plot.plot("t", "0_mean_model:foot_pos_x")
        .plot("t", "1_mean_model:foot_pos_x")
        .plot("t", "0_mean_goal_model:foot_pos_x")
        .render();
    // return 0.0;

    Leph::VectorLabel mean0All = containerMean[0].mean();
    Leph::VectorLabel mean1All = containerMean[1].mean();
    Leph::VectorLabel var0All = containerMean[0].stdDev();
    Leph::VectorLabel var1All = containerMean[1].stdDev();
    mean0All.sqrtOp();
    mean1All.sqrtOp();
    double RMSE0Pos = 0.0;
    double RMSE0Axis = 0.0;
    double RMSE1Pos = 0.0;
    double RMSE1Axis = 0.0;
    RMSE0Pos += mean0All("error_model:trunk_pos_x");
    RMSE0Pos += mean0All("error_model:trunk_pos_y");
    RMSE0Pos += mean0All("error_model:trunk_pos_z");
    RMSE0Axis += mean0All("error_model:trunk_axis_x");
    RMSE0Axis += mean0All("error_model:trunk_axis_y");
    RMSE0Axis += mean0All("error_model:trunk_axis_z");
    RMSE0Pos += mean0All("error_model:foot_pos_x");
    RMSE0Pos += mean0All("error_model:foot_pos_y");
    RMSE0Pos += mean0All("error_model:foot_pos_z");
    RMSE0Axis += mean0All("error_model:foot_axis_x");
    RMSE0Axis += mean0All("error_model:foot_axis_y");
    RMSE0Axis += mean0All("error_model:foot_axis_z");

    RMSE1Pos += mean1All("error_model:trunk_pos_x");
    RMSE1Pos += mean1All("error_model:trunk_pos_y");
    RMSE1Pos += mean1All("error_model:trunk_pos_z");
    RMSE1Axis += mean1All("error_model:trunk_axis_x");
    RMSE1Axis += mean1All("error_model:trunk_axis_y");
    RMSE1Axis += mean1All("error_model:trunk_axis_z");
    RMSE1Pos += mean1All("error_model:foot_pos_x");
    RMSE1Pos += mean1All("error_model:foot_pos_y");
    RMSE1Pos += mean1All("error_model:foot_pos_z");
    RMSE1Axis += mean1All("error_model:foot_axis_x");
    RMSE1Axis += mean1All("error_model:foot_axis_y");
    RMSE1Axis += mean1All("error_model:foot_axis_z");

    std::cout << std::endl;
    std::cout << std::endl;
    std::cout << std::endl;
    std::vector<std::string> names = {
      "trunk_pos_x", "trunk_pos_y", "trunk_pos_z", "trunk_axis_x", "trunk_axis_y", "trunk_axis_z",
      "foot_pos_x",  "foot_pos_y",  "foot_pos_z",  "foot_axis_x",  "foot_axis_y",  "foot_axis_z",
    };
    for (size_t i = 0; i < names.size(); i++)
    {
      std::cout << "'" << names[i] << "' " << mean0All("error_model:" + names[i]) << " "
                << var0All("error_model:" + names[i]) << " " << mean1All("error_model:" + names[i]) << " "
                << var1All("error_model:" + names[i]) << std::endl;
    }
    return 0;

    // std::cout << "Delta: " << delta << " --> " << RMSE0Pos << "//" << RMSE0Axis << " Vs. " << RMSE1Pos << "//" <<
    // RMSE1Axis << std::endl;
  }

  /*
  std::cout << containerMean[0].mean().extract("error_model") <<std::endl;
  std::cout << containerMean[1].mean().extract("error_model") <<std::endl;
  plot
      .plot("t", "0_mean_model:trunk_pos_x")
      .plot("t", "0_mean_model:trunk_pos_y")
      .plot("t", "0_mean_model:trunk_pos_z")
      .plot("t", "1_mean_model:trunk_pos_x")
      .plot("t", "1_mean_model:trunk_pos_y")
      .plot("t", "1_mean_model:trunk_pos_z")
      .plot("t", "0_mean_goal_model:trunk_pos_x")
      .plot("t", "0_mean_goal_model:trunk_pos_y")
      .plot("t", "0_mean_goal_model:trunk_pos_z")
      .render();


  containerMean[1].plot()
      .plot("t", "model:trunk_axis_x")
      .plot("t", "model:trunk_axis_y")
      .plot("t", "model:trunk_axis_z")
      .plot("t", "goal_model:trunk_axis_x")
      .plot("t", "goal_model:trunk_axis_y")
      .plot("t", "goal_model:trunk_axis_z")
      .render();
  */

  return 0;
}
