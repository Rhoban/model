/*****************************************************************************/
/*! \file    math_log.cpp
 *  \author  Rhoban Project
 *  \date    2012-10
 *  \brief   math library log
 *****************************************************************************/
#include <stdlib.h>
#include <stdio.h>
#include <stdexcept>
#include <iostream>
#include <sstream>
//#include <string>
#include "math_log.h"
using namespace std;

void MathLog::fatal_error(string msg)
{
  stringstream reason;
  reason << "FATAL ERROR : " << msg;
  cerr << reason.str();
  cout << reason.str();
  throw std::runtime_error(reason.str());
}

void MathLog::check(string msg, bool cond)
{
  if (!cond)
  {
    stringstream reason;
    reason << "ASSERTION FAILER : " << msg;
    cerr << reason.str();
    throw std::runtime_error(reason.str());
  }
}

void MathLog::comment(string msg)
{
  cout << "LOG: " << msg << endl;
  cout.flush();
}

/*****************************************************************************/
/*****************************************************************************/
