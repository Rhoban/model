/*****************************************************************************/
/*! \file    math_log.cpp
 *  \author  Rhoban Project
 *  \date    2012-10
 *  \brief   math library log
 *****************************************************************************/
#ifndef MATH_LOG_H
#define MATH_LOG_H
#include <string>

#if (__GNUC__ > 2 || (__GNUC__ == 2 && __GNUC_MINOR__ >= 5))
#define NORETURN __attribute__((noreturn))
#else
#define NORETURN
#endif

/****************************************************************************/

class MathLog
{
public:
  static void fatal_error(std::string msg) NORETURN;
  static void check(std::string msg, bool cond);
  static void comment(std::string msg);
};

#endif /* MATH_LOG */
/****************************************************************************/
/****************************************************************************/
