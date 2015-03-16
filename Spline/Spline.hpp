#ifndef LEPH_SPLINE_HPP
#define LEPH_SPLINE_HPP

#include <vector>
#include "Spline/Polynom.hpp"

namespace Leph {

/**
 * Spline
 *
 * Generic one dimentional 
 * polynomial spline generator
 */
class Spline 
{
    public:

        /**
         * Return spline interpolation
         * at given t. Compute spline value,
         * its first and second derivative
         */
        double pos(double t) const;
        double vel(double t) const;
        double acc(double t) const;

        /**
         * Return spline interpolation
         * value, first and second derivative
         * with given t bound betewwen 0 and 1
         */
        double posMod(double t) const;
        double velMod(double t) const;
        double accMod(double t) const;

    protected:

        /**
         * Internal spline part structure
         * with a polynom valid on an interval
         * If isNormalization, polynom evaluation
         * is scale from [min:max] to [0:max-min]
         */
        struct Spline_t {
            Polynom polynom;
            double min;
            double max;
            bool isNormalization;
        };

        /**
         * Spline part container
         */
        std::vector<Spline_t> _splines;
        
        /**
         * Return spline interpolation of given value and
         * used given polynom evaluation function
         * (member function pointer)
         */
        double interpolation(double x, 
            double(Polynom::*func)(double) const) const;

        /**
         * Return interpolation with x 
         * bound between 0 and 1
         */
        double interpolationMod(double x, 
            double(Polynom::*func)(double) const) const;
};

}

#endif

