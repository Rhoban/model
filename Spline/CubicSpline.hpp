#ifndef LEPH_CUBICSPLINE_HPP
#define LEPH_CUBICSPLINE_HPP

#include "Spline/Spline.hpp"

namespace Leph {

/**
 * CubicSpline
 *
 * Implementation of 3th order 
 * polynomial splines 
 */
class CubicSpline : public Spline
{
    public:

        /**
         * Add a new point with its time, position value,
         * and velocity
         */
        void addPoint(double time, double position, 
            double velocity = 0.0);
        
        /**
         * Apply normal random noise on spline point 
         * position and velocity of given standart deviation.
         * If updateBounds is true, extremum point (min and max time)
         * are also updated.
         */
        void randomNoise(
            double stdDevPos, double stdDevVel, bool updateBounds);

    protected:

        /**
         * Inherit
         * Load Points
         */
        virtual void importCallBack() override;
        
    private:

        /**
         * Simple point struture
         */
        struct Point {
            double time;
            double position;
            double velocity;
        };

        /**
         * Points container
         */
        std::vector<Point> _points;
        
        /**
         * Fit a polynom between 0 and t with given
         * pos, vel and acc initial and final conditions
         */
        Polynom polynomFit(double t, 
            double pos1, double vel1,
            double pos2, double vel2) const;
        
        /**
         * Recompute splines interpolation model
         */
        void computeSplines();
};

}

#endif

