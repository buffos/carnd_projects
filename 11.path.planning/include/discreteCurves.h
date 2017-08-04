#ifndef DISCRETE_CURVES_H
#define DISCRETE_CURVES_H

#include "various_structs.h"
#include "vehicle.h"
#include "road.h"

struct CurveHandler {
  /**
   * Create a Discrete curve. A set of points to follow in XY space
   * @param tr Trajectory: Two polynomials that hold the motion in Frenet coordinates
   * @param r Road: Provides the spline equations that help convert s,d coordinate to x,y
   * @param numberOfPoints: The number of points that need to be generated.
   * @return A new discrete curve Cartesian space
   */
  DiscreteCurve createCurve(Trajectory &tr, const Road & r, int numberOfPoints);
  /**
   * Create a Discrete curve. A set of points to follow in Frenet space
   * @param tr Trajectory: Two polynomials that hold the motion in Frenet coordinates
   * @param numberOfPoints: The number of points that need to be generated.
   * @return A new discrete curve in Frenet space
   */
  DiscreteCurve createCurveFromCoefficientsInFrenet(Trajectory &tr, int numberOfPoints);
  /**
   * Create a Discrete curve. A set of points to follow in XY space.
   * It uses the local "on the fly" create spline coordinate system.
   * It is more general and can be use in all environments.
   *
   * @param tr Trajectory: Two polynomials that hold the motion in Frenet coordinates
   * @param numberOfPoints: The number of points that need to be generated.
   * @param r
   * @return A new discrete curve Cartesian space
   */
  DiscreteCurve createCurveFromCoefficientsInXY(Trajectory &tr, int numberOfPoints, Road &r);
  /**
   * Merge to Discrete curves trying to establish continuity.
   * Unfortunately this is extremely hard to do for first and second order derivatives
   * Continuity can be established partly on first derivates
   * @param newCurve: The newly created curve that is to be merged to the old curve
   * @param oldCurve: The old curve that acts as a merging point for the new curve
   * @return A new discrete curve
   */
  DiscreteCurve mergeCurves(DiscreteCurve &newCurve, DiscreteCurve &oldCurve);
  /**
   * Append the new curve to the old curve. This is used when they curves are
   * designed to be continuous at their endpoints like in our case where the
   * goal points act enforce continuity on all derivatives.
   * @param newCurve: The newly created curve that is to be merged to the old curve
   * @param oldCurve: The old curve that acts as a merging point for the new curve
   * @return A new discrete curve
   */
  DiscreteCurve appendCurve(DiscreteCurve &newCurve, DiscreteCurve &oldCurve);
};

#endif //! DISCRETE_CURVES_H