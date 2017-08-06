#include "discreteCurves.h"
#include "polynomial.h"
#include "tools.h"
#include "trajectoryGenerator.h"

DiscreteCurve CurveHandler::createCurve(Trajectory &tr, const Road &r, int numberOfPoints) {
  Polynomial p_1(tr.s_trajectory);
  Polynomial p_2(tr.d_trajectory);

  DiscreteCurve curve;

  double dt = tr.duration / (numberOfPoints - 1);

  for (int i = 0; i < numberOfPoints; i++) {

    auto xy = r.toXY(p_1.evalAt(dt * i, 0), p_2.evalAt(dt * i, 0));

    curve.c_1.push_back(xy[0]);
    curve.c_2.push_back(xy[1]);
  }
  return curve;
}

DiscreteCurve CurveHandler::createCurveFromCoefficientsInFrenet(Trajectory &tr,
                                                                int numberOfPoints) {
  Polynomial p_1(tr.s_trajectory);
  Polynomial p_2(tr.d_trajectory);

  DiscreteCurve curve;

  double dt = tr.duration / (numberOfPoints - 1);
  for (int i = 0; i < numberOfPoints; i++) {
    curve.c_1.push_back(p_1.evalAt(dt * i, 0));
    curve.c_2.push_back(p_2.evalAt(dt * i, 0));
  }
  curve.coordinateSystem = 2;
  curve.timestep = dt;

  return curve;
}

DiscreteCurve CurveHandler::createCurveFromCoefficientsInXY(Trajectory &tr, int numberOfPoints, Road &r) {
  Polynomial p_1(tr.s_trajectory);
  Polynomial p_2(tr.d_trajectory);

  cout << endl << "The new curve has : " << endl;
  cout << "Plan Duration: " << tr.duration << endl;
  cout << "   New goal s_velocity: " << p_1.evalAt(tr.duration, 1);
  cout << "   New goal s_acceleration: " << p_1.evalAt(tr.duration, 2) << endl;
  cout << "   New goal d_velocity: " << p_2.evalAt(tr.duration, 1);
  cout << "   New goal d_acceleration: " << p_2.evalAt(tr.duration, 2) << endl << endl;

  DiscreteCurve curve, xyCurve;

  curve = std::move(createCurveFromCoefficientsInFrenet(tr, numberOfPoints));
  double s = p_1.evalAt(0.0, 0);  // the start s position for the goal
  double d = p_2.evalAt(0.0, 0);  // the start d position for the goal
  double trackLength = constants::TRACKLENGTH;
  // Splines spline = coords::createLocalSplines(s, 10, 30, r.wpts, trackLength); -> local waypoints

  double dt = tr.duration / (numberOfPoints - 1);
  for (size_t i = 0; i < curve.c_1.size(); i++) {
    // TODO:: SHOULD CHECK IF POINT IS IN SPLINE
    // Local waypoints
    //auto xy = coords::evaluateSplineAtS(curve.c_1[i],
    //	curve.c_2[i],
    //	spline,
    //	trackLength);
    auto xy = r.toXY(curve.c_1[i], curve.c_2[i]);

    xyCurve.c_1.push_back(xy[0]);
    xyCurve.c_2.push_back(xy[1]);
  }
  xyCurve.coordinateSystem = 1;
  xyCurve.timestep = dt;

  return xyCurve;
}

DiscreteCurve CurveHandler::mergeCurves(DiscreteCurve &newCurve, DiscreteCurve &oldCurve) {
  DiscreteCurve mergedCurve;

  size_t mergeLow = 15;
  size_t mergeHigh = mergeLow + 10;

  if (oldCurve.c_1.size() < mergeLow) {
    return newCurve;
  }

  double eps = 0.001;

  mergedCurve.coordinateSystem = newCurve.coordinateSystem;
  mergedCurve.timestep = newCurve.timestep;

  for (unsigned int i = 0; i < newCurve.size(); i++) {
    // when i == 0 the argument inside logistic will be inf -> logistic -> 1
    // when i == mergeSize the argument inside logistic will be 0 -> logistic -> 0
    // so the contribution of old curve will start as 1 * 50% and end at 0 * 50%
    if (i < mergeLow) {
      mergedCurve.c_1.push_back(oldCurve.c_1[i]);
      mergedCurve.c_2.push_back(oldCurve.c_2[i]);
    } else if (i >= mergeLow && i < mergeHigh) {
      //double oldCurveWeight = 1.0 * logistic(mergeSize / (i - mergeLow + eps) - 1);
      //double newCurveWeight = 1 - oldCurveWeight;
      auto oldCurveWeight = static_cast<double>((mergeHigh - i) / (mergeHigh - mergeLow));
      double newCurveWeight = 1 - oldCurveWeight;
      mergedCurve.c_1.push_back(oldCurveWeight * oldCurve.c_1[i] + newCurveWeight * newCurve.c_1[i]);
      mergedCurve.c_2.push_back(oldCurveWeight * oldCurve.c_2[i] + newCurveWeight * newCurve.c_2[i]);
    } else {
      mergedCurve.c_1.push_back(newCurve.c_1[i]);
      mergedCurve.c_2.push_back(newCurve.c_2[i]);
    }
  }

  return mergedCurve;
}

DiscreteCurve CurveHandler::appendCurve(DiscreteCurve &newCurve, DiscreteCurve &oldCurve) {

  DiscreteCurve mergedCurve;

  for (size_t i = 0; i < oldCurve.size(); i++) {
    mergedCurve.c_1.push_back(oldCurve.c_1[i]);
    mergedCurve.c_2.push_back(oldCurve.c_2[i]);
  }

  for (size_t i = 1; i < newCurve.size(); i++) {
    mergedCurve.c_1.push_back(newCurve.c_1[i]);
    mergedCurve.c_2.push_back(newCurve.c_2[i]);
  }
  return mergedCurve;
}