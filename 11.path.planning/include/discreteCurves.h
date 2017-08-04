#ifndef DISCRETE_CURVES_H
#define DISCRETE_CURVES_H

#include "various_structs.h"
#include "vehicle.h"
#include "road.h"

struct CurveHandler {
  DiscreteCurve createCurve(Trajectory &tr, const Road & r, int numberOfPoints);
  DiscreteCurve createCurveFromCoefficientsInFrenet(Trajectory &tr, int numberOfPoints);
  DiscreteCurve createCurveFromCoefficientsInXY(Trajectory &tr, int numberOfPoints, Road &r);
  DiscreteCurve mergeCurves(DiscreteCurve &newCurve, DiscreteCurve &oldCurve);
  DiscreteCurve mergeCurves2(DiscreteCurve &newCurve, DiscreteCurve &oldCurve);
  DiscreteCurve appendCurve(DiscreteCurve &newCurve, DiscreteCurve &oldCurve);
};

#endif //! DISCRETE_CURVES_H