#ifndef DISCRETE_CURVES_H
#define DISCRETE_CURVES_H

#include "various_structs.h"

struct CurveHandler
{
    DiscreteCurve createCurveFromCoefficientsInFrenet(Trajectory &tr, int numberOfPoints);
    DiscreteCurve createCurveFromCoefficientsInXY(Trajectory &tr, int numberOfPoints, vector<WayPoint> &wp);
    DiscreteCurve mergeCurves(DiscreteCurve &newCurve, DiscreteCurve &oldCurve);
};

#endif //! DISCRETE_CURVES_H