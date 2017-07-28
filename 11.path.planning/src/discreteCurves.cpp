#include "discreteCurves.h"
#include "polynomial.h"
#include "tools.h"

DiscreteCurve CurveHandler::createCurveFromCoefficientsInFrenet(Trajectory &tr, int numberOfPoints)
{
    Polynomial p_1(tr.s_trajectory);
    Polynomial p_2(tr.d_trajectory);

    DiscreteCurve curve;

    double dt = tr.duration / (numberOfPoints - 1);
    for (int i = 1; i <= numberOfPoints; i++)
    {
        curve.c_1.push_back(p_1.evalAt(dt * i, 0));
        curve.c_2.push_back(p_2.evalAt(dt * i, 0));
    }
    curve.coordinateSystem = 2;
    curve.timestep = dt;

    return curve;
}

DiscreteCurve CurveHandler::createCurveFromCoefficientsInXY(Trajectory &tr, int numberOfPoints, vector<WayPoint> &wp)
{
    Polynomial p_1(tr.s_trajectory);
    Polynomial p_2(tr.d_trajectory);

    DiscreteCurve curve;

    double dt = tr.duration / (numberOfPoints - 1);
    for (int i = 1; i <= numberOfPoints; i++)
    {
        double s = p_1.evalAt(dt * i, 0);
        double d = p_2.evalAt(dt * i, 0);

        auto xy = coords::getXY(s, d, wp);

        curve.c_1.push_back(xy[0]);
        curve.c_2.push_back(xy[1]);
    }
    curve.coordinateSystem = 2;
    curve.timestep = dt;

    return curve;
}

DiscreteCurve CurveHandler::mergeCurves(DiscreteCurve &newCurve, DiscreteCurve &oldCurve)
{

    DiscreteCurve mergedCurve;

    if (oldCurve.c_1.size() > newCurve.c_1.size())
    {
        return mergedCurve;
    }

    int mergeSize = oldCurve.c_1.size();
    double eps = 0.001;

    mergedCurve.coordinateSystem = newCurve.coordinateSystem;
    mergedCurve.timestep = newCurve.timestep;

    for (unsigned int i = 0; i < mergeSize; i++)
    {
        // when i == 0 the argument inside logistic will be inf -> logistic -> 1
        // when i == mergeSize the argument inside logistic will be 0 -> logistic -> 0
        // so the contribution of old curve will start as 1 * 50% and end at 0 * 50%
        double oldCurveWeight = 0.50 * logistic(mergeSize / (i + eps) - 1);
        double newCurveWeight = 1 - oldCurveWeight;
        mergedCurve.c_1.push_back(oldCurveWeight * oldCurve.c_1[i] + newCurveWeight * newCurve.c_1[i]);
        mergedCurve.c_2.push_back(oldCurveWeight * oldCurve.c_2[i] + newCurveWeight * newCurve.c_2[i]);
    }

    // now add the rest of the points
    for (unsigned int i = mergeSize; i < newCurve.c_1.size(); i++)
    {
        mergedCurve.c_1.push_back(newCurve.c_1[i]);
        mergedCurve.c_2.push_back(newCurve.c_2[i]);
    }

    return mergedCurve;
}