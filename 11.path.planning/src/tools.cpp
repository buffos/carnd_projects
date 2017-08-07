/**
 * @file tools.cpp
 * @brief Implementation of helper functions contained in coords namespace
 *
 *
 * @author  Kostas Oreopoulos
 */
#include "tools.h"

/// Logistic fucntions. maps [0 inf] to [0 1]
double logistic(double x) {
  // A function that returns a value between 0 and 1 for x in the range [0, infinity]
  //  and -1 to 1 for x in the range [-infinity, infinity].
  return 2.0 / (1 + exp(-x)) - 1.0;
}

/// Calc classical Euclidean distance
double coords::distance(const double x1, const double y1,
                        const double x2, const double y2) {
  return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
}

/// Find the closest waypoint to a given position
int coords::ClosestWaypoint(const double x, const double y,
                            const vector<WayPoint> &wp) {

  double closestLen = 100000; //large number
  int closestWaypoint = 0;

  for (unsigned int i = 0; i < wp.size(); i++) {
    double map_x = wp[i].x;
    double map_y = wp[i].y;
    double dist = coords::distance(x, y, map_x, map_y);
    if (dist < closestLen) {
      closestLen = dist;
      closestWaypoint = i;
    }
  }

  return closestWaypoint;
}

/// Find the next waypoint in the moving direction of the car.
int coords::NextWaypoint(const double x, const double y, const double theta,
                         const vector<WayPoint> &wp) {

  int closestWaypoint = coords::ClosestWaypoint(x, y, wp);

  double map_x = wp[closestWaypoint].x;
  double map_y = wp[closestWaypoint].y;

  double heading = atan2((map_y - y), (map_x - x));

  double angle = abs(theta - heading);

  if (angle > pi() / 4) {
    closestWaypoint++;
  }

  return closestWaypoint;
}

/// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> coords::getFrenet(const double x, const double y,
                                 const double theta,
                                 const vector<WayPoint> &wp) {
  int next_wp = coords::NextWaypoint(x, y, theta, wp);

  int prev_wp;
  prev_wp = next_wp - 1;
  if (next_wp == 0) {
    prev_wp = wp.size() - 1;
  }

  double n_x = wp[next_wp].x - wp[prev_wp].x;
  double n_y = wp[next_wp].y - wp[prev_wp].y;
  double x_x = x - wp[prev_wp].x;
  double x_y = y - wp[prev_wp].y;

  // find the projection of x onto n
  double proj_norm = (x_x * n_x + x_y * n_y) / (n_x * n_x + n_y * n_y);
  double proj_x = proj_norm * n_x;
  double proj_y = proj_norm * n_y;

  double frenet_d = distance(x_x, x_y, proj_x, proj_y);

  //see if d value is positive or negative by comparing it to a center point

  double center_x = 1000 - wp[prev_wp].x;
  double center_y = 2000 - wp[prev_wp].y;
  double centerToPos = distance(center_x, center_y, x_x, x_y);
  double centerToRef = distance(center_x, center_y, proj_x, proj_y);

  if (centerToPos <= centerToRef) {
    frenet_d *= -1;
  }

  // calculate s value
  double frenet_s = 0;
  for (int i = 0; i < prev_wp; i++) {
    frenet_s += distance(wp[i].x, wp[i].y, wp[i + 1].x, wp[i + 1].y);
  }

  frenet_s += distance(0, 0, proj_x, proj_y);

  return {frenet_s, frenet_d};
}

/// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> coords::getXY(const double s, const double d,
                             const vector<WayPoint> &wp) {
  int prev_wp = -1;

  while ((prev_wp < (int) (wp.size() - 1)) && s > wp[prev_wp + 1].s) {
    prev_wp++;
  }

  int wp2 = (prev_wp + 1) % wp.size();

  double
      heading = atan2((wp[wp2].y - wp[prev_wp].y), (wp[wp2].x - wp[prev_wp].x));
  // the x,y,s along the segment
  double seg_s = (s - wp[prev_wp].s);

  double seg_x = wp[prev_wp].x + seg_s * cos(heading);
  double seg_y = wp[prev_wp].y + seg_s * sin(heading);

  double perp_heading = heading - pi() / 2;

  double x = seg_x + d * cos(perp_heading);
  double y = seg_y + d * sin(perp_heading);

  return {x, y};
}

/// Create a vector of indexes of Waypoint around the current position
vector<int> coords::getLocalWayPointIndexes(const int index, const int back,
                                            const int front,
                                            const int wp_size) {
  // take K waypoints before and M ahead
  int k = back;
  int m = front;
  int prev_wp = index;
  vector<int> wp_indexes;
  // i need k-1 more previous wp
  for (int i = prev_wp - k + 1; i <= prev_wp; i++) {
    if (i < 0) {
      wp_indexes.push_back(i + wp_size); // at the beginning of the loop
    } else {
      wp_indexes.push_back(i);
    }
  }
  // now add m ahead
  for (int i = prev_wp + 1; i <= prev_wp + m; i++) {
    if (i >= wp_size) {
      wp_indexes.push_back(i - wp_size); // at the end of the loop
    } else {
      wp_indexes.push_back(i);
    }
  }
  return wp_indexes;
}

///  Create around global Frenet Coordinate s a Spline
Splines coords::createLocalSplines(const double s, const int back,
                                   const int front,
                                   const vector<WayPoint> &wp,
                                   const double trackLength) {
  int prev_wp = -1;

  while ((prev_wp < (int) (wp.size() - 1)) && s > wp[prev_wp + 1].s) {
    prev_wp++;
  }

  // if current_wp = 0 then prev_wp = -1 == the last in the list
  prev_wp = (prev_wp == -1) ? prev_wp + (int) wp.size() : prev_wp;

  auto indexes = getLocalWayPointIndexes(prev_wp, back, front, wp.size());

  vector<double> x;
  vector<double> y;
  vector<double> ss;
  vector<double> dx;
  vector<double> dy;

  for (auto index : indexes) {
    x.push_back(wp[index].x);
    y.push_back(wp[index].y);
    ss.push_back(wp[index].s);
    dx.push_back(wp[index].dx);
    dy.push_back(wp[index].dy);
  }

  // fix s values for looping case
  // now i store the values of s that the spline struct can facilitate
  int first_wp = indexes[0];
  int last_wp = indexes[indexes.size() - 1];

  // trackLength = 6914.14925765991;
  if (wp[first_wp].s > wp[last_wp].s) {
    // add max_s to all variables less than
    // last element (which is the left most element)
    for (auto &element : ss) {
      if (element < wp[first_wp].s) {
        element += trackLength;
      }
    }
  }

  Splines sp;
  sp.x.set_points(ss, x);
  sp.y.set_points(ss, y);
  sp.dx.set_points(ss, dx);
  sp.dy.set_points(ss, dy);

  sp.start_s = wp[first_wp].s;
  sp.end_s = wp[last_wp].s;
  return sp;
}

/// Evaluate a Spline Curve around {s,d} and get {x,y} coordinates
vector<double> coords::evaluateSplineAtS(const double s,
                                         const double d,
                                         const Splines &sp,
                                         const double trackLength) {
  double x = constants::OUT_OF_BOUNDS;
  double y = constants::OUT_OF_BOUNDS;
  vector<double> result = {x, y};

  if (sp.start_s < sp.end_s)
    // this is a normal spline not at the end of the track
  {
    if (s >= sp.start_s && s <= sp.end_s) {
      // in the range
      x = sp.x(s) + sp.dx(s) * d;
      // the contribution from S and the contribution from d
      y = sp.y(s) + sp.dy(s) * d;
      result = {x, y};
    }
  } else // we are at the loop of the track. I
  {
    if (s > sp.start_s) {
      // since this is at the loop point if s > sp.start_s then its definitely in the range
      // and there is also no need to add the extra max_s
      x = sp.x(s) + sp.dx(s) * d;
      // the contribution from S and the contribution from d
      y = sp.y(s) + sp.dy(s) * d;
      result = {x, y};
    } else if (s < sp.start_s && s < sp.end_s) {
      // this means that its also in the range but I must add max_s to retrieve the x, y values
      x = sp.x(s + trackLength) + sp.dx(s + trackLength) * d;
      // the contribution from S and the contribution from d
      y = sp.y(s + trackLength) + sp.dy(s + trackLength) * d;
      result = {x, y};
    }
  }
  return result;
}

/// Check if a point with s coordinate is in the range covered by the Spline Curve
bool coords::isPointInSpline(double s, const Splines &sp) {
  bool result = false;
  if (sp.start_s
      < sp.end_s) // this is a normal spline not at the end of the track
  {
    if (s >= sp.start_s
        && s <= sp.end_s) {                                // in the range
      result = true;
    }
  } else // we are at the loop of the track. I
  {
    if (s > sp.start_s) {
      // since this is at the loop point if s > sp.start_s then its definitely in the range
      // and there is also no need to add the extra max_s
      result = true;
    } else if (s < sp.start_s && s < sp.end_s) {
      // this means that its also in the range but I must add max_s to retrieve the x, y values
      result = true;
    }
  }
  return result;
}

/// Calc acceleration from points on curve
double coords::accelerationFromCurve(const DiscreteCurve &curve,
                                     const int fromPoint,
                                     int toPoint) {
  if (curve.c_1.size() < 3) {
    return 0.0; // no valid curve
  }

  unsigned int mergeSize = curve.c_1.size();
  toPoint = (toPoint >= mergeSize) ? (int) (mergeSize - 1) : toPoint;
  double maxAcceleration = 0.0;
  double dt = 1. / constants::FRAMES_PER_SEC;

  for (int i = fromPoint; i <= toPoint; i++) {
    double speed_x = (curve.c_1[fromPoint + 1] - curve.c_1[fromPoint]) / dt;
    double speed_y = (curve.c_2[fromPoint + 1] - curve.c_2[fromPoint]) / dt;
    double speed_1 = sqrt(speed_x * speed_x + speed_y * speed_y);
    speed_x = (curve.c_1[fromPoint + 2] - curve.c_1[fromPoint + 1]) / dt;
    speed_y = (curve.c_2[fromPoint + 2] - curve.c_2[fromPoint + 1]) / dt;
    double speed_2 = sqrt(speed_x * speed_x + speed_y * speed_y);
    double acceleration = (speed_2 - speed_1) / dt;
    maxAcceleration =
        (abs(acceleration) > maxAcceleration) ? acceleration : maxAcceleration;
  }

  return maxAcceleration;
}