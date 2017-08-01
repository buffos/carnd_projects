
#include "tools.h"

double logistic(double x)
{
	// A function that returns a value between 0 and 1 for x in the range [0, infinity]
	//  and -1 to 1 for x in the range [-infinity, infinity].
	return 2.0 / (1 + exp(-x)) - 1.0;
}

double coords::distance(double x1, double y1, double x2, double y2)
{
	return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
}

int coords::ClosestWaypoint(double x, double y, vector<WayPoint> &wp)
{

	double closestLen = 100000; //large number
	int closestWaypoint = 0;

	for (unsigned int i = 0; i < wp.size(); i++)
	{
		double map_x = wp[i].x;
		double map_y = wp[i].y;
		double dist = coords::distance(x, y, map_x, map_y);
		if (dist < closestLen)
		{
			closestLen = dist;
			closestWaypoint = i;
		}
	}

	return closestWaypoint;
}

int coords::NextWaypoint(double x, double y, double theta, vector<WayPoint> &wp)
{

	int closestWaypoint = coords::ClosestWaypoint(x, y, wp);

	double map_x = wp[closestWaypoint].x;
	double map_y = wp[closestWaypoint].y;

	double heading = atan2((map_y - y), (map_x - x));

	double angle = abs(theta - heading);

	if (angle > pi() / 4)
	{
		closestWaypoint++;
	}

	return closestWaypoint;
}

/// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> coords::getFrenet(double x, double y, double theta, vector<WayPoint> &wp)
{
	int next_wp = coords::NextWaypoint(x, y, theta, wp);

	int prev_wp;
	prev_wp = next_wp - 1;
	if (next_wp == 0)
	{
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

	if (centerToPos <= centerToRef)
	{
		frenet_d *= -1;
	}

	// calculate s value
	double frenet_s = 0;
	for (int i = 0; i < prev_wp; i++)
	{
		frenet_s += distance(wp[i].x, wp[i].y, wp[i + 1].x, wp[i + 1].y);
	}

	frenet_s += distance(0, 0, proj_x, proj_y);

	return { frenet_s, frenet_d };
}

/// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> coords::getXY(double s, double d, vector<WayPoint> &wp)
{
	int prev_wp = -1;

	while ((prev_wp < (int)(wp.size() - 1)) && s > wp[prev_wp + 1].s)
	{
		prev_wp++;
	}

	int wp2 = (prev_wp + 1) % wp.size();

	double heading = atan2((wp[wp2].y - wp[prev_wp].y), (wp[wp2].x - wp[prev_wp].x));
	// the x,y,s along the segment
	double seg_s = (s - wp[prev_wp].s);

	double seg_x = wp[prev_wp].x + seg_s * cos(heading);
	double seg_y = wp[prev_wp].y + seg_s * sin(heading);

	double perp_heading = heading - pi() / 2;

	double x = seg_x + d * cos(perp_heading);
	double y = seg_y + d * sin(perp_heading);

	return { x, y };
}

/// The real distance between two cars , taking into account the track loop
vector<double> coords::real_s_distance(double s1, double s2, double trackLength)
{
	double classicalDistance = abs(s1 - s2);
	double realDistance;
	double inFront; // 1 is s1 , 2 is s2
	if (s1 > s2)
	{
		double distanceToEndOfTrack = trackLength - s1;
		double forwardDistance = distanceToEndOfTrack + s2;
		if (forwardDistance < classicalDistance)
		{
			realDistance = forwardDistance;
			inFront = 2;
		}
		else
		{
			realDistance = classicalDistance;
			inFront = 1;
		}
	}
	else
	{
		double distanceToEndOfTrack = trackLength - s2;
		double forwardDistance = distanceToEndOfTrack + s1;
		if (forwardDistance < classicalDistance)
		{
			realDistance = forwardDistance;
			inFront = 1;
		}
		else
		{
			realDistance = classicalDistance;
			inFront = 2;
		}
	}
	return vector<double>{realDistance, inFront};
}

/// Create a vector of indexes of Waypoint around the current position
vector<int> coords::getLocalWayPointIndexes(int index, int back, int front, int wp_size) {
	// take K waypoints before and M ahead
	int k = back;
	int m = front;
	int prev_wp = index;
	vector<int> wp_indexes;
	// i need k-1 more previous wp
	for (int i = prev_wp - k + 1; i >= prev_wp; i--) {
		if (i < 0)
		{
			wp_indexes.push_back(i + wp_size); // at the beginning of the loop
		}
		else {
			wp_indexes.push_back(i);
		}
	}
	// now add m ahead
	for (int i = prev_wp + 1; i <= prev_wp + m; i++) {
		if (i > wp_size)
		{
			wp_indexes.push_back(i - wp_size); // at the end of the loop
		}
		else
		{
			wp_indexes.push_back(i);
		}
	}
	return wp_indexes;
}

///  Create around global Frenet Coordinate s a Spline
Splines coords::createLocalSplines(double s, vector<WayPoint> &wp, double trackLength) {
	int prev_wp = -1;

	while ((prev_wp < (int)(wp.size() - 1)) && s > wp[prev_wp + 1].s)
	{
		prev_wp++;
	}

	int back = 5;
	int front = 15;
	auto indexes = std::move(getLocalWayPointIndexes(prev_wp, back, front, wp.size()));

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
		// add max_s to all variables less than last element (which is the left most element)
		for (auto element : ss) {
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
vector<double> coords::evaluateSplineAtS(double s, double d, Splines sp, double trackLength) {
	double x = 0.0;
	double y = 0.0;

	if (sp.start_s < sp.end_s) // this is a normal spline not at the end of the track
	{
		if (s >= sp.start_s && s <= sp.end_s) { // in the range
			x = sp.x(s) + sp.dx(s) * d; // the contribution from S and the contribiution from d
			y = sp.y(s) + sp.dy(s) * d;
			return { x, y };
		}
	}
	else // we are at the loop of the track. I 
	{
		if (s > sp.start_s)
		{
			// since this is at the loop point if s > sp.start_s then its definately in the range
			// and there is also no need to add the extra max_s
			x = sp.x(s) + sp.dx(s) * d; // the contribution from S and the contribiution from d
			y = sp.y(s) + sp.dy(s) * d;
			return { x, y };
		}
		else if (s < sp.start_s && s < sp.end_s)
		{
			// this means that its also in the range but I must add max_s to retrieve the x, y values
			x = sp.x(s + trackLength) + sp.dx(s + trackLength) * d; // the contribution from S and the contribiution from d
			y = sp.y(s + trackLength) + sp.dy(s + trackLength) * d;
			return { x, y };
		}
	}
	return { x, y };
}
