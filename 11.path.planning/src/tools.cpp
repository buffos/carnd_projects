
#include "tools.h"

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

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
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

	return {frenet_s, frenet_d};
}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> coords::getXY(double s, double d, vector<WayPoint> &wp)
{
	int prev_wp = -1;

	while (s > wp[prev_wp + 1].s && (prev_wp < (int)(wp.size() - 1)))
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

	return {x, y};
}

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