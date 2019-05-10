
#include "spline.h"
using namespace std;


double ref_vel = 0.0; //mph
int lane = 1;

void definexypoints_circle(vector<double>& next_x_vals, vector<double>& next_y_vals, double car_x, double car_y, double car_yaw, vector<double> previous_path_x, vector<double> previous_path_y)
{
	double pos_x;
	double pos_y;
	double angle;
	int path_size = previous_path_x.size();

	for (int i = 0; i < path_size; ++i) {
		next_x_vals.push_back(previous_path_x[i]);
		next_y_vals.push_back(previous_path_y[i]);
	}

	if (path_size == 0) {
		pos_x = car_x;
		pos_y = car_y;
		angle = deg2rad(car_yaw);
	}
	else {
		pos_x = previous_path_x[path_size - 1];
		pos_y = previous_path_y[path_size - 1];

		double pos_x2 = previous_path_x[path_size - 2];
		double pos_y2 = previous_path_y[path_size - 2];
		angle = atan2(pos_y - pos_y2, pos_x - pos_x2);
	}

	double dist_inc = 0.5;
	for (int i = 0; i < 50 - path_size; ++i) {
		next_x_vals.push_back(pos_x + (dist_inc)* cos(angle + (i + 1) * (pi() / 100)));
		next_y_vals.push_back(pos_y + (dist_inc)* sin(angle + (i + 1) * (pi() / 100)));
		pos_x += (dist_inc)* cos(angle + (i + 1) * (pi() / 100));
		pos_y += (dist_inc)* sin(angle + (i + 1) * (pi() / 100));
	}


}

void definexypoints_straight(vector<double> & next_x_vals, vector<double> & next_y_vals, double car_s, vector<double> previous_path_x, vector<double> previous_path_y, vector<double> maps_s, vector<double> maps_x, vector<double> maps_y)
{

	double dist_inc = 0.5;
	for (int i = 0; i < 50; ++i) {
		double next_s = car_s + (i + 1) * dist_inc;
		double next_d = 6;
		vector<double> xy = getXY(next_s, next_d, maps_s, maps_x, maps_y);
		next_x_vals.push_back(xy[0]);
		next_y_vals.push_back(xy[1]);
	}
}


void definexypoints_straight_1(vector<double> & next_x_vals, vector<double> & next_y_vals, double car_s, double car_x, double car_y, double car_yaw, vector<double> previous_path_x, vector<double> previous_path_y, vector<double> maps_s, vector<double> maps_x, vector<double> maps_y)
{
	int prev_size = previous_path_x.size();


	vector<double> ptsx;
	vector<double> ptsy;
	int lane = 1;

	// Reference states.
	double ref_vel = 49.5; //mph
	double ref_x = car_x;
	double ref_y = car_y;
	double ref_yaw = deg2rad(car_yaw);

	// Previous list is cloe to empty. Car is the initial reference.
	if (prev_size < 2)
	{
		// Points tangent to car.
		double prev_car_x = car_x - cos(car_yaw);
		double prev_car_y = car_y - sin(car_yaw);

		ptsx.push_back(prev_car_x);
		ptsx.push_back(car_x);

		ptsy.push_back(prev_car_y);
		ptsy.push_back(car_y);

	}
	else
	{
		// Previous path as reference.
		// Assign previous path points as initial reference.
		ref_x = previous_path_x[prev_size - 1];
		ref_y = previous_path_y[prev_size - 1];

		double ref_x_prev = previous_path_x[prev_size - 2];
		double ref_y_prev = previous_path_y[prev_size - 2];
		ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

		ptsx.push_back(ref_x_prev);
		ptsx.push_back(ref_x);

		ptsy.push_back(ref_y_prev);
		ptsy.push_back(ref_y);

	}
	//Frenet co-ordinates with 30m spaced points w.r.t initial reference. 
	vector<double> next_wp0 = getXY(car_s + 30, (2 + 4 * lane), maps_s, maps_x, maps_y);
	vector<double> next_wp1 = getXY(car_s + 60, (2 + 4 * lane), maps_s, maps_x, maps_y);
	vector<double> next_wp2 = getXY(car_s + 90, (2 + 4 * lane), maps_s, maps_x, maps_y);

	ptsx.push_back(next_wp0[0]);
	ptsx.push_back(next_wp1[0]);
	ptsx.push_back(next_wp2[0]);

	ptsy.push_back(next_wp0[1]);
	ptsy.push_back(next_wp1[1]);
	ptsy.push_back(next_wp2[1]);

	for (int i = 0; i < ptsx.size(); i++)
	{
		//Shift car reference angle.
		double shift_x = ptsx[i] - ref_x;
		double shift_y = ptsy[i] - ref_y;

		ptsx[i] = (shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw));
		ptsy[i] = (shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw));



	}
	// Create a spline.
	tk::spline s;

	// Set points to spline.
	s.set_points(ptsx, ptsy);
	// Add previous points.
	for (int i = 0; i < prev_size; i++)
	{
		next_x_vals.push_back(previous_path_x[i]);
		next_y_vals.push_back(previous_path_y[i]);
	}

	// Spacing spline points to get desired velocity.
	double target_x = 30.0;
	double target_y = s(target_x);
	double target_dist = sqrt((target_x) * (target_x)+(target_y) * (target_y));

	double x_add_on = 0;

	// Ensuring we output 50 points.
	for (int i = 1; i <= 50 - prev_size; i++)
	{
		double N = (target_dist / (0.02 * ref_vel / 2.24));
		double x_point = x_add_on + (target_x) / N;
		double y_point = s(x_point);

		x_add_on = x_point;

		double x_ref = x_point;
		double y_ref = y_point;

		// Rotate back to normal.
		x_point = (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));
		y_point = (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw));

		x_point += ref_x;
		y_point += ref_y;

		next_x_vals.push_back(x_point);
		next_y_vals.push_back(y_point);

	}

	/*double dist_inc = 0.5;
	for (int i = 0; i < 50; ++i) {
		double next_s = car_s + (i + 1) * dist_inc;
		double next_d = 6;
		vector<double> xy = getXY(next_s, next_d, maps_s, maps_x, maps_y);
		next_x_vals.push_back(xy[0]);
		next_y_vals.push_back(xy[1]);
	}*/
}

void definexypoints_straight_2(vector<double> & next_x_vals, vector<double> & next_y_vals, double car_s, double car_x, double car_y, double car_yaw, double end_path_s, double end_path_d, vector<vector<double>> sensor_fusion, vector<double> previous_path_x, vector<double> previous_path_y, vector<double> maps_s, vector<double> maps_x, vector<double> maps_y)
{
	int prev_size = previous_path_x.size();

	if (prev_size > 0)
	{
		car_s = end_path_s;
	}
	bool too_close = false;
	bool is_left_lane_safe = true;
	bool is_right_lane_safe = true;
	const double safe_dist = 30.0;

	// Iterate through other vehicles.
	for (int i = 0; i < sensor_fusion.size(); i++)
	{
		double vx = sensor_fusion[i][3];
		double vy = sensor_fusion[i][4];
		double check_speed = sqrt(vx * vx + vy * vy);
		double check_car_s = sensor_fusion[i][5];
		float d = sensor_fusion[i][6];
		// Other vehicle is in same lane as ours.
		if (d < (2 + 4 * lane + 2) && d >(2 + 4 * lane - 2))
		{
			check_car_s += (double)(prev_size * 0.02 * check_speed);

			if ((check_car_s > car_s) && ((check_car_s - car_s) < safe_dist))
			{
				//ref_vel = 29.5; //mph
				too_close = true;
			}
		}
		// Relative distance betweeen our vehicle and the other vehicle.
		double rel_dist = abs(check_car_s - car_s);
		// Check other vehicle is left of our vehicle.
		int rel_left_lane = lane - 1;
		if (is_left_lane_safe && rel_left_lane >= 0 && d < (2 + 4 * rel_left_lane + 2) && d >(2 + 4 * rel_left_lane - 2)) {

			if (rel_dist < safe_dist) {
				is_left_lane_safe = false;
			}
		}
		// Check other vehicle is in right of our vehicle.
		int rel_right_lane = lane + 1;
		if (is_right_lane_safe && rel_right_lane <= 2 && d < (2 + 4 * rel_right_lane + 2) && d >(2 + 4 * rel_right_lane - 2)) {

			if (rel_dist < safe_dist) {
				is_right_lane_safe = false;
			}
		}

	}
	// Change lane if vehicle ahead is too close and it is safe.
	if (too_close) {
		if (lane != 0 && is_left_lane_safe)
			lane--;
		else if (lane != 2 && is_right_lane_safe)
			lane++;
		else
			ref_vel -= 0.224;
	}
	else if (ref_vel < 49.5) {
		ref_vel += 0.224;
	}

	vector<double> ptsx;
	vector<double> ptsy;
	// Reference states.
	double ref_x = car_x;
	double ref_y = car_y;
	double ref_yaw = deg2rad(car_yaw);

	// Previous list is close to empty. Car is the initial reference.
	if (prev_size < 2)
	{
		// Points tangent to car.
		double prev_car_x = car_x - cos(car_yaw);
		double prev_car_y = car_y - sin(car_yaw);

		ptsx.push_back(prev_car_x);
		ptsx.push_back(car_x);

		ptsy.push_back(prev_car_y);
		ptsy.push_back(car_y);

	}
	else
	{
		// Previous path as reference.
		// Assign previous path points as initial reference.
		ref_x = previous_path_x[prev_size - 1];
		ref_y = previous_path_y[prev_size - 1];

		double ref_x_prev = previous_path_x[prev_size - 2];
		double ref_y_prev = previous_path_y[prev_size - 2];
		ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

		ptsx.push_back(ref_x_prev);
		ptsx.push_back(ref_x);

		ptsy.push_back(ref_y_prev);
		ptsy.push_back(ref_y);

	}
	//Frenet co-ordinates with 30m spaced points w.r.t initial reference. 
	vector<double> next_wp0 = getXY(car_s + 30, (2 + 4 * lane), maps_s, maps_x, maps_y);
	vector<double> next_wp1 = getXY(car_s + 60, (2 + 4 * lane), maps_s, maps_x, maps_y);
	vector<double> next_wp2 = getXY(car_s + 90, (2 + 4 * lane), maps_s, maps_x, maps_y);

	ptsx.push_back(next_wp0[0]);
	ptsx.push_back(next_wp1[0]);
	ptsx.push_back(next_wp2[0]);

	ptsy.push_back(next_wp0[1]);
	ptsy.push_back(next_wp1[1]);
	ptsy.push_back(next_wp2[1]);

	for (int i = 0; i < ptsx.size(); i++)
	{
		//Shift car reference angle.
		double shift_x = ptsx[i] - ref_x;
		double shift_y = ptsy[i] - ref_y;

		ptsx[i] = (shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw));
		ptsy[i] = (shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw));
	}
	// Create a spline.
	tk::spline s;
	// Set points to spline.
	s.set_points(ptsx, ptsy);
	// Add previous points.
	for (int i = 0; i < prev_size; i++)
	{
		next_x_vals.push_back(previous_path_x[i]);
		next_y_vals.push_back(previous_path_y[i]);
	}

	// Spacing spline points to get desired velocity.
	double target_x = 30.0;
	double target_y = s(target_x);
	double target_dist = sqrt((target_x) * (target_x)+(target_y) * (target_y));

	double x_add_on = 0;
	// Ensuring 50 points are outputted.
	for (int i = 1; i <= 50 - prev_size; i++)
	{
		// Compiler will optimize.
		double N = (target_dist / (0.02 * ref_vel / 2.24));
		double x_point = x_add_on + (target_x) / N;
		double y_point = s(x_point);

		x_add_on = x_point;

		double x_ref = x_point;
		double y_ref = y_point;
		// Rotate back to normal.
		x_point = (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));
		y_point = (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw));

		x_point += ref_x;
		y_point += ref_y;

		next_x_vals.push_back(x_point);
		next_y_vals.push_back(y_point);

	}
}