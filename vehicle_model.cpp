/**
 * @file vehicle_model.cpp
 *
 * @author abdelrhman werby
 * Contact: abdelrhmanwerby@gmail.com
 *
 */

#include "vehicle_model.h"
#include <cmath>
#include <random>
#include <iostream>

using namespace std;

#define RAD2DEG 57.2957795

state::state(double _x, double _y, double _th, double _v, double _cte, double _eth)
{
	this->x = _x;
	this->y = _y;
	this->th = _th;
	this->v = _v;
	this->cte = _cte;
	this->eth = _eth;
}

void state::print_state()
{
	cout << "X = "       << this->x << " Meter" << endl;
	cout << "Y = "       << this->y << " Meter" << endl;
	cout << "V = "       << this->v << " M/S" << endl;
	cout << "Theta = "   << this->th  * RAD2DEG << " Degree" << endl;
	cout << "E_theta = " << this->eth * RAD2DEG << " Degree" << endl;
	cout << "CTE= "      << this->cte << " Meter" << endl;
}


inputs::inputs(double _steerangle, double _accelartion)
{
	this->steerangle = _steerangle;
	this->accelartion = _accelartion;
}

void inputs::print_inputs()
{
	cout << "steer angle: " << this->steerangle;
	cout << "\t accelration: " << this->accelartion << endl;
}

double polyval(double* p, int n, double x)
{
	double px = 0.0;

	for (int i = 0; i < n; i++)
	{
		px += p[i] * pow(x, n - i - 1);
	}

	return px;
}

state update_state(inputs input, state last_state, double* coff, double dt, double base_length)
{
	state current_state;
	double th_des;

	current_state.x = last_state.x + last_state.v * cos(last_state.th) * dt;
	current_state.y = last_state.y + last_state.v * sin(last_state.th) * dt;
	current_state.th = last_state.th + (last_state.v / base_length) * input.steerangle * dt;
	current_state.v = last_state.v + input.accelartion * dt;

	th_des = atan(coff[2] + 2 * coff[1] * last_state.x + 3 * coff[0] * pow(last_state.x, 2));

	current_state.cte = polyval(coff, 4, last_state.x) - last_state.y + (last_state.v * sin(last_state.eth) * dt);
	current_state.eth = last_state.th - th_des + ((last_state.v / base_length) * input.steerangle * dt);

	return current_state;
}

state update_state_noise(inputs input, state last_state, double* coff, double dt, double base_length)
{
	state current_state;
	double th_des;

	current_state.x = last_state.x + last_state.v * cos(last_state.th) * dt;
	current_state.y = last_state.y + last_state.v * sin(last_state.th) * dt;
	current_state.th = last_state.th + (last_state.v / base_length) * input.steerangle * dt + (((double)rand() / (RAND_MAX)) - 0.5) * 0.01;
	current_state.v = last_state.v + input.accelartion * dt - ((double)rand() / (RAND_MAX)) * 0.05;

	th_des = atan(coff[2] + 2 * coff[1] * last_state.x + 3 * coff[0] * pow(last_state.x, 2));

	current_state.cte = polyval(coff, 4, last_state.x) - last_state.y + (last_state.v * sin(last_state.eth) * dt);
	current_state.eth = last_state.th - th_des + ((last_state.v / base_length) * input.steerangle * dt);

	return current_state;
}
