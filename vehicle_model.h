/**
 * @file vehicle_model.h
 *
 * @author abdelrhman werby
 * Contact: abdelrhmanwerby@gmail.com
 *
 */

#pragma once

/**
 * @Brief : class save the vehicle state and location
 *
 * @param x : the X (longtuide) postion of the vehicle in meter.
 * @param y : the Y (letral) postion of the vehicle in meter.
 * @param v : the linear velocity of the vehicle in meter per second.
 * @param th  : the heading angle of the vehicle in radian.
 * @param eth : the heading angle error to the ref trajectory of the vehicle in radian.
 * @param cte : the letral location error to the ref trajectory of the vehicle in meter.
 *
 */
class state
{
public:
	double x;
	double y;
	double th;
	double v;
	double cte;
	double eth;
	state(double _x=0, double _y=0, double _th=0, double _v=0, double _cte=0, double eth=0);
	void print_state();

};


/**
 * @Brief : class save the vehicle inputs accelartion and steer angle
 *
 * @param steerangle  : the steer angle of the vehicle front wheel in radian (-0.43, 0.43).
 * @param accelartion : the accelartion of the vehicle (-1, 1).
 *
 */
class inputs
{
public:
	double steerangle;
	double accelartion;
	inputs(double _steerangle=0, double _accelartion=0);
	void print_inputs();
};


/**
 * @Brief : evulate the f(x) of polynomial
 *
 * @param p : the coffencint array pointer.
 * @param n : the coff array length.
 * @param x : the input number to the polynomial.
 *
 * @Return : f(x)
 */
double polyval(double* p, int n, double x);

/**
 * @Brief : function to the kinmatics bycicle model of vehicle.
 *
 * @param input : the inputs commmand to the vehicle.
 * @param last_state : the last state of the vehicle.
 * @param coff : the coff of the polynomial to ref trajectory.
 * @param dt  : the sample time of system in seconds.
 * @param base_length : the base length of the vehicle in meter.
 *
 * @Return : the new state of the vehicle
 */
state update_state(inputs input, state last_state, double* coff, double dt, double base_length);

/**
 * @Brief : function to the kinmatics bycicle model of vehicle with noise in heading angle and add friction to reduce velocity.
 *
 * @param input : the inputs commmand to the vehicle.
 * @param last_state : the last state of the vehicle.
 * @param coff : the coff of the polynomial to ref trajectory.
 * @param dt  : the sample time of system in seconds.
 * @param base_length : the base length of the vehicle in meter.
 *
 * @Return : the new state of the vehicle
 */
state update_state_noise(inputs input, state last_state, double* coff, double dt, double base_length);
