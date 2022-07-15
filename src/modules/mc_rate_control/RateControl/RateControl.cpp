/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file RateControl.cpp
 */

#include <RateControl.hpp>
#include <px4_platform_common/defines.h>



#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include<uORB/topics/vehicle_status.h>
#include <uORB/uORB.h>
#include<string.h>
#include<iostream>


//SWITCH TO PID EULER RATE CONTROL VERSION 0:CLOSE
#define PID_EULER_VER_CONTROLLER 1

//SWITCH TO PID ORIGINAL CONTROL VERSION 0:CLOSE
#define PID_ORIGIN_VER_CONTROLLER 0
//SWITCH TO SMC RATE CONTROL VERSION  0:CLOSE
#define SMC_CONTROLLER 0

using namespace matrix;
Vector3f torque;
int Flag_file;             //flag for file
bool PID_EULER_FLAG = 0;   //temporary flag for PID EULER
bool  SMC_FLAG  = 0;       //temporary flag for SMC
bool PID_ORIG_FLAG = 0;   //temoprary flag for PID origin

#define IS_MANUAL_MODE "MANUAL"   // MANUAL mode keywords, will use it to compare current flight mode

//subscribe the vehicle current mode status
vehicle_status_s vehicle_Status;
uORB::Subscription _vehicle_status_subscription {ORB_ID(vehicle_status)};

//bad smc param for origin iris.sdf
// float c_ = 4.2;
// float k_ = 0.36;

//good param for s500 .iris sdf  c = 3.6  k = 0.315

//param for smcnn : c = 3.8 k = 0.35

// float c_ = 3.8;
// float k_ = 0.35;

float c_ = 3.8;
float k_ = 0.35;


float c_best = 3.95;
float k_best = 0.2;
Vector3f c_smc;
Vector3f k_smc;



//EULER PID parameters part:
//Euler PID version with angle error  ok !
const float _p = 1.1;
const float _i = 0.3;
const float _d = 0.2;



//PID original(PX4) parameters P , I , D
Vector3f pid_p_orig ;
Vector3f pid_i_orig ;
Vector3f pid_d_orig ;







void RateControl::setGains(const Vector3f &P, const Vector3f &I, const Vector3f &D)
{


	//origin version PID gain
	 _gain_p = P;
	 _gain_i = I;
	 _gain_d = D;


	pid_p_orig = _gain_p;
	pid_i_orig = _gain_i;
	pid_d_orig = _gain_d;


}




void RateControl::setSaturationStatus(const Vector<bool, 3> &saturation_positive, const Vector<bool, 3> &saturation_negative)
{
	_control_allocator_saturation_positive = saturation_positive;
	_control_allocator_saturation_negative = saturation_negative;
}


// function to get current mode status
const char* RateControl::getVehicleStatus(vehicle_status_s vs)
	{
		const char* flight_mode = "UNKNOWN";
		switch (vs.nav_state)
		{
		case vehicle_status_s::NAVIGATION_STATE_MANUAL:
			flight_mode = "MANUAL";
			break;

		case vehicle_status_s::NAVIGATION_STATE_ALTCTL:
			flight_mode = "ALTITUDE";
			break;

		case vehicle_status_s::NAVIGATION_STATE_POSCTL:
			flight_mode = "POSITION";
			break;

		case vehicle_status_s::NAVIGATION_STATE_AUTO_RTL:
			flight_mode = "RETURN";
			break;

		case vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION:
			flight_mode = "MISSION";
			break;

		case vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER:
		case vehicle_status_s::NAVIGATION_STATE_DESCEND:
		case vehicle_status_s::NAVIGATION_STATE_AUTO_TAKEOFF:
		case vehicle_status_s::NAVIGATION_STATE_AUTO_LAND:
		case vehicle_status_s::NAVIGATION_STATE_AUTO_FOLLOW_TARGET:
		case vehicle_status_s::NAVIGATION_STATE_AUTO_PRECLAND:
			flight_mode = "AUTO";
			break;

		case vehicle_status_s::NAVIGATION_STATE_AUTO_LANDENGFAIL:
			flight_mode = "FAILURE";
			break;

		case vehicle_status_s::NAVIGATION_STATE_ACRO:
			flight_mode = "ACRO";
			break;

		case vehicle_status_s::NAVIGATION_STATE_TERMINATION:
			flight_mode = "TERMINATE";
			break;

		case vehicle_status_s::NAVIGATION_STATE_OFFBOARD:
			flight_mode = "OFFBOARD";
			break;

		case vehicle_status_s::NAVIGATION_STATE_STAB:
			flight_mode = "STABILIZED";
			break;
		}
		return flight_mode;
	}




Vector3f RateControl::update(const Vector3f &rate, const Vector3f &rate_sp, const Vector3f &angular_accel,
			     const float dt, const bool landed)
{



	Vector3f rate_error = rate_sp - rate;	// angular rates error
	Vector3f rad_error = euler_rad_sp - euler_rad_current; // angle error in radian

	//update drone flight mode status and compare with manual mode keywords
	_vehicle_status_subscription.update(&vehicle_Status);
	const char* result = getVehicleStatus(vehicle_Status);

	//IF WE ARE IN MANUAL MODE
	if(strcmp(result,IS_MANUAL_MODE)==0)
	{

		//SET ALL PID GAINS TO ORIGIN PX4 VALUES.
		//STOP ALL CUSTOM RATE CONTROLLERS AND SWITCH TO RATE CONTROLLER PID PX4 VERSION.
		SMC_FLAG = 0;
		PID_EULER_FLAG = 0;
		PID_ORIG_FLAG = 1;

		_gain_p = pid_p_orig;
		_gain_i = pid_i_orig;
		_gain_d = pid_d_orig;

	}

	//IF NOT IN MANUAL MODE
	else
	{
		//IF WE CHOOSE PID EULER RATE CONTROLLER , SET PID EULER FLAG = 1
		#if PID_EULER_VER_CONTROLLER
			PID_EULER_FLAG = 1;
			PID_ORIG_FLAG = 0;
			SMC_FLAG = 0;

		#endif

		#if PID_ORIG_VER_CONTROLLER
			PID_ORIG_FLAG = 1;
			PID_EULER_FLAG = 1;
			SMC_FLAG = 0;



		#endif

		//IF WE CHOOSE SMC  RATE CONTROLLER , SMC FLAG = 1
		#if SMC_CONTROLLER
			SMC_FLAG = 1;
			PID_ORIG_FLAG = 0;
			PID_EULER_FLAG = 0;
		#endif

		//IF PID_EULER_FLAG = 1, SET ALL PID GAINS TO OUR CUSTOM GAINS.
		if(PID_EULER_FLAG)
		{
			Flag_file = 1;
			_gain_p = Vector3f(_p,_p,_p);
			_gain_i = Vector3f(_i,_i,_i);
			_gain_d = Vector3f(_d,_d,_d);


		}

		// IF PID ORIG FLAG = 1 ,SET ALL PID GAINS TO ORIGINAL GAINS
		if(PID_ORIG_FLAG)
		{
			Flag_file = 0;
			_gain_p = pid_p_orig;
			_gain_i = pid_i_orig;
			_gain_d = pid_d_orig;


		}


		//IF SMC FLAG = 1  SET ALL PID GAINS TO OUR CUSTOM GAINS.
		if(SMC_FLAG)
		{

			Flag_file = 2;

			//if(best)
			//{
				c_smc = Vector3f(c_best,c_best,c_best);
				k_smc = Vector3f(k_best,k_best,k_best);

			//}
			//else
			//{
				//c_smc = Vector3f(c_,c_,c_);
				//k_smc = Vector3f(k_,k_,k_);
			//}
		}
	}

	//IF PID EULER FLAG = 1 USE COMMAND LAW  :PID EULER
	if(PID_EULER_FLAG)
	{
		torque = _gain_p.emult(rad_error)+rad_error_int.emult(_gain_i) + _gain_d.emult(rate_error);


	}

	else if (SMC_FLAG)
	{
		Vector3f SMCrate_S =c_smc.emult(rad_error) +rate_error;   //Surface S for SMC
		torque =SMCrate_S.emult(k_smc)+Zeta*ControlMath::sign(SMCrate_S);
	}
	// ELSE USE ORIGINAL PX4 COMMAND LAW
	else if(PID_ORIG_FLAG)
	{
		torque = _gain_p.emult(rate_error) + _rate_int- _gain_d.emult(angular_accel) + _gain_ff.emult(rate_sp);
		//cout<<_rate_int(0)<<","<<_rate_int(1)<<","<<_rate_int(2)<<endl;
	}


	if (!landed)
	{
		if(PID_EULER_FLAG)
		{

			// IF PID EULER VERSION, UPDATE INTEGRAL VALUE BY USING OUR METHOD
			updateIntegral_Ver_Euler(rad_error, dt);
		}
		else if(PID_ORIG_FLAG)
		{
			//IF PID ORIGIN VERSION ,UPDATE INTEGRAL VALUE BY USING THEIR OWN METHOD
			updateIntegral(rate_error,dt);
		}

	}

	return torque;
}

//RateControl::updateIntegral ORIGIN PX4
void RateControl::updateIntegral(Vector3f &rate_error, const float dt)
{
	for (int i = 0; i < 3; i++)
	 {
		// prevent further positive control saturation
		if (_control_allocator_saturation_positive(i)) {
			rate_error(i) = math::min(rate_error(i), 0.f);
		}

		// prevent further negative control saturation
		if (_control_allocator_saturation_negative(i)) {
			rate_error(i) = math::max(rate_error(i), 0.f);
		}

		// I term factor: reduce the I gain with increasing rate error.
		// This counteracts a non-linear effect where the integral builds up quickly upon a large setpoint
		// change (noticeable in a bounce-back effect after a flip).
		// The formula leads to a gradual decrease w/o steps, while only affecting the cases where it should:
		// with the parameter set to 400 degrees, up to 100 deg rate error, i_factor is almost 1 (having no effect),
		// and up to 200 deg error leads to <25% reduction of I.
		float i_factor = rate_error(i) / math::radians(400.f);
		i_factor = math::max(0.0f, 1.f - i_factor * i_factor);

		// Perform the integration using a first order method
		float rate_i = _rate_int(i) + i_factor * _gain_i(i) * rate_error(i) * dt;

		// do not propagate the result if out of range or invalid
		if (PX4_ISFINITE(rate_i))
		{
			_rate_int(i) = math::constrain(rate_i, -_lim_int(i), _lim_int(i));   //-0.3----+0.3
		}


	}


}



void RateControl::updateIntegral_Ver_Euler(Vector3f rad_err, const float dt)
{
	for (int i = 0; i < 3; i++)
	{
		rad_error_int(i) = rad_error_int(i)+rad_err(i)*dt;
	}
}




void RateControl::getRateControlStatus(rate_ctrl_status_s &rate_ctrl_status)
{
	rate_ctrl_status.rollspeed_integ = _rate_int(0);
	rate_ctrl_status.pitchspeed_integ = _rate_int(1);
	rate_ctrl_status.yawspeed_integ = _rate_int(2);
}
