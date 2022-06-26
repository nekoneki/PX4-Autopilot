/****************************************************************************
 *
 *   Copyright (c) 2018 - 2019 PX4 Development Team. All rights reserved.
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
 * @file PositionControl.hpp
 *
 * A cascaded position controller for position/velocity control only.
 */

#pragma once
#include <iostream>
#include <lib/mathlib/mathlib.h>
#include <float.h>
#include <matrix/matrix/math.hpp>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>
#include <fstream>
#include "ControlMath.hpp"
using namespace matrix;
using namespace std;

extern int best;


struct fuzzy_e
{
	bool NB = 1;
	bool NS = 1;
	bool Z = 1;
	bool PS = 1;
	bool PB = 1;
	float e_NB = 0;
	float e_NS = 0;
	float e_Z = 0;
	float e_PS = 0;
	float e_PB = 0;
};


struct fuzzy_edot
{
	bool NB = 1;
	bool NS = 1;
	bool Z = 1;
	bool PS = 1;
	bool PB = 1;
	float edot_NB = 0;
	float edot_NS = 0;
	float edot_Z = 0;
	float edot_PS = 0;
	float edot_PB = 0;
};



//fuzzy logic parameters
const float A = 3.33333;
const float B = 2.5;
//constant for c:
const float c_VS = 0.4;
const float c_S = 0.5;
const float c_M = 0.6;
const float c_B = 0.7;
const float c_VB = 0.8;
//constant for k:
const float k_VS = 2.0;
const float k_S = 2.2;
const float k_M = 2.4;
const float k_B = 2.6;
const float k_VB = 2.8;


struct PositionControlStates {
	matrix::Vector3f position;
	matrix::Vector3f velocity;
	matrix::Vector3f acceleration;
	float yaw;
};


/**
 * 	Core Position-Control for MC.
 * 	This class contains P-controller for position and
 * 	PID-controller for velocity.
 * 	Inputs:
 * 		vehicle position/velocity/yaw
 * 		desired set-point position/velocity/thrust/yaw/yaw-speed
 * 		constraints that are stricter than global limits
 * 	Output
 * 		thrust vector and a yaw-setpoint
 *
 * 	If there is a position and a velocity set-point present, then
 * 	the velocity set-point is used as feed-forward. If feed-forward is
 * 	active, then the velocity component of the P-controller output has
 * 	priority over the feed-forward component.
 *
 * 	A setpoint that is NAN is considered as not set.
 * 	If there is a position/velocity- and thrust-setpoint present, then
 *  the thrust-setpoint is ommitted and recomputed from position-velocity-PID-loop.
 */
class PositionControl
{
public:

	PositionControl() = default;
	~PositionControl() = default;






	/**
	 * Set the position control gains
	 * @param P 3D vector of proportional gains for x,y,z axis
	 */
	void setPositionGains(const matrix::Vector3f &P)
	{
		_gain_pos_p = Vector3f(5,5,5);
	}

	/**
	 * Set the velocity control gains
	 * @param P 3D vector of proportional gains for x,y,z axis
	 * @param I 3D vector of integral gains
	 * @param D 3D vector of derivative gains
	 */
	void setVelocityGains(const matrix::Vector3f &P, const matrix::Vector3f &I, const matrix::Vector3f &D);


	/**
	 * Set the maximum velocity to execute with feed forward and position control
	 * @param vel_horizontal horizontal velocity limit
	 * @param vel_up upwards velocity limit
	 * @param vel_down downwards velocity limit
	 */
	void setVelocityLimits(const float vel_horizontal, const float vel_up, float vel_down);

	/**
	 * Set the minimum and maximum collective normalized thrust [0,1] that can be output by the controller
	 * @param min minimum thrust e.g. 0.1 or 0
	 * @param max maximum thrust e.g. 0.9 or 1
	 */
	void setThrustLimits(const float min, const float max);

	/**
	 * Set margin that is kept for horizontal control when prioritizing vertical thrust
	 * @param margin of normalized thrust that is kept for horizontal control e.g. 0.3
	 */
	void setHorizontalThrustMargin(const float margin);

	/**
	 * Set the maximum tilt angle in radians the output attitude is allowed to have
	 * @param tilt angle in radians from level orientation
	 */
	void setTiltLimit(const float tilt) { _lim_tilt = tilt; }

	/**
	 * Set the normalized hover thrust
	 * @param thrust [0.1, 0.9] with which the vehicle hovers not acelerating down or up with level orientation
	 */
	void setHoverThrust(const float hover_thrust) { _hover_thrust = math::constrain(hover_thrust, 0.1f, 0.9f); }

	/**
	 * Update the hover thrust without immediately affecting the output
	 * by adjusting the integrator. This prevents propagating the dynamics
	 * of the hover thrust signal directly to the output of the controller.
	 */
	void updateHoverThrust(const float hover_thrust_new);

	/**
	 * Pass the current vehicle state to the controller
	 * @param PositionControlStates structure
	 */
	void setState(const PositionControlStates &states);

	/**
	 * Pass the desired setpoints
	 * Note: NAN value means no feed forward/leave state uncontrolled if there's no higher order setpoint.
	 * @param setpoint a vehicle_local_position_setpoint_s structure
	 */
	void setInputSetpoint(const vehicle_local_position_setpoint_s &setpoint);

	/**
	 * Apply P-position and PID-velocity controller that updates the member
	 * thrust, yaw- and yawspeed-setpoints.
	 * @see _thr_sp
	 * @see _yaw_sp
	 * @see _yawspeed_sp
	 * @param dt time in seconds since last iteration
	 * @return true if update succeeded and output setpoint is executable, false if not
	 */
	bool update(const float dt);

	/**
	 * Set the integral term in xy to 0.
	 * @see _vel_int
	 */
	void resetIntegral() { _vel_int.setZero(); }

	/**
	 * Get the controllers output local position setpoint
	 * These setpoints are the ones which were executed on including PID output and feed-forward.
	 * The acceleration or thrust setpoints can be used for attitude control.
	 * @param local_position_setpoint reference to struct to fill up
	 */
	void getLocalPositionSetpoint(vehicle_local_position_setpoint_s &local_position_setpoint) const;

	/**
	 * Get the controllers output attitude setpoint
	 * This attitude setpoint was generated from the resulting acceleration setpoint after position and velocity control.
	 * It needs to be executed by the attitude controller to achieve velocity and position tracking.
	 * @param attitude_setpoint reference to struct to fill up
	 */
	void getAttitudeSetpoint(vehicle_attitude_setpoint_s &attitude_setpoint) const;

private:
	bool _inputValid();


	/////////////////////////FUZZY LOGIC START////////////////////////
	//-----------------FUZZIFICAITION FOR e--------------------
	struct fuzzy_e Fuzzification_e(float e)
	{
		fuzzy_e f_e;

		if(e<=(float)-0.6)
		{
	 		//e_NB function
			f_e.e_NB = 1;
		}

		else if(e>(float)-0.6&&e<=(float)-0.3)
		{
			//e_NB, e_NS function
			f_e.e_NB = -A*e-1;
			f_e.e_NS = A*e+2;

		}

		else if(e>(float)-0.3&&e<=0)
		{
			//e_NS, e_Z	 function
			f_e.e_NS = -A*e;
			f_e.e_Z = A*e+1;

		}

		else if(e>0&&e<=(float)0.3)
		{
			// e_Z, e_PS function
			f_e.e_Z = -A*e+1;
			f_e.e_PS = A*e;

		}

		else if(e>(float)0.3&&e<=(float)0.6)
		{
			//e_PS, e_PB function
			f_e.e_PS = -A*e+2;
			f_e.e_PB = A*e-1;

		}

		else if(e>(float)0.6)
		{
			//e_PB function
			f_e.e_PB = 1;
		}
		return f_e;
	}


	//-----------------FUZZIFICAITION FOR edot--------------------
	struct fuzzy_edot Fuzzification_edot(float edot)
	{
		fuzzy_edot f_edot;

		if(edot<=(float)-0.8)
		{
			//e_NB function
			f_edot.edot_NB = 1;

		}

		else if(edot>(float)-0.8&&edot<=(float)-0.4)
		{
			//e_NB, e_NS function

			f_edot.edot_NB = -B*edot-1;
			f_edot.edot_NS = B*edot+2;
		}

		else if(edot>(float)-0.4&&edot<=0)
		{
			//e_NS, e_Z	 function
			f_edot.edot_NS = -B*edot;
			f_edot.edot_Z = B*edot+1;
		}

		else if(edot>0&&edot<=(float)0.4)
		{
			//e_Z, e_PS	 function
			f_edot.edot_Z = -B*edot+1;
			f_edot.edot_PS = B*edot;
		}

		else if(edot>(float)0.4&&edot<=(float)0.8)
		{
			//e_PS, e_PB function
			f_edot.edot_PS = -B*edot+2;
			f_edot.edot_PB = B*edot-1;
		}

		else if(edot>(float)0.8)
		{
			//e_PB	 function
			f_edot.edot_PB = 1;
		}
		return f_edot;
	}


	//-----------------CALCULATE FINAL OUTPUT FOR c--------------------
	float Calculate_final_output_c(fuzzy_e f_e,fuzzy_edot f_edot)
	{

	//25 rules of fuzzification to calculate final output c with values of output level corresponding

	float output_level_c ;
	float w_c[25];  //firing_strength for c
	float WO_c[25]; //weighted_output_for_one_rule for c
	float firing_strength_total_c = 0;
	float weighted_output_total_c = 0;
	float final_output_c;


	if(f_e.NB)
	{
		if(f_edot.NB)
		{
			output_level_c = c_M;
			w_c[0] = ControlMath::min(f_e.e_NB,f_edot.edot_NB);
			WO_c[0] = output_level_c*w_c[0];

		}


		if(f_edot.NS)
		{
			output_level_c = c_S;
			w_c[1] = ControlMath::min(f_e.e_NB,f_edot.edot_NS);
			WO_c[1] = output_level_c*w_c[1];

		}

		if(f_edot.Z)
		{
			output_level_c = c_VS;
			w_c[2] = ControlMath::min(f_e.e_NB,f_edot.edot_Z);
			WO_c[2] = output_level_c*w_c[2];

		}
		if(f_edot.PS)
		{
			output_level_c = c_S;
			w_c[3] = ControlMath::min(f_e.e_NB,f_edot.edot_PS);
			WO_c[3] = output_level_c*w_c[3];

		}
		if(f_edot.PB)
		{
			output_level_c = c_M;
			w_c[4] = ControlMath::min(f_e.e_NB,f_edot.edot_PB);
			WO_c[4] = output_level_c*w_c[4];

		}
	}

	if(f_e.NS)
	{
		if(f_edot.NB)
		{
			output_level_c = c_B;
			w_c[5] = ControlMath::min(f_e.e_NS,f_edot.edot_NB);
			WO_c[5] = output_level_c*w_c[5];

		}

		if(f_edot.NS)
		{
			output_level_c = c_M;
			w_c[6] = ControlMath::min(f_e.e_NS,f_edot.edot_NS);
			WO_c[6] = output_level_c*w_c[6];

		}

		if(f_edot.Z)
		{
			output_level_c = c_S;
			w_c[7] = ControlMath::min(f_e.e_NS,f_edot.edot_Z);
			WO_c[7] = output_level_c*w_c[7];

		}

		if(f_edot.PS)
		{
			output_level_c = c_M;
			w_c[8] = ControlMath::min(f_e.e_NS,f_edot.edot_PS);
			WO_c[8] = output_level_c*w_c[8];

		}

		if(f_edot.PB)
		{
			output_level_c = c_B;
			w_c[9] = ControlMath::min(f_e.e_NS,f_edot.edot_PB);
			WO_c[9] = output_level_c*w_c[9];

		}
	}

	if(f_e.Z)
	{

		if(f_edot.NB)
		{
			output_level_c = c_VS;
			w_c[10] = ControlMath::min(f_e.e_Z,f_edot.edot_NB);
			WO_c[10] = output_level_c*w_c[10];

		}

		if(f_edot.NS)
		{
			output_level_c = c_B;
			w_c[11] = ControlMath::min(f_e.e_Z,f_edot.edot_NS);
			WO_c[11] = output_level_c*w_c[11];

		}

		if(f_edot.Z)
		{
			output_level_c = c_M;
			w_c[12] = ControlMath::min(f_e.e_Z,f_edot.edot_Z);
			WO_c[12] = output_level_c*w_c[12];

		}

		if(f_edot.PS)
		{
			output_level_c = c_B;
			w_c[13] = ControlMath::min(f_e.e_Z,f_edot.edot_PS);
			WO_c[13] = output_level_c*w_c[13];

		}

		if(f_edot.PB)
		{
			output_level_c = c_VS;
			w_c[14] = ControlMath::min(f_e.e_Z,f_edot.edot_PB);
			WO_c[14] = output_level_c*w_c[14];

		}



	}

	if(f_e.PS)
	{
		if(f_edot.NB)
		{
			output_level_c = c_B;
			w_c[15] = ControlMath::min(f_e.e_PS,f_edot.edot_NB);
			WO_c[15] = output_level_c*w_c[15];

		}

		if(f_edot.NS)
		{
			output_level_c = c_M;
			w_c[16] = ControlMath::min(f_e.e_PS,f_edot.edot_NS);
			WO_c[16] = output_level_c*w_c[16];

		}

		if(f_edot.Z)
		{
			output_level_c = c_S;
			w_c[17] = ControlMath::min(f_e.e_PS,f_edot.edot_Z);
			WO_c[17] = output_level_c*w_c[17];

		}

		if(f_edot.PS)
		{
			output_level_c = c_M;
			w_c[18] = ControlMath::min(f_e.e_PS,f_edot.edot_PS);
			WO_c[18] = output_level_c*w_c[18];

		}

		if(f_edot.PB)
		{
			output_level_c = c_B;
			w_c[19] = ControlMath::min(f_e.e_PS,f_edot.edot_PB);
			WO_c[19] = output_level_c*w_c[19];

		}


	}

	if(f_e.PB)
	{
		if(f_edot.NB)
		{
			output_level_c = c_M;
			w_c[20] = ControlMath::min(f_e.e_PB,f_edot.edot_NB);
			WO_c[20] = output_level_c*w_c[20];

		}

		if(f_edot.NS)
		{
			output_level_c = c_S;
			w_c[21] = ControlMath::min(f_e.e_PB,f_edot.edot_NS);
			WO_c[21] = output_level_c*w_c[21];

		}

		if(f_edot.Z)
		{
			output_level_c = c_VS;
			w_c[22] = ControlMath::min(f_e.e_PB,f_edot.edot_Z);
			WO_c[22] = output_level_c*w_c[22];

		}

		if(f_edot.PS)
		{
			output_level_c = c_S;
			w_c[23] = ControlMath::min(f_e.e_PB,f_edot.edot_PS);
			WO_c[23] = output_level_c*w_c[23];

		}

		if(f_edot.PB)
		{
			output_level_c = c_M;
			w_c[24] = ControlMath::min(f_e.e_PB,f_edot.edot_PB);
			WO_c[24] = output_level_c*w_c[24];

		}
	}


	for(int i =0; i<24;i++)
	{
		firing_strength_total_c = firing_strength_total_c+w_c[i];
		weighted_output_total_c = weighted_output_total_c+WO_c[i];
		//file1<<"No."<<i<<" w and it's weighted output(numerator) are:"<<"w_c:"<<w_c[i]<<",WO_c:"<<WO_c[i]<<endl;

	}

	final_output_c = weighted_output_total_c/firing_strength_total_c;

	return final_output_c;

	}

	//-----------------CALCULATE FINAL OUTPUT FOR k--------------------
	float Calculate_final_output_k(fuzzy_e f_e,fuzzy_edot f_edot)
	{

	//25 rules of fuzzification to calculate final output k
	float output_level_k ;
	float w_k[25]; //firing_strength for k
	float WO_k[25]; //weighted_output_for_one_rule for k
	float firing_strength_total_k = 0;
	float weighted_output_total_k = 0;

	float final_output_k;

	if(f_e.NB)
	{

		if(f_edot.NB)
		{
			output_level_k = k_M;
			w_k[0] = ControlMath::min(f_e.e_NB,f_edot.edot_NB);
			WO_k[0] = output_level_k*w_k[0];

		}


		if(f_edot.NS)
		{
			output_level_k = k_B;
			w_k[1] = ControlMath::min(f_e.e_NB,f_edot.edot_NS);
			WO_k[1] = output_level_k*w_k[1];

		}



		if(f_edot.Z)
		{
			output_level_k = k_VB;
			w_k[2] = ControlMath::min(f_e.e_NB,f_edot.edot_Z);
			WO_k[2] = output_level_k*w_k[2];

		}
		if(f_edot.PS)
		{
			output_level_k = k_B;
			w_k[3] = ControlMath::min(f_e.e_NB,f_edot.edot_PS);
			WO_k[3] = output_level_k*w_k[3];

		}
		if(f_edot.PB)
		{
			output_level_k = k_M;
			w_k[4] = ControlMath::min(f_e.e_NB,f_edot.edot_PB);
			WO_k[4] = output_level_k*w_k[4];

		}

	}


	if(f_e.NS)
	{

		if(f_edot.NB)
		{
			output_level_k = k_S;
			w_k[5] = ControlMath::min(f_e.e_NS,f_edot.edot_NB);
			WO_k[5] = output_level_k*w_k[5];

		}

		if(f_edot.NS)
		{
			output_level_k = k_M;
			w_k[6] = ControlMath::min(f_e.e_NS,f_edot.edot_NS);
			WO_k[6] = output_level_k*w_k[6];

		}

		if(f_edot.Z)
		{
			output_level_k = k_B;
			w_k[7] = ControlMath::min(f_e.e_NS,f_edot.edot_Z);
			WO_k[7] = output_level_k*w_k[7];

		}

		if(f_edot.PS)
		{
			output_level_k = k_M;
			w_k[8] = ControlMath::min(f_e.e_NS,f_edot.edot_PS);
			WO_k[8] = output_level_k*w_k[8];

		}

		if(f_edot.PB)
		{
			output_level_k = k_S;
			w_k[9] = ControlMath::min(f_e.e_NS,f_edot.edot_PB);
			WO_k[9] = output_level_k*w_k[9];

		}


	}

	if(f_e.Z)
	{
		if(f_edot.NB)
		{
			output_level_k = k_VB;
			w_k[10] = ControlMath::min(f_e.e_Z,f_edot.edot_NB);
			WO_k[10] = output_level_k*w_k[10];

		}

		if(f_edot.NS)
		{
			output_level_k = k_S;
			w_k[11] = ControlMath::min(f_e.e_Z,f_edot.edot_NS);
			WO_k[11] = output_level_k*w_k[11];

		}

		if(f_edot.Z)
		{
			output_level_k = k_M;
			w_k[12] = ControlMath::min(f_e.e_Z,f_edot.edot_Z);
			WO_k[12] = output_level_k*w_k[12];

		}

		if(f_edot.PS)
		{
			output_level_k = k_S;
			w_k[13] = ControlMath::min(f_e.e_Z,f_edot.edot_PS);
			WO_k[13] = output_level_k*w_k[13];

		}

		if(f_edot.PB)
		{
			output_level_k = k_VB;
			w_k[14] = ControlMath::min(f_e.e_Z,f_edot.edot_PB);
			WO_k[14] = output_level_k*w_k[14];

		}

	}

	if(f_e.PS)
	{
		if(f_edot.NB)
		{
			output_level_k = k_S;
			w_k[15] = ControlMath::min(f_e.e_PS,f_edot.edot_NB);
			WO_k[15] = output_level_k*w_k[15];

		}

		if(f_edot.NS)
		{
			output_level_k = k_M;
			w_k[16] = ControlMath::min(f_e.e_PS,f_edot.edot_NS);
			WO_k[16] = output_level_k*w_k[16];

		}

		if(f_edot.Z)
		{
			output_level_k = k_B;
			w_k[17] = ControlMath::min(f_e.e_PS,f_edot.edot_Z);
			WO_k[17] = output_level_k*w_k[17];

		}

		if(f_edot.PS)
		{
			output_level_k = k_M;
			w_k[18] = ControlMath::min(f_e.e_PS,f_edot.edot_PS);
			WO_k[18] = output_level_k*w_k[18];

		}

		if(f_edot.PB)
		{
			output_level_k = k_S;
			w_k[19] = ControlMath::min(f_e.e_PS,f_edot.edot_PB);
			WO_k[19] = output_level_k*w_k[19];

		}
	}

	if(f_e.PB)
	{

		if(f_edot.NB)
		{
			output_level_k = k_M;
			w_k[20] = ControlMath::min(f_e.e_PB,f_edot.edot_NB);
			WO_k[20] = output_level_k*w_k[20];

		}

		if(f_edot.NS)
		{
			output_level_k = k_B;
			w_k[21] = ControlMath::min(f_e.e_PB,f_edot.edot_NS);
			WO_k[21] = output_level_k*w_k[21];

		}

		if(f_edot.Z)
		{
			output_level_k = k_VB;
			w_k[22] = ControlMath::min(f_e.e_PB,f_edot.edot_Z);
			WO_k[22] = output_level_k*w_k[22];

		}

		if(f_edot.PS)
		{
			output_level_k = k_B;
			w_k[23] = ControlMath::min(f_e.e_PB,f_edot.edot_PS);
			WO_k[23] = output_level_k*w_k[23];

		}

		if(f_edot.PB)
		{
			output_level_k = k_M;
			w_k[24] = ControlMath::min(f_e.e_PB,f_edot.edot_PB);
			WO_k[24] = output_level_k*w_k[24];

		}
	}

	for(int i =0; i<24;i++)
	{
		firing_strength_total_k = firing_strength_total_k+w_k[i];
		weighted_output_total_k = weighted_output_total_k+WO_k[i];

	}

	final_output_k = weighted_output_total_k/firing_strength_total_k;

	return final_output_k;

	}
	/////////////////////////FUZZY LOGIC END////////////////////////







	void _positionControl(); ///< Position proportional control
	void _velocityControl(const float dt); ///< Velocity PID control
	void _accelerationControl(); ///< Acceleration setpoint processing

	// Gains
	matrix::Vector3f _gain_pos_p; ///< Position control proportional gain
	matrix::Vector3f _gain_vel_p; ///< Velocity control proportional gain
	matrix::Vector3f _gain_vel_i; ///< Velocity control integral gain
	matrix::Vector3f _gain_vel_d; ///< Velocity control derivative gain

	// Limits
	float _lim_vel_horizontal{}; ///< Horizontal velocity limit with feed forward and position control
	float _lim_vel_up{}; ///< Upwards velocity limit with feed forward and position control
	float _lim_vel_down{}; ///< Downwards velocity limit with feed forward and position control
	float _lim_thr_min{}; ///< Minimum collective thrust allowed as output [-1,0] e.g. -0.9
	float _lim_thr_max{}; ///< Maximum collective thrust allowed as output [-1,0] e.g. -0.1
	float _lim_thr_xy_margin{}; ///< Margin to keep for horizontal control when saturating prioritized vertical thrust
	float _lim_tilt{}; ///< Maximum tilt from level the output attitude is allowed to have

	float _hover_thrust{}; ///< Thrust [0.1, 0.9] with which the vehicle hovers not accelerating down or up with level orientation

	// States
	matrix::Vector3f _pos; /**< current position */
	matrix::Vector3f _vel; /**< current velocity */
	matrix::Vector3f _vel_dot; /**< velocity derivative (replacement for acceleration estimate) */
	matrix::Vector3f _vel_int; /**< integral term of the velocity controller */
	float _yaw{}; /**< current heading */

	// Setpoints
	matrix::Vector3f _pos_sp; /**< desired position */
	matrix::Vector3f _vel_sp; /**< desired velocity */
	matrix::Vector3f _acc_sp; /**< desired acceleration */
	matrix::Vector3f _thr_sp; /**< desired thrust */
	float _yaw_sp{}; /**< desired heading */
	float _yawspeed_sp{}; /** desired yaw-speed */
};
