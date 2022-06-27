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
 * @file RateControl.hpp
 *
 * PID 3 axis angular rate / angular velocity control.
 */

#pragma once
//
#include <matrix/matrix/math.hpp>
#include<mc_att_control/AttitudeControl/AttitudeControl.hpp>
#include <lib/mixer/MultirotorMixer/MultirotorMixer.hpp>
#include <uORB/topics/rate_ctrl_status.h>
#include<mc_pos_control/PositionControl/ControlMath.hpp>
#include<mc_pos_control/PositionControl/PositionControl.hpp>

//px4fmuv5
// #include<iostream>
// #include<fstream>
// #include <chrono>
//#include<string.h>

using namespace std;
const float Zeta = 0.005;


/*fuzzy parameters part


const float A = 20;
const float B = 5;
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

//constant for c:
// const float c_VS = 3.6;
// const float c_S = 3.604;
// const float c_M = 3.608;
// const float c_B = 3.612;
// const float c_VB = 3.615;

// //constant for k:
// const float k_VS = 0.31;
// const float k_S = 0.311;
// const float k_M = 0.313;
// const float k_B = 0.314;
// const float k_VB = 0.315;


*/




class RateControl
{
public:
	RateControl() = default;
	~RateControl() = default;

	/**
	 * Set the rate control gains
	 * @param P 3D vector of proportional gains for body x,y,z axis
	 * @param I 3D vector of integral gains
	 * @param D 3D vector of derivative gains
	 */
	void setGains(const matrix::Vector3f &P, const matrix::Vector3f &I, const matrix::Vector3f &D);

	/**
	 * Set the mximum absolute value of the integrator for all axes
	 * @param integrator_limit limit value for all axes x, y, z
	 */
	void setIntegratorLimit(const matrix::Vector3f &integrator_limit) { _lim_int = integrator_limit; };

	/**
	 * Set direct rate to torque feed forward gain
	 * @see _gain_ff
	 * @param FF 3D vector of feed forward gains for body x,y,z axis
	 */
	void setFeedForwardGain(const matrix::Vector3f &FF) { _gain_ff = FF; };

	/**
	 * Set saturation status
	 * @param control saturation vector from control allocator
	 */
	void setSaturationStatus(const matrix::Vector<bool, 3> &saturation_positive,
				 const matrix::Vector<bool, 3> &saturation_negative);

	/**
	 * Run one control loop cycle calculation
	 * @param rate estimation of the current vehicle angular rate
	 * @param rate_sp desired vehicle angular rate setpoint
	 * @param dt desired vehicle angular rate setpoint
	 * @return [-1,1] normalized torque vector to apply to the vehicle
	 */
	matrix::Vector3f update(const matrix::Vector3f &rate, const matrix::Vector3f &rate_sp,
				const matrix::Vector3f &angular_accel, const float dt, const bool landed);

	/**
	 * Set the integral term to 0 to prevent windup
	 * @see _rate_int
	 */
	void resetIntegral() { _rate_int.zero(); }

	/**
	 * Get status message of controller for logging/debugging
	 * @param rate_ctrl_status status message to fill with internal states
	 */
	void getRateControlStatus(rate_ctrl_status_s &rate_ctrl_status);




	/*/////////////////////////FUZZY LOGIC START////////////////////////
	//-----------------FUZZIFICAITION FOR e--------------------
	// struct fuzzy_e Fuzzification_e(float e)
	// {
	// 	fuzzy_e f_e;

	// 	if(e<=(float)-0.1)
	// 	{
	//  		//e_NB function
	// 		f_e.e_NB = 1;
	// 	}

	// 	else if(e>(float)-0.1&&e<=(float)-0.05)
	// 	{
	// 		//e_NB, e_NS function
	// 		f_e.e_NB = -A*e-1;
	// 		f_e.e_NS = A*e+2;

	// 	}

	// 	else if(e>(float)-0.05&&e<=0)
	// 	{
	// 		//e_NS, e_Z	 function
	// 		f_e.e_NS = -A*e;
	// 		f_e.e_Z = A*e+1;

	// 	}

	// 	else if(e>0&&e<=(float)0.05)
	// 	{
	// 		// e_Z, e_PS function
	// 		f_e.e_Z = -A*e+1;
	// 		f_e.e_PS = A*e;

	// 	}

	// 	else if(e>(float)0.05&&e<=(float)0.1)
	// 	{
	// 		//e_PS, e_PB function
	// 		f_e.e_PS = -A*e+2;
	// 		f_e.e_PB = A*e-1;

	// 	}

	// 	else if(e>(float)0.6)
	// 	{
	// 		//e_PB function
	// 		f_e.e_PB = 1;
	// 	}
	// 	return f_e;
	// }


	// //-----------------FUZZIFICAITION FOR edot--------------------
	// struct fuzzy_edot Fuzzification_edot(float edot)
	// {
	// 	fuzzy_edot f_edot;

	// 	if(edot<=(float)-0.4)
	// 	{
	// 		//e_NB function
	// 		f_edot.edot_NB = 1;

	// 	}

	// 	else if(edot>(float)-0.4&&edot<=(float)-0.2)
	// 	{
	// 		//e_NB, e_NS function

	// 		f_edot.edot_NB = -B*edot-1;
	// 		f_edot.edot_NS = B*edot+2;
	// 	}

	// 	else if(edot>(float)-0.2&&edot<=0)
	// 	{
	// 		//e_NS, e_Z	 function
	// 		f_edot.edot_NS = -B*edot;
	// 		f_edot.edot_Z = B*edot+1;
	// 	}

	// 	else if(edot>0&&edot<=(float)0.2)
	// 	{
	// 		//e_Z, e_PS	 function
	// 		f_edot.edot_Z = -B*edot+1;
	// 		f_edot.edot_PS = B*edot;
	// 	}

	// 	else if(edot>(float)0.2&&edot<=(float)0.4)
	// 	{
	// 		//e_PS, e_PB function
	// 		f_edot.edot_PS = -B*edot+2;
	// 		f_edot.edot_PB = B*edot-1;
	// 	}

	// 	else if(edot>(float)0.4)
	// 	{
	// 		//e_PB	 function
	// 		f_edot.edot_PB = 1;
	// 	}
	// 	return f_edot;
	// }


	// //-----------------CALCULATE FINAL OUTPUT FOR c--------------------
	// float Calculate_final_output_c(fuzzy_e f_e,fuzzy_edot f_edot)
	// {

	// //25 rules of fuzzification to calculate final output c with values of output level corresponding

	// float output_level_c ;
	// float w_c[25];  //firing_strength for c
	// float WO_c[25]; //weighted_output_for_one_rule for c
	// float firing_strength_total_c = 0;
	// float weighted_output_total_c = 0;
	// float final_output_c;


	// if(f_e.NB)
	// {
	// 	if(f_edot.NB)
	// 	{
	// 		output_level_c = c_M;
	// 		w_c[0] = ControlMath::min(f_e.e_NB,f_edot.edot_NB);
	// 		WO_c[0] = output_level_c*w_c[0];

	// 	}


	// 	if(f_edot.NS)
	// 	{
	// 		output_level_c = c_S;
	// 		w_c[1] = ControlMath::min(f_e.e_NB,f_edot.edot_NS);
	// 		WO_c[1] = output_level_c*w_c[1];

	// 	}

	// 	if(f_edot.Z)
	// 	{
	// 		output_level_c = c_VS;
	// 		w_c[2] = ControlMath::min(f_e.e_NB,f_edot.edot_Z);
	// 		WO_c[2] = output_level_c*w_c[2];

	// 	}
	// 	if(f_edot.PS)
	// 	{
	// 		output_level_c = c_S;
	// 		w_c[3] = ControlMath::min(f_e.e_NB,f_edot.edot_PS);
	// 		WO_c[3] = output_level_c*w_c[3];

	// 	}
	// 	if(f_edot.PB)
	// 	{
	// 		output_level_c = c_M;
	// 		w_c[4] = ControlMath::min(f_e.e_NB,f_edot.edot_PB);
	// 		WO_c[4] = output_level_c*w_c[4];

	// 	}
	// }

	// if(f_e.NS)
	// {
	// 	if(f_edot.NB)
	// 	{
	// 		output_level_c = c_B;
	// 		w_c[5] = ControlMath::min(f_e.e_NS,f_edot.edot_NB);
	// 		WO_c[5] = output_level_c*w_c[5];

	// 	}

	// 	if(f_edot.NS)
	// 	{
	// 		output_level_c = c_M;
	// 		w_c[6] = ControlMath::min(f_e.e_NS,f_edot.edot_NS);
	// 		WO_c[6] = output_level_c*w_c[6];

	// 	}

	// 	if(f_edot.Z)
	// 	{
	// 		output_level_c = c_S;
	// 		w_c[7] = ControlMath::min(f_e.e_NS,f_edot.edot_Z);
	// 		WO_c[7] = output_level_c*w_c[7];

	// 	}

	// 	if(f_edot.PS)
	// 	{
	// 		output_level_c = c_M;
	// 		w_c[8] = ControlMath::min(f_e.e_NS,f_edot.edot_PS);
	// 		WO_c[8] = output_level_c*w_c[8];

	// 	}

	// 	if(f_edot.PB)
	// 	{
	// 		output_level_c = c_B;
	// 		w_c[9] = ControlMath::min(f_e.e_NS,f_edot.edot_PB);
	// 		WO_c[9] = output_level_c*w_c[9];

	// 	}
	// }

	// if(f_e.Z)
	// {

	// 	if(f_edot.NB)
	// 	{
	// 		output_level_c = c_VS;
	// 		w_c[10] = ControlMath::min(f_e.e_Z,f_edot.edot_NB);
	// 		WO_c[10] = output_level_c*w_c[10];

	// 	}

	// 	if(f_edot.NS)
	// 	{
	// 		output_level_c = c_B;
	// 		w_c[11] = ControlMath::min(f_e.e_Z,f_edot.edot_NS);
	// 		WO_c[11] = output_level_c*w_c[11];

	// 	}

	// 	if(f_edot.Z)
	// 	{
	// 		output_level_c = c_M;
	// 		w_c[12] = ControlMath::min(f_e.e_Z,f_edot.edot_Z);
	// 		WO_c[12] = output_level_c*w_c[12];

	// 	}

	// 	if(f_edot.PS)
	// 	{
	// 		output_level_c = c_B;
	// 		w_c[13] = ControlMath::min(f_e.e_Z,f_edot.edot_PS);
	// 		WO_c[13] = output_level_c*w_c[13];

	// 	}

	// 	if(f_edot.PB)
	// 	{
	// 		output_level_c = c_VS;
	// 		w_c[14] = ControlMath::min(f_e.e_Z,f_edot.edot_PB);
	// 		WO_c[14] = output_level_c*w_c[14];

	// 	}



	// }

	// if(f_e.PS)
	// {
	// 	if(f_edot.NB)
	// 	{
	// 		output_level_c = c_B;
	// 		w_c[15] = ControlMath::min(f_e.e_PS,f_edot.edot_NB);
	// 		WO_c[15] = output_level_c*w_c[15];

	// 	}

	// 	if(f_edot.NS)
	// 	{
	// 		output_level_c = c_M;
	// 		w_c[16] = ControlMath::min(f_e.e_PS,f_edot.edot_NS);
	// 		WO_c[16] = output_level_c*w_c[16];

	// 	}

	// 	if(f_edot.Z)
	// 	{
	// 		output_level_c = c_S;
	// 		w_c[17] = ControlMath::min(f_e.e_PS,f_edot.edot_Z);
	// 		WO_c[17] = output_level_c*w_c[17];

	// 	}

	// 	if(f_edot.PS)
	// 	{
	// 		output_level_c = c_M;
	// 		w_c[18] = ControlMath::min(f_e.e_PS,f_edot.edot_PS);
	// 		WO_c[18] = output_level_c*w_c[18];

	// 	}

	// 	if(f_edot.PB)
	// 	{
	// 		output_level_c = c_B;
	// 		w_c[19] = ControlMath::min(f_e.e_PS,f_edot.edot_PB);
	// 		WO_c[19] = output_level_c*w_c[19];

	// 	}


	// }

	// if(f_e.PB)
	// {
	// 	if(f_edot.NB)
	// 	{
	// 		output_level_c = c_M;
	// 		w_c[20] = ControlMath::min(f_e.e_PB,f_edot.edot_NB);
	// 		WO_c[20] = output_level_c*w_c[20];

	// 	}

	// 	if(f_edot.NS)
	// 	{
	// 		output_level_c = c_S;
	// 		w_c[21] = ControlMath::min(f_e.e_PB,f_edot.edot_NS);
	// 		WO_c[21] = output_level_c*w_c[21];

	// 	}

	// 	if(f_edot.Z)
	// 	{
	// 		output_level_c = c_VS;
	// 		w_c[22] = ControlMath::min(f_e.e_PB,f_edot.edot_Z);
	// 		WO_c[22] = output_level_c*w_c[22];

	// 	}

	// 	if(f_edot.PS)
	// 	{
	// 		output_level_c = c_S;
	// 		w_c[23] = ControlMath::min(f_e.e_PB,f_edot.edot_PS);
	// 		WO_c[23] = output_level_c*w_c[23];

	// 	}

	// 	if(f_edot.PB)
	// 	{
	// 		output_level_c = c_M;
	// 		w_c[24] = ControlMath::min(f_e.e_PB,f_edot.edot_PB);
	// 		WO_c[24] = output_level_c*w_c[24];

	// 	}
	// }


	// for(int i =0; i<24;i++)
	// {
	// 	firing_strength_total_c = firing_strength_total_c+w_c[i];
	// 	weighted_output_total_c = weighted_output_total_c+WO_c[i];
	// 	//file1<<"No."<<i<<" w and it's weighted output(numerator) are:"<<"w_c:"<<w_c[i]<<",WO_c:"<<WO_c[i]<<endl;

	// }

	// final_output_c = weighted_output_total_c/firing_strength_total_c;

	// return final_output_c;

	// }

	// //-----------------CALCULATE FINAL OUTPUT FOR k--------------------
	// float Calculate_final_output_k(fuzzy_e f_e,fuzzy_edot f_edot)
	// {

	// //25 rules of fuzzification to calculate final output k
	// float output_level_k ;
	// float w_k[25]; //firing_strength for k
	// float WO_k[25]; //weighted_output_for_one_rule for k
	// float firing_strength_total_k = 0;
	// float weighted_output_total_k = 0;

	// float final_output_k;

	// if(f_e.NB)
	// {

	// 	if(f_edot.NB)
	// 	{
	// 		output_level_k = k_M;
	// 		w_k[0] = ControlMath::min(f_e.e_NB,f_edot.edot_NB);
	// 		WO_k[0] = output_level_k*w_k[0];

	// 	}


	// 	if(f_edot.NS)
	// 	{
	// 		output_level_k = k_B;
	// 		w_k[1] = ControlMath::min(f_e.e_NB,f_edot.edot_NS);
	// 		WO_k[1] = output_level_k*w_k[1];

	// 	}



	// 	if(f_edot.Z)
	// 	{
	// 		output_level_k = k_VB;
	// 		w_k[2] = ControlMath::min(f_e.e_NB,f_edot.edot_Z);
	// 		WO_k[2] = output_level_k*w_k[2];

	// 	}
	// 	if(f_edot.PS)
	// 	{
	// 		output_level_k = k_B;
	// 		w_k[3] = ControlMath::min(f_e.e_NB,f_edot.edot_PS);
	// 		WO_k[3] = output_level_k*w_k[3];

	// 	}
	// 	if(f_edot.PB)
	// 	{
	// 		output_level_k = k_M;
	// 		w_k[4] = ControlMath::min(f_e.e_NB,f_edot.edot_PB);
	// 		WO_k[4] = output_level_k*w_k[4];

	// 	}

	// }


	// if(f_e.NS)
	// {

	// 	if(f_edot.NB)
	// 	{
	// 		output_level_k = k_S;
	// 		w_k[5] = ControlMath::min(f_e.e_NS,f_edot.edot_NB);
	// 		WO_k[5] = output_level_k*w_k[5];

	// 	}

	// 	if(f_edot.NS)
	// 	{
	// 		output_level_k = k_M;
	// 		w_k[6] = ControlMath::min(f_e.e_NS,f_edot.edot_NS);
	// 		WO_k[6] = output_level_k*w_k[6];

	// 	}

	// 	if(f_edot.Z)
	// 	{
	// 		output_level_k = k_B;
	// 		w_k[7] = ControlMath::min(f_e.e_NS,f_edot.edot_Z);
	// 		WO_k[7] = output_level_k*w_k[7];

	// 	}

	// 	if(f_edot.PS)
	// 	{
	// 		output_level_k = k_M;
	// 		w_k[8] = ControlMath::min(f_e.e_NS,f_edot.edot_PS);
	// 		WO_k[8] = output_level_k*w_k[8];

	// 	}

	// 	if(f_edot.PB)
	// 	{
	// 		output_level_k = k_S;
	// 		w_k[9] = ControlMath::min(f_e.e_NS,f_edot.edot_PB);
	// 		WO_k[9] = output_level_k*w_k[9];

	// 	}


	// }

	// if(f_e.Z)
	// {
	// 	if(f_edot.NB)
	// 	{
	// 		output_level_k = k_VB;
	// 		w_k[10] = ControlMath::min(f_e.e_Z,f_edot.edot_NB);
	// 		WO_k[10] = output_level_k*w_k[10];

	// 	}

	// 	if(f_edot.NS)
	// 	{
	// 		output_level_k = k_S;
	// 		w_k[11] = ControlMath::min(f_e.e_Z,f_edot.edot_NS);
	// 		WO_k[11] = output_level_k*w_k[11];

	// 	}

	// 	if(f_edot.Z)
	// 	{
	// 		output_level_k = k_M;
	// 		w_k[12] = ControlMath::min(f_e.e_Z,f_edot.edot_Z);
	// 		WO_k[12] = output_level_k*w_k[12];

	// 	}

	// 	if(f_edot.PS)
	// 	{
	// 		output_level_k = k_S;
	// 		w_k[13] = ControlMath::min(f_e.e_Z,f_edot.edot_PS);
	// 		WO_k[13] = output_level_k*w_k[13];

	// 	}

	// 	if(f_edot.PB)
	// 	{
	// 		output_level_k = k_VB;
	// 		w_k[14] = ControlMath::min(f_e.e_Z,f_edot.edot_PB);
	// 		WO_k[14] = output_level_k*w_k[14];

	// 	}

	// }

	// if(f_e.PS)
	// {
	// 	if(f_edot.NB)
	// 	{
	// 		output_level_k = k_S;
	// 		w_k[15] = ControlMath::min(f_e.e_PS,f_edot.edot_NB);
	// 		WO_k[15] = output_level_k*w_k[15];

	// 	}

	// 	if(f_edot.NS)
	// 	{
	// 		output_level_k = k_M;
	// 		w_k[16] = ControlMath::min(f_e.e_PS,f_edot.edot_NS);
	// 		WO_k[16] = output_level_k*w_k[16];

	// 	}

	// 	if(f_edot.Z)
	// 	{
	// 		output_level_k = k_B;
	// 		w_k[17] = ControlMath::min(f_e.e_PS,f_edot.edot_Z);
	// 		WO_k[17] = output_level_k*w_k[17];

	// 	}

	// 	if(f_edot.PS)
	// 	{
	// 		output_level_k = k_M;
	// 		w_k[18] = ControlMath::min(f_e.e_PS,f_edot.edot_PS);
	// 		WO_k[18] = output_level_k*w_k[18];

	// 	}

	// 	if(f_edot.PB)
	// 	{
	// 		output_level_k = k_S;
	// 		w_k[19] = ControlMath::min(f_e.e_PS,f_edot.edot_PB);
	// 		WO_k[19] = output_level_k*w_k[19];

	// 	}
	// }

	// if(f_e.PB)
	// {

	// 	if(f_edot.NB)
	// 	{
	// 		output_level_k = k_M;
	// 		w_k[20] = ControlMath::min(f_e.e_PB,f_edot.edot_NB);
	// 		WO_k[20] = output_level_k*w_k[20];

	// 	}

	// 	if(f_edot.NS)
	// 	{
	// 		output_level_k = k_B;
	// 		w_k[21] = ControlMath::min(f_e.e_PB,f_edot.edot_NS);
	// 		WO_k[21] = output_level_k*w_k[21];

	// 	}

	// 	if(f_edot.Z)
	// 	{
	// 		output_level_k = k_VB;
	// 		w_k[22] = ControlMath::min(f_e.e_PB,f_edot.edot_Z);
	// 		WO_k[22] = output_level_k*w_k[22];

	// 	}

	// 	if(f_edot.PS)
	// 	{
	// 		output_level_k = k_B;
	// 		w_k[23] = ControlMath::min(f_e.e_PB,f_edot.edot_PS);
	// 		WO_k[23] = output_level_k*w_k[23];

	// 	}

	// 	if(f_edot.PB)
	// 	{
	// 		output_level_k = k_M;
	// 		w_k[24] = ControlMath::min(f_e.e_PB,f_edot.edot_PB);
	// 		WO_k[24] = output_level_k*w_k[24];

	// 	}
	// }

	// for(int i =0; i<24;i++)
	// {
	// 	firing_strength_total_k = firing_strength_total_k+w_k[i];
	// 	weighted_output_total_k = weighted_output_total_k+WO_k[i];

	// }

	// final_output_k = weighted_output_total_k/firing_strength_total_k;

	// return final_output_k;

	// }
	/////////////////////////FUZZY LOGIC END////////////////////////*/



private:
	//void updateIntegral(matrix::Vector3f &rate_error, const float dt);
	void updateIntegral_Ver_Euler(matrix::Vector3f rad_err, const float dt);

	// Gains
	matrix::Vector3f _gain_p; ///< rate control proportional gain for all axes x, y, z
	matrix::Vector3f _gain_i; ///< rate control integral gain
	matrix::Vector3f _gain_d; ///< rate control derivative gain
	matrix::Vector3f _lim_int; ///< integrator term maximum absolute value
	matrix::Vector3f _gain_ff; ///< direct rate to torque feed forward gain only useful for helicopters

	// States
	matrix::Vector3f _rate_int; ///< integral term of the rate controller
	matrix::Vector3f rad_error_int; ///< integral term of the PID euler version


	// Feedback from control allocation
	matrix::Vector<bool, 3> _control_allocator_saturation_negative;
	matrix::Vector<bool, 3> _control_allocator_saturation_positive;




};
