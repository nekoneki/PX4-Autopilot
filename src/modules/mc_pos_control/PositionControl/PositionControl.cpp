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
 * @file PositionControl.cpp
 */

#include "PositionControl.hpp"
#include <float.h>
#include <mathlib/mathlib.h>
#include <px4_platform_common/defines.h>
#include <geo/geo.h>

//#include<iostream>
//px4fmuv5
// string file_path;
//double timed = std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();


//PIDNN CONTROLLER SWITCH
/*
IF WANNA PIDNN SET 1,1 AND SET ALL SMCNN PART TO 0,0,0
*/
#define RBFNN_PID 1
#define PIDNN_CONTROLLER 1


//SMCNN(+FUZZY) SWITCH
/*
IF WANNA SMCNN ONY , SET 1,1,0 AND SET ALL PIDNN PART TO 0,0,0
IF WANNA SMCNN FUZZY, SET 1,1,1 AND SET ALL PIDNN PART TO 0,0,0
*/
#define SMCNN_CONTROLLER 0
#define RBFNN_SMC 0
#define FUZZY_SMC 0


using namespace matrix;
using namespace std;
struct fuzzy_e;
struct fuzzy_edot;
int flag_file;
int best;   // let mc_rate_control has best smc value while we are having smc_nn_fuzzy in mc_pos_control

const float Mat_B[] ={0,0,0,1,1,1,0,0,0,1,1,1,0,0,0,1,1,1};
const float mu_xy[] = {-0.5000,-0.3889,-0.2778,-0.1667,-0.0556,0.0556,0.1667,0.2778,0.3889,0.5000};
const float mu_z[] = {-0.7000,-0.5444,-0.3889,-0.2333,-0.0778,0.0778,0.2333,0.3889,0.5444,0.7000};
const float P_mat[] = {3.0497,0,0,0.7483,0,0,0,3.0497,0,0,0.7483,0,0,0,0,0,0,0,0.7483,0,0,1.5,0,0,0,0.7483,0,0,1.5,0,0,0,0,0,0,0};
const Matrix<float, 6, 3> Matrix_B(Mat_B);  // matrix B
const Matrix<float, 6, 6> P_matrix(P_mat);   // 6x6 symetric matrix

float kw[100] = {
	1,0,0,0,0,0,0,0,0,0,
	0,1,0,0,0,0,0,0,0,0,
	0,0,1,0,0,0,0,0,0,0,
	0,0,0,1,0,0,0,0,0,0,
	0,0,0,0,1,0,0,0,0,0,
	0,0,0,0,0,1,0,0,0,0,
	0,0,0,0,0,0,1,0,0,0,
	0,0,0,0,0,0,0,1,0,0,
	0,0,0,0,0,0,0,0,1,0,
	0,0,0,0,0,0,0,0,0,1};  //400 Byte = 0.39kb

const Matrix<float, 10, 10> Kw(kw);

float what_copy_initialize[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; //initialization for W_hat matrix



//for PIDNN
Matrix<float, 10, 3> W_hat_dot;  //matrix W_hat_dot
Matrix<float, 3, 1> f_x;        //matrix f_x output NN

//for SMCNN
Matrix<float, 10, 3> W_hat_dot1; //
Matrix<float, 3, 1> f_x1;


// SMC parameters
Vector3f smc_c;
Vector3f smc_k;

//PID parameters
//best parameter for pid-NN K1(P) = 1.89 K2(D) = 2.7
const float K1 = 4.1;
const float K2 = 3.9;

//SMC parameters
const float c = 1.9;
const float k = 2.85;
Vector3f S;

//RBFNN parameters
const float etha = 2;
const float zeta = 0.5;


//FUZZY PARAMETERS
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



void PositionControl::setVelocityGains(const Vector3f &P, const Vector3f &I, const Vector3f &D)
{
	_gain_vel_p = Vector3f(K1,K1,K1);
	_gain_vel_i = I;
	_gain_vel_d = Vector3f(K2,K2,K2);

}

void PositionControl::setVelocityLimits(const float vel_horizontal, const float vel_up, const float vel_down)
{
	_lim_vel_horizontal = vel_horizontal;
	_lim_vel_up = vel_up;
	_lim_vel_down = vel_down;
}

void PositionControl::setThrustLimits(const float min, const float max)
{
	// make sure there's always enough thrust vector length to infer the attitude
	_lim_thr_min = math::max(min, 10e-4f);
	_lim_thr_max = max;
}

void PositionControl::setHorizontalThrustMargin(const float margin)
{
	_lim_thr_xy_margin = margin;
}

void PositionControl::updateHoverThrust(const float hover_thrust_new)
{
	// Given that the equation for thrust is T = a_sp * Th / g - Th
	// with a_sp = desired acceleration, Th = hover thrust and g = gravity constant,
	// we want to find the acceleration that needs to be added to the integrator in order obtain
	// the same thrust after replacing the current hover thrust by the new one.
	// T' = T => a_sp' * Th' / g - Th' = a_sp * Th / g - Th
	// so a_sp' = (a_sp - g) * Th / Th' + g
	// we can then add a_sp' - a_sp to the current integrator to absorb the effect of changing Th by Th'
	if (hover_thrust_new > FLT_EPSILON) {
		_vel_int(2) += (_acc_sp(2) - CONSTANTS_ONE_G) * _hover_thrust / hover_thrust_new + CONSTANTS_ONE_G - _acc_sp(2);
		setHoverThrust(hover_thrust_new);
	}
}

void PositionControl::setState(const PositionControlStates &states)
{
	_pos = states.position;
	_vel = states.velocity;
	_yaw = states.yaw;
	_vel_dot = states.acceleration;
}

void PositionControl::setInputSetpoint(const vehicle_local_position_setpoint_s &setpoint)
{
	_pos_sp = Vector3f(setpoint.x, setpoint.y, setpoint.z);
	_vel_sp = Vector3f(setpoint.vx, setpoint.vy, setpoint.vz);
	_acc_sp = Vector3f(setpoint.acceleration);
	_yaw_sp = setpoint.yaw;
	_yawspeed_sp = setpoint.yawspeed;
}

bool PositionControl::update(const float dt)
{
	bool valid = _inputValid();

	if (valid) {
		_positionControl();
		_velocityControl(dt);
		_yawspeed_sp = PX4_ISFINITE(_yawspeed_sp) ? _yawspeed_sp : 0.f;
		_yaw_sp = PX4_ISFINITE(_yaw_sp) ? _yaw_sp : _yaw; // TODO: better way to disable yaw control
	}

	// There has to be a valid output accleration and thrust setpoint otherwise something went wrong
	valid = valid && PX4_ISFINITE(_acc_sp(0)) && PX4_ISFINITE(_acc_sp(1)) && PX4_ISFINITE(_acc_sp(2));
	valid = valid && PX4_ISFINITE(_thr_sp(0)) && PX4_ISFINITE(_thr_sp(1)) && PX4_ISFINITE(_thr_sp(2));

	return valid;
}




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



















void PositionControl::_positionControl()
{

	// P-position controller
	Vector3f vel_sp_position = (_pos_sp - _pos).emult(_gain_pos_p);
	ControlMath::addIfNotNanVector3f(_vel_sp, vel_sp_position);
	ControlMath::setZeroIfNanVector3f(vel_sp_position);	// make sure there are no NAN elements for further reference while constraining

	_vel_sp.xy() = ControlMath::constrainXY(vel_sp_position.xy(), (_vel_sp - vel_sp_position).xy(), 	_lim_vel_horizontal);

	_vel_sp(2) = math::constrain(_vel_sp(2), -_lim_vel_up, _lim_vel_down);
}




void PositionControl::_velocityControl(const float dt)
{
	Vector3f pos_error = _pos_sp - _pos;
	Vector3f vel_error = _vel_sp - _vel;
	Vector3f acc_sp_velocity;
	Matrix<float, 10, 1> Hx;         //matrix Hx


	float E_mat[] = {pos_error(0), pos_error(1), pos_error(2), vel_error(0), vel_error(1), vel_error(2)};
	//calculate matrix Hx(i) and assign to Hx
	for(int i = 0;i<10;i++)
	{
		Hx(i,0)= exp(-(pow((E_mat[0]-mu_xy[i]),2)+ pow((E_mat[1]-mu_xy[i]),2)+ pow((E_mat[2]-mu_z[i]),2)+ pow((E_mat[3]-mu_xy[i]),2)+ pow((E_mat[4]-mu_xy[i]),2)+ pow((E_mat[5]-mu_z[i]),2))/pow(etha,2));
		//Hx(i,0)= exp(-(pow((E(0,0)-mu_xy[i]),2)+ pow((E(0,1)-mu_xy[i]),2)+ pow((E(0,2)-mu_z[i]),2)+ pow((E(0,3)-mu_xy[i]),2)+ pow((E(0,4)-mu_xy[i]),2)+ pow((E(0,5)-mu_z[i]),2))/pow(etha,2));

	}



	//PIDNN PART
	#if PIDNN_CONTROLLER
		#if RBFNN_PID
			// wait for test .
			//PX4_INFO("LAUNCH POS_CONTROL PID_NN");
			if (!isnan(_pos_sp(0)) && !isnan(_pos_sp(1)))
			{
				flag_file = 1;
				//E matrix must keep at here !!
				Matrix<float, 6, 1> E(E_mat);  //  matrix E_mat

				//W_hat_copy must keep at here !!!
				float _W_hat_copy[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; //initialization for W_hat matrix


				W_hat_dot = -Hx*E.transpose()*P_matrix*Matrix_B; // calculate W_hat_dot    matrix 10x3
				Matrix<float, 10, 3> W_hat(_W_hat_copy);    // matrix 10x3
				W_hat += W_hat_dot*dt;
				W_hat.copyTo(_W_hat_copy);
				f_x = W_hat.transpose()*Hx;  // calculate f_x: (3x10)(10x1)= 3x1 matrix ,  output of NN , estimate error of controller to x,y,z acceleration_setpoint_velocity

				//PIDNN control law:
				acc_sp_velocity = pos_error.emult(_gain_vel_p)+vel_error.emult(_gain_vel_d)-f_x;
			}

		#else
		acc_sp_velocity = pos_error.emult(_gain_vel_p)+vel_error.emult(_gain_vel_d);
		//PX4_WARN("RBFNN_PID WAS NOT DEFINED AS TRUE!");

		#endif

	#endif



	//SMCNN (+ FUZZY) PART
	#if SMCNN_CONTROLLER
		//SMC +FUZZY PART
		#if FUZZY_SMC

			//PX4_INFO("LAUNCH POS_CONTROL SMC_NN_FUZZY");
			best = 1;
			flag_file = 3;
			fuzzy_e fuzzy_e_x;
			fuzzy_edot fuzzy_edot_x;

			fuzzy_e fuzzy_e_y;
			fuzzy_edot fuzzy_edot_y;

			fuzzy_e fuzzy_e_z;
			fuzzy_edot fuzzy_edot_z;

			fuzzy_e_x = Fuzzification_e(pos_error(0));
			fuzzy_edot_x = Fuzzification_edot(vel_error(0));
			fuzzy_e_y = Fuzzification_e(pos_error(1));
			fuzzy_edot_y = Fuzzification_edot(vel_error(1));
			fuzzy_e_z = Fuzzification_e(pos_error(2));
			fuzzy_edot_z = Fuzzification_edot(vel_error(2));

			//Calculate cx,cy,cz for c
			float final_value_cx = Calculate_final_output_c(fuzzy_e_x,fuzzy_edot_x);
			float final_value_cy = Calculate_final_output_c(fuzzy_e_y,fuzzy_edot_y);
			float final_value_cz = Calculate_final_output_c(fuzzy_e_z,fuzzy_edot_z);

			//Calculate cx,cy,cz for k
			float final_value_kx = Calculate_final_output_k(fuzzy_e_x,fuzzy_edot_x);
			float final_value_ky = Calculate_final_output_k(fuzzy_e_y,fuzzy_edot_y);
			float final_value_kz = Calculate_final_output_k(fuzzy_e_z,fuzzy_edot_z);

			smc_c = Vector3f(final_value_cx,final_value_cy,final_value_cz);
			smc_k = Vector3f(final_value_kx,final_value_ky,final_value_kz);
			S =smc_c.emult(pos_error) + vel_error;   //Surface S for SMC




		#else
			//ONLY SMC PART
			//PX4_INFO("LAUNCH POS_CONTROL SMC_NN");
			flag_file = 2;
			smc_c = Vector3f(c,c,c);
			smc_k = Vector3f(k,k,k);
			S =smc_c.emult(pos_error) + vel_error;   //Surface S for SMC
		#endif

		#if RBFNN_SMC
			if(!isnan(_pos_sp(0)) && !isnan(_pos_sp(1)))
			{


				float _W_hat_copy1[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; //initialization for W_hat matrix

				W_hat_dot1= -Kw*Hx*S.transpose(); // calculate W_hat_dot
				Matrix<float, 10, 3> W_hat1(_W_hat_copy1);
				W_hat1 += W_hat_dot1*dt;
				W_hat1.copyTo(_W_hat_copy1);
				f_x1 = W_hat1.transpose()*Hx;  // calculate f_x: 3x1 matrix ,  output of NN
				acc_sp_velocity = S.emult(smc_k)+zeta*ControlMath::sign(S)+f_x1;
			}
		#else
		//acc_sp_velocity = S.emult(smc_k)+zeta*ControlMath::sign(S);
		//PX4_WARN("RBFNN_SMC WAS NOT DEFINED AS TRUE!");

		#endif
	#endif


	//WRITE output into file txt
	//px4fmuv5
	// if(flag_file  == 1) file_path ="/home/tang/Desktop/PIDNN_POS_CTL.txt";
	// else if(flag_file  == 2) file_path ="/home/tang/Desktop/SMCNN_POS_CTL.txt";
	// else if(flag_file  == 3) file_path ="/home/tang/Desktop/SMCNNFUZZY_POS_CTL.txt";

	// std::ofstream file;
	// double time1 = std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
	// file.open(file_path,std::ios::app);  // write output
	// file<<_pos(0)<<" "<<_pos_sp(0)<<" "<<pos_error(0)<<" "<<_pos(1)<<" "<<_pos_sp(1)<<" "<<pos_error(1)<<" "<<_pos(2)<<" "<<_pos_sp(2)<<" "<<pos_error(2)<<" "<<(double)(time1-timed)<<endl;
	// file.close();



	// FILE *fichier = fopen(file_path,"a");

	// 	double timeA = std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
	// 	fprintf(fichier,"%f\t%f\t\n", (double)_pos(0),(double)(timeA-timed));
	// fclose(fichier);



	// if(flag_file  == 2)
	// {
	// 	char path1[] = "PID_ATT_CTL.txt";
	// 	FILE *fichier = fopen(path1,"a");

	// 	fprintf(fichier,"%f \n", (double)pos_error(0));
	// 	fclose(fichier);
	// }

	/*
	// write final value of c(x,y,z)into file final_value_c.txt
	std::ofstream file1;
	file1.open(F_PATH_FUZZY_VALUE_C,std::ios::app);
	file1<<"Final value for c is:"<<final_value_cx<<","<<final_value_cy<<","<<final_value_cz<<endl;
	file1.close();

	// write final value of k(x,y,z)into file final_value_k.txt
	std::ofstream file2;
	file2.open(F_PATH_FUZZY_VALUE_K,std::ios::app);
	file2<<"Final value for k is:"<<final_value_kx<<","<<final_value_ky<<","<<final_value_kz<<endl;
	file2.close();
	*/



	/*
	// wind noise adding
	Vector3f noise(rand()%6+1, rand()%6+1, rand()%6+1);
	Vector3f wind(sin(noise(0)), sin(noise(1)), sin(noise(2)));
	cout<<"noise0 is"<<noise(0)<<"noise1 is "<<noise(1)<<"noise2 is "<<noise(2)<<"\n"<<endl;
	noise *= 1.3;
	noise -= 0.7;
	noise *= 0.08;
	Vector3f Total_noise = wind + noise;
	*/

	// old version for smc with artificial noise
	//Vector3f acc_sp_velocity = smc[3].emult(smc[0]) + smc[4].emult(smc[1]);//+Total_noise +f_x_result;
	//refers:  acc_sp_velocity = e_dot .emult(k)      +   S   .emult(c2)   //+Total_noise;




	// No control input from setpoints or corresponding states which are NAN
	ControlMath::addIfNotNanVector3f(_acc_sp, acc_sp_velocity);
	_accelerationControl();

	// Integrator anti-windup in vertical direction
	if ((_thr_sp(2) >= -_lim_thr_min && vel_error(2) >= 0.0f) ||
	    (_thr_sp(2) <= -_lim_thr_max && vel_error(2) <= 0.0f)) {
		vel_error(2) = 0.f;
	}

	// Prioritize vertical control while keeping a horizontal margin
	const Vector2f thrust_sp_xy(_thr_sp);
	const float thrust_sp_xy_norm = thrust_sp_xy.norm();
	const float thrust_max_squared = math::sq(_lim_thr_max);

	// Determine how much vertical thrust is left keeping horizontal margin
	const float allocated_horizontal_thrust = math::min(thrust_sp_xy_norm, _lim_thr_xy_margin);
	const float thrust_z_max_squared = thrust_max_squared - math::sq(allocated_horizontal_thrust);

	// Saturate maximal vertical thrust
	_thr_sp(2) = math::max(_thr_sp(2), -sqrtf(thrust_z_max_squared));

	// Determine how much horizontal thrust is left after prioritizing vertical control
	const float thrust_max_xy_squared = thrust_max_squared - math::sq(_thr_sp(2));
	float thrust_max_xy = 0;

	if (thrust_max_xy_squared > 0) {
		thrust_max_xy = sqrtf(thrust_max_xy_squared);
	}

	// Saturate thrust in horizontal direction
	if (thrust_sp_xy_norm > thrust_max_xy) {
		_thr_sp.xy() = thrust_sp_xy / thrust_sp_xy_norm * thrust_max_xy;
	}

	// Use tracking Anti-Windup for horizontal direction: during saturation, the integrator is used to unsaturate the output
	// see Anti-Reset Windup for PID controllers, L.Rundqwist, 1990
	const Vector2f acc_sp_xy_limited = Vector2f(_thr_sp) * (CONSTANTS_ONE_G / _hover_thrust);
	const float arw_gain = 2.f / _gain_vel_p(0);
	vel_error.xy() = Vector2f(vel_error) - (arw_gain * (Vector2f(_acc_sp) - acc_sp_xy_limited));

	// Make sure integral doesn't get NAN
	ControlMath::setZeroIfNanVector3f(vel_error);
	// Update integral part of velocity control
	_vel_int += vel_error.emult(_gain_vel_i) * dt;

	// limit thrust integral
	_vel_int(2) = math::min(fabsf(_vel_int(2)), CONSTANTS_ONE_G) * sign(_vel_int(2));
}

void PositionControl::_accelerationControl()
{
	// Assume standard acceleration due to gravity in vertical direction for attitude generation
	Vector3f body_z = Vector3f(-_acc_sp(0), -_acc_sp(1), CONSTANTS_ONE_G).normalized();
	ControlMath::limitTilt(body_z, Vector3f(0, 0, 1), _lim_tilt);
	// Scale thrust assuming hover thrust produces standard gravity
	float collective_thrust = _acc_sp(2) * (_hover_thrust / CONSTANTS_ONE_G) - _hover_thrust;
	// Project thrust to planned body attitude
	collective_thrust /= (Vector3f(0, 0, 1).dot(body_z));
	collective_thrust = math::min(collective_thrust, -_lim_thr_min);
	_thr_sp = body_z * collective_thrust;
}

bool PositionControl::_inputValid()
{
	bool valid = true;

	// Every axis x, y, z needs to have some setpoint
	for (int i = 0; i <= 2; i++) {
		valid = valid && (PX4_ISFINITE(_pos_sp(i)) || PX4_ISFINITE(_vel_sp(i)) || PX4_ISFINITE(_acc_sp(i)));
	}

	// x and y input setpoints always have to come in pairs
	valid = valid && (PX4_ISFINITE(_pos_sp(0)) == PX4_ISFINITE(_pos_sp(1)));
	valid = valid && (PX4_ISFINITE(_vel_sp(0)) == PX4_ISFINITE(_vel_sp(1)));
	valid = valid && (PX4_ISFINITE(_acc_sp(0)) == PX4_ISFINITE(_acc_sp(1)));

	// For each controlled state the estimate has to be valid
	for (int i = 0; i <= 2; i++) {
		if (PX4_ISFINITE(_pos_sp(i))) {
			valid = valid && PX4_ISFINITE(_pos(i));
		}

		if (PX4_ISFINITE(_vel_sp(i))) {
			valid = valid && PX4_ISFINITE(_vel(i)) && PX4_ISFINITE(_vel_dot(i));
		}
	}

	return valid;
}

void PositionControl::getLocalPositionSetpoint(vehicle_local_position_setpoint_s &local_position_setpoint) const
{
	local_position_setpoint.x = _pos_sp(0);
	local_position_setpoint.y = _pos_sp(1);
	local_position_setpoint.z = _pos_sp(2);
	local_position_setpoint.yaw = _yaw_sp;
	local_position_setpoint.yawspeed = _yawspeed_sp;
	local_position_setpoint.vx = _vel_sp(0);
	local_position_setpoint.vy = _vel_sp(1);
	local_position_setpoint.vz = _vel_sp(2);
	_acc_sp.copyTo(local_position_setpoint.acceleration);
	_thr_sp.copyTo(local_position_setpoint.thrust);
}

void PositionControl::getAttitudeSetpoint(vehicle_attitude_setpoint_s &attitude_setpoint) const
{
	ControlMath::thrustToAttitude(_thr_sp, _yaw_sp, attitude_setpoint);
	attitude_setpoint.yaw_sp_move_rate = _yawspeed_sp;
}
