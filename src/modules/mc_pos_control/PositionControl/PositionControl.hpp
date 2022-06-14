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
#define F_PATH_RBFNN "/home/tang/Desktop/NeuralNetwork_output.txt"
#define F_PATH_FUZZY_VALUE_C "/home/tang/Desktop/final_value_c.txt"
#define F_PATH_FUZZY_VALUE_K "/home/tang/Desktop/final_value_k.txt"


#include <iostream> 
#include <lib/mathlib/mathlib.h>
#include <float.h>
#include <matrix/matrix/math.hpp>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>
#include <fstream>
#include <iostream>
#include "ControlMath.hpp"

using namespace matrix;
using namespace std;

const float etha = 2;
const int n = 10;   
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


const float zeta = 0.5;



struct NeuralNetwork
{
	Matrix<float, n, 3> W_hat_dot;
	Matrix<float, n, 1> Sz;
	Matrix<float, n, 3> W_hat;
	Matrix<float, 3, 1> f_x;
};


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
		_gain_pos_p = P;
		//_gain_pos_p = Vector3f(1.0,1.0,1.0);
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
	Vector3f SMC[5];
	/////////////////////////SLIDING MODE CONTROL////////////////////////
	Vector3f * _SMCController(float cx,float cy,float cz,float kx,float ky, float kz)
	{
		//constante de perturbation c,k
		Vector3f smc_c = Vector3f(cx, cy, cz);
		Vector3f smc_k = Vector3f(kx, ky, kz);	
		SMC[0] = smc_c;
		SMC[1] = smc_k;
		Vector3f e = _pos_sp - _pos;  // error of position
		SMC[2] = e;
		Vector3f e_dot = _vel_sp - _vel; //error of velocity
		SMC[3] = e_dot;

		//calculate about S
		Vector3f S =smc_c.emult(e) + e_dot;   //Surface S for SMC
		// 0Saturate S
		for (int i=0; i<3; i++) 
		{
			if (!(-1 < S(i) && S(i) < 1 ))
				S(i) = (S(i)>=0)?1:-1;
		}
		SMC[4] = S;	
		return SMC;
}
	/////////////////////////SLIDING MODE CONTROL END////////////////////////


	/////////////////////////NEURAL NETWORK RBF START////////////////////////

	Matrix<float, 3, 1>_NeuralNetwork(Vector3f pos, Vector3f pos_sp, Vector3f vel, Vector3f vel_sp, Vector3f S, const float dt)
	{
		int i = 0;
		NeuralNetwork nn;  // neural network structure definition
		Vector3f e =  pos_sp-pos;   //error position
		Vector3f e_dot =  vel_sp-vel; //error velocity
		float mu_x[] = {-0.5000,-0.3889,-0.2778,-0.1667,-0.0556,0.0556,0.1667,0.2778,0.3889,0.5000};
		float mu_y[] = {-0.5000,-0.3889,-0.2778,-0.1667,-0.0556,0.0556,0.1667,0.2778,0.3889,0.5000};
		float mu_z[] = {-0.7000,-0.5444,-0.3889,-0.2333,-0.0778,0.0778,0.2333,0.3889,0.5444,0.7000}; 
		float Z[] = {e(0), e(1), e(2), e_dot(0), e_dot(1), e_dot(2)};  // Z matrix 
		Matrix<float, n, 3> W_hat_dot; //
		Matrix<float, n, 1> Sz; 
		Matrix<float, 3, 1> f_x;
		float _W_hat_copy[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; //initialization for W_hat matrix

		Matrix<float, n, n> Kw;


		Kw.setIdentity();

		//calculate matrix Sz(i) and assign to Sz
		for(i = 0;i<n;i++)	
		{
		
			Sz(i,0)= exp(-(pow((Z[0]-mu_x[i]),2)+ pow((Z[1]-mu_y[i]),2)+ pow((Z[2]-mu_z[i]),2)+ pow((Z[3]-mu_x[i]),2)+ pow((Z[4]-mu_x[i]),2)+ pow((Z[5]-mu_x[i]),2))/pow(etha,2));

		}

		W_hat_dot = -Kw*Sz*S.transpose(); // calculate W_hat_dot
		Matrix<float, n, 3> W_hat(_W_hat_copy);
		W_hat += W_hat_dot*dt; 
		W_hat.copyTo(_W_hat_copy);
		f_x = W_hat.transpose()*Sz;  // calculate f_x: 3x1 matrix ,  output of NN 
		
		if(setNN(&nn,W_hat_dot,Sz,W_hat,f_x) ==-1)
			printf("Error of NN variable!\n");
		else   // write the f_x value into a txt file to observer its value
		{
			if (!isnan(_pos_sp(0)) && !isnan(_pos_sp(1)))  // if position value x, y is not NAN
			{
			std::ofstream file; 
			file.open(F_PATH_RBFNN,std::ios::app);  // write f_x value into file NEURAL_NETWORK_output.txt
			file<<"The current f_x matrix is \n"<<(double)nn.f_x(0,0)<<"\n"<<(double)nn.f_x(1,0)<<"\n"<<nn.f_x(2,0)<<"\n"<<endl;
			file.close();
			}
		}
		return f_x;
	}



	int setNN(NeuralNetwork *nn,Matrix<float, n, 3> a, Matrix<float, n, 1> b, Matrix<float, n, 3> c, Matrix<float, 3, 1> d)
	{
		if(!nn)
			return -1;
		nn->W_hat_dot = a;
		nn->Sz = b;
		nn->W_hat = c;
		nn->f_x = d;
		return 0;
	}

	/////////////////////////NEURAL NETWORK RBF END////////////////////////



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
