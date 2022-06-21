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

#include<iostream>
using namespace std;

const float etha = 2;
const int n = 10;
const float zeta = 0.5;

struct NeuralNetwork
{
	Matrix<float, n, 3> W_hat_dot;
	Matrix<float, n, 1> Sz;
	Matrix<float, n, 3> W_hat;
	Matrix<float, 3, 1> f_x;
};


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

private:
	void updateIntegral(matrix::Vector3f &rate_error, const float dt);

	// Gains
	matrix::Vector3f _gain_p; ///< rate control proportional gain for all axes x, y, z
	matrix::Vector3f _gain_i; ///< rate control integral gain
	matrix::Vector3f _gain_d; ///< rate control derivative gain
	matrix::Vector3f _lim_int; ///< integrator term maximum absolute value
	matrix::Vector3f _gain_ff; ///< direct rate to torque feed forward gain only useful for helicopters

	// States
	matrix::Vector3f _rate_int; ///< integral term of the rate controller

	// Feedback from control allocation
	matrix::Vector<bool, 3> _control_allocator_saturation_negative;
	matrix::Vector<bool, 3> _control_allocator_saturation_positive;



	/////////////////////////NEURAL NETWORK RBF START////////////////////////

	Matrix<float, 3, 1>_NeuralNetwork(Vector3f ang_err, Vector3f rate_err, Vector3f S, const float dt)
	{
		int i = 0;
		NeuralNetwork nn;  // neural network structure definition
		Vector3f e =  ang_err;   //angular error
		Vector3f e_dot =  rate_err; //rate error
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
};
