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
using namespace matrix;


//px4fmuv5
//string File_path;


Vector3f torque;
int Flag_file;

//PID EULER VERSION CONTROLLER SWITCH  0 IS CLOSE.
#define PID_VER_EULER_CONTROLLER 0

//SMC CONTROLLER SWITCH  0 IS CLOSE.
#define SMC_CONTROLLER 1


//double timed0 = std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
//struct fuzzy_e;
//struct fuzzy_edot;

//SMC parameters   best value:  c = 3.6   k = 0.315    bad 4.2,  0.36
float c_ = 4.2;
float k_ = 0.36;
float c_best = 3.6;
float k_best = 0.315;
Vector3f c_smc;
Vector3f k_smc;


void RateControl::setGains(const Vector3f &P, const Vector3f &I, const Vector3f &D)
{

	/*origin version PID gain
	//  _gain_p = P;
	//  _gain_i = I;
	//  _gain_d = D;
	*/

	//PID parameters part:
	//Euler PID version with angle error  ok !
	const float _p = 0.6;
	const float _i = 0.3;
	const float _d = 0.4;

	_gain_p = Vector3f(_p,_p,_p);
	_gain_i = Vector3f(_i,_i,_i);;
	_gain_d = Vector3f(_d,_d,_d);

}




void RateControl::setSaturationStatus(const Vector<bool, 3> &saturation_positive, const Vector<bool, 3> &saturation_negative)
{
	_control_allocator_saturation_positive = saturation_positive;
	_control_allocator_saturation_negative = saturation_negative;
}

Vector3f RateControl::update(const Vector3f &rate, const Vector3f &rate_sp, const Vector3f &angular_accel,
			     const float dt, const bool landed)
{
	Vector3f rate_error = rate_sp - rate;	// angular rates error
	Vector3f rad_error = euler_rad_sp - euler_rad_current; // angle error in radian

	/*FUZZY PART if need
	// fuzzy_e fuzzy_e_x;
	// fuzzy_edot fuzzy_edot_x;

	// fuzzy_e fuzzy_e_y;
	// fuzzy_edot fuzzy_edot_y;

	// fuzzy_e fuzzy_e_z;
	// fuzzy_edot fuzzy_edot_z;

	// fuzzy_e_x = Fuzzification_e(rate_error(0));
	// fuzzy_edot_x = Fuzzification_edot(rad_error(0));
	// fuzzy_e_y = Fuzzification_e(rate_error(1));
	// fuzzy_edot_y = Fuzzification_edot(rad_error(1));
	// fuzzy_e_z = Fuzzification_e(rate_error(2));
	// fuzzy_edot_z = Fuzzification_edot(rad_error(2));

	// //Calculate cx,cy,cz for c
	// float final_value_cx = Calculate_final_output_c(fuzzy_e_x,fuzzy_edot_x);
	// float final_value_cy = Calculate_final_output_c(fuzzy_e_y,fuzzy_edot_y);
	// float final_value_cz = Calculate_final_output_c(fuzzy_e_z,fuzzy_edot_z);

	// //Calculate cx,cy,cz for k
	// float final_value_kx = Calculate_final_output_k(fuzzy_e_x,fuzzy_edot_x);
	// float final_value_ky = Calculate_final_output_k(fuzzy_e_y,fuzzy_edot_y);
	// float final_value_kz = Calculate_final_output_k(fuzzy_e_z,fuzzy_edot_z);
	// Vector3f smcC = Vector3f(final_value_cx,final_value_cy,final_value_cz);
	// Vector3f smcK = Vector3f(final_value_kx,final_value_ky,final_value_kz);

	//END FUZZY PART

	*/


	//PID EULER VERSION CONTROLLER
	#if PID_VER_EULER_CONTROLLER
		Flag_file = 1;
		//PX4_INFO("START RATE CONTROL: PID EULER VERSION");
		torque = _gain_p.emult(rad_error)+rad_error_int.emult(_gain_i) + _gain_d.emult(rate_error);
	#endif

	//SMC CONTROLLER
	#if SMC_CONTROLLER
		if(best)
		{
			c_smc = Vector3f(c_best,c_best,c_best);
			k_smc = Vector3f(k_best,k_best,k_best);
		}
		else
		{
			c_smc = Vector3f(c_,c_,c_);
			k_smc = Vector3f(k_,k_,k_);
		}
		Flag_file = 2;
		//PX4_INFO("START RATE CONTROL: SMC CONTROLLER");
		Vector3f SMCrate_S =c_smc.emult(rad_error) +rate_error;   //Surface S for SMC// last changed
		torque =SMCrate_S.emult(k_smc)+Zeta*ControlMath::sign(SMCrate_S);
	#endif




	//WRITE output into file txt

	//px4fmuv5
	//if(Flag_file  == 1) File_path ="/home/tang/Desktop/PID_ATT_CTL.txt";
	//else if(Flag_file  == 2) File_path ="/home/tang/Desktop/SMC_ATT_CTL.txt";
	// std::ofstream file;
	// double timeB = std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
	// file.open(File_path,std::ios::app);
	// file<<euler_rad_current(0)<<" "<<euler_rad_sp(0)<<" "<<rad_error(0)<<" "<<euler_rad_current(1)<<" "<<euler_rad_sp(1)<<" "<<rad_error(1)<<" "<<euler_rad_current(2)<<" "<<euler_rad_sp(2)<<" "<<rad_error(2)<<" "<<(double)(timeB-timed0)<<endl;
	// file.close();


	/*ORIGIN PX4 PID version control law
	//const Vector3f torque = _gain_p.emult(rate_error) + _rate_int- _gain_d.emult(angular_accel) ;//+ _gain_ff.emult(rate_sp);
	*/

	// update integral only if we are not landed for PID euler version**
	if (!landed) {
		#if PID_VER_EULER_CONTROLLER
		updateIntegral_Ver_Euler(rad_error, dt);
		#endif
		//updateIntegral(rate_error,dt);
	}

	return torque;
}

/*  RateControl::updateIntegral ORIGIN PX4
// void RateControl::updateIntegral(Vector3f &rate_error, const float dt)
// {
// 	for (int i = 0; i < 3; i++)
// 	 {
// 		// prevent further positive control saturation
// 		if (_control_allocator_saturation_positive(i)) {
// 			rate_error(i) = math::min(rate_error(i), 0.f);
// 		}

// 		// prevent further negative control saturation
// 		if (_control_allocator_saturation_negative(i)) {
// 			rate_error(i) = math::max(rate_error(i), 0.f);
// 		}

// 		// I term factor: reduce the I gain with increasing rate error.
// 		// This counteracts a non-linear effect where the integral builds up quickly upon a large setpoint
// 		// change (noticeable in a bounce-back effect after a flip).
// 		// The formula leads to a gradual decrease w/o steps, while only affecting the cases where it should:
// 		// with the parameter set to 400 degrees, up to 100 deg rate error, i_factor is almost 1 (having no effect),
// 		// and up to 200 deg error leads to <25% reduction of I.
// 		float i_factor = rate_error(i) / math::radians(400.f);
// 		i_factor = math::max(0.0f, 1.f - i_factor * i_factor);

// 		// Perform the integration using a first order method
// 		float rate_i = _rate_int(i) + i_factor * _gain_i(i) * rate_error(i) * dt;

// 		// do not propagate the result if out of range or invalid
// 		if (PX4_ISFINITE(rate_i))
// 		{
// 			_rate_int(i) = math::constrain(rate_i, -_lim_int(i), _lim_int(i));
// 		}
// 	}
// }
*/


void RateControl::updateIntegral_Ver_Euler(Vector3f rad_err, const float dt)
{
	for (int i = 0; i < 3; i++)
	{
		rad_error_int(i) = rad_error_int(i)+rad_err(i)*dt;
		rad_error_int(i) = math::constrain(rad_error_int(i), float(-0.1), float(0.1));
	}
}





void RateControl::getRateControlStatus(rate_ctrl_status_s &rate_ctrl_status)
{
	rate_ctrl_status.rollspeed_integ = _rate_int(0);
	rate_ctrl_status.pitchspeed_integ = _rate_int(1);
	rate_ctrl_status.yawspeed_integ = _rate_int(2);
}
