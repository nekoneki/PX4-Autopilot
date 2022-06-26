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
#include <chrono>
#include<string>

//PIDNN CONTROLLER SWITCH
/*
IF WANNA PIDNN SET 1,1 AND SET ALL SMCNN PART TO 0,0,0
*/
#define RBFNN_PID 0
#define PIDNN_CONTROLLER 0


//SMCNN(+FUZZY) SWITCH
/*
IF WANNA SMCNN ONY , SET 1,1,0 AND SET ALL PIDNN PART TO 0,0,0
IF WANNA SMCNN FUZZY, SET 1,1,1 AND SET ALL PIDNN PART TO 0,0,0
*/
#define SMCNN_CONTROLLER 1
#define RBFNN_SMC 1
#define FUZZY_SMC 0


using namespace matrix;
using namespace std;
struct fuzzy_e;
struct fuzzy_edot;
int flag_file;
int best;   // let mc_rate_control has best smc value while we are having smc_nn_fuzzy in mc_pos_control
string file_path;


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
const int n = 10;
const float zeta = 0.5;

double timed = std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();



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


	// if(isNAN(_pos_sp(0))) _pos_sp(0) = last_pos_xd;
	// if(isNAN(_pos_sp(1))) _pos_sp(1) = last_pos_yd;
	// if(isNAN(_pos_sp(2))) _pos_sp(2) = last_pos_zd;

	Vector3f pos_error = _pos_sp - _pos;
	Vector3f vel_error = _vel_sp - _vel;
	Vector3f acc_sp_velocity;

	//PIDNN PART
	#if PIDNN_CONTROLLER
		#if RBFNN_PID
			//PX4_INFO("LAUNCH POS_CONTROL PID_NN");
			flag_file = 1;
			Vector3f e =  pos_error;   //error position
			Vector3f e_dot =  vel_error; //error velocity
			float Mat_B[] ={0,0,0,1,1,1,0,0,0,1,1,1,0,0,0,1,1,1};
			float mu_x[] = {-0.5000,-0.3889,-0.2778,-0.1667,-0.0556,0.0556,0.1667,0.2778,0.3889,0.5000};
			float mu_y[] = {-0.5000,-0.3889,-0.2778,-0.1667,-0.0556,0.0556,0.1667,0.2778,0.3889,0.5000};
			float mu_z[] = {-0.7000,-0.5444,-0.3889,-0.2333,-0.0778,0.0778,0.2333,0.3889,0.5444,0.7000};
			float E_mat[] = {e(0), e(1), e(2), e_dot(0), e_dot(1), e_dot(2)};

			Matrix<float, 6, 1> E(E_mat);   //  matrix E_mat
			Matrix<float, n, 3> W_hat_dot;  //matrix W_hat_dot
			Matrix<float, n, 1> Hx;         //matrix Hx
			Matrix<float, 3, 1> f_x;        //matrix f_x output NN
			Matrix<float, 6, 3> Matrix_B(Mat_B);  // matrix B
			float _W_hat_copy[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; //initialization for W_hat matrix
			float P_mat[] = {3.0497,0,0,0.7483,0,0,0,3.0497,0,0,0.7483,0,0,0,0,0,0,0,0.7483,0,0,1.5,0,0,0,0.7483,0,0,1.5,0,0,0,0,0,0,0};
			Matrix<float, 6, 6> P(P_mat);   // 6x6 symetric matrix


			//calculate matrix Hx(i) and assign to Hx
			for(int i = 0;i<n;i++)
			{

				Hx(i,0)= exp(-(pow((E_mat[0]-mu_x[i]),2)+ pow((E_mat[1]-mu_y[i]),2)+ pow((E_mat[2]-mu_z[i]),2)+ pow((E_mat[3]-mu_x[i]),2)+ pow((E_mat[4]-mu_x[i]),2)+ pow((E_mat[5]-mu_x[i]),2))/pow(etha,2));

			}

			W_hat_dot = -Hx*E.transpose()*P*Matrix_B; // calculate W_hat_dot    matrix 10x3
			Matrix<float, n, 3> W_hat(_W_hat_copy);    // matrix 10x3
			W_hat += W_hat_dot*dt;
			W_hat.copyTo(_W_hat_copy);
			f_x = W_hat.transpose()*Hx;  // calculate f_x: (3x10)(10x1)= 3x1 matrix ,  output of NN , estimate error of controller to x,y,z acceleration_setpoint_velocity

			//PIDNN control law:
			acc_sp_velocity = pos_error.emult(_gain_vel_p)+vel_error.emult(_gain_vel_d)-f_x;

		#else
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

			Vector3f e1=  pos_error;   //error position
			Vector3f e_dot1 =  vel_error; //error velocity
			float mu_x1[] = {-0.5000,-0.3889,-0.2778,-0.1667,-0.0556,0.0556,0.1667,0.2778,0.3889,0.5000};
			float mu_y1[] = {-0.5000,-0.3889,-0.2778,-0.1667,-0.0556,0.0556,0.1667,0.2778,0.3889,0.5000};
			float mu_z1[] = {-0.7000,-0.5444,-0.3889,-0.2333,-0.0778,0.0778,0.2333,0.3889,0.5444,0.7000};
			float Z[] = {e1(0), e1(1), e1(2), e_dot1(0), e_dot1(1), e_dot1(2)};  // Z matrix
			Matrix<float, n, 3> W_hat_dot1; //
			Matrix<float, n, 1> Sz1;
			Matrix<float, 3, 1> f_x1;
			float _W_hat_copy1[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; //initialization for W_hat matrix
			Matrix<float, n, n> Kw1;
			Kw1.setIdentity();

			//calculate matrix Sz(i) and assign to Sz
			for(int i = 0;i<n;i++)
			{

				Sz1(i,0)= exp(-(pow((Z[0]-mu_x1[i]),2)+ pow((Z[1]-mu_y1[i]),2)+ pow((Z[2]-mu_z1[i]),2)+ pow((Z[3]-mu_x1[i]),2)+ pow((Z[4]-mu_x1[i]),2)+ pow((Z[5]-mu_x1[i]),2))/pow(etha,2));

			}

			W_hat_dot1= -Kw1*Sz1*S.transpose(); // calculate W_hat_dot
			Matrix<float, n, 3> W_hat1(_W_hat_copy1);
			W_hat1 += W_hat_dot1*dt;
			W_hat1.copyTo(_W_hat_copy1);
			f_x1 = W_hat1.transpose()*Sz1;  // calculate f_x: 3x1 matrix ,  output of NN
			acc_sp_velocity = S.emult(smc_k)+zeta*ControlMath::sign(S)+f_x1;
		#else
		//PX4_WARN("RBFNN_SMC WAS NOT DEFINED AS TRUE!");

		#endif
	#endif


	//WRITE output into file txt
	if(flag_file  == 1) file_path ="/home/tang/Desktop/PIDNN_POS_CTL.txt";
	else if(flag_file  == 2) file_path ="/home/tang/Desktop/SMCNN_POS_CTL.txt";
	else if(flag_file  == 3) file_path ="/home/tang/Desktop/SMCNNFUZZY_POS_CTL.txt";

	std::ofstream file;
	double time1 = std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
	file.open(file_path,std::ios::app);  // write output
	file<<_pos(0)<<" "<<_pos_sp(0)<<" "<<pos_error(0)<<" "<<_pos(1)<<" "<<_pos_sp(1)<<" "<<pos_error(1)<<" "<<_pos(2)<<" "<<_pos_sp(2)<<" "<<pos_error(2)<<" "<<(double)(time1-timed)<<endl;
	file.close();



	// FILE *fichier = fopen(file_path,"a");

	// 	double timeA = std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
	// 	fprintf(fichier,"%f\t%f\t\n", (double)_pos(0),(double)(timeA-timed));
	// fclose(fichier);





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
