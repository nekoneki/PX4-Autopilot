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
#include <iostream>
#include <float.h>
#include <mathlib/mathlib.h>
#include <px4_platform_common/defines.h>
#include <geo/geo.h>
#include <chrono>



using namespace matrix;
using namespace std;



struct NeuralNetwork;
struct fuzzy_e;
struct fuzzy_edot;


double timed = std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();










void PositionControl::setVelocityGains(const Vector3f &P, const Vector3f &I, const Vector3f &D)
{
	_gain_vel_p = P;
	_gain_vel_i = I;
	_gain_vel_d = D;
	
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



//根据DAM家的版本修改:自己修改的出来的结论(暂定) Position 的P控制器不需要删除。
//P控制器用来获取XYZ的期望速度。 但是 dam写的Sliding mode control 最终目的是获取期望加速度.

void PositionControl::_positionControl()
{

	// P-position controller
	//根据位置误差和位置环P参数计算速度期望（NED系）
	Vector3f vel_sp_position = (_pos_sp - _pos).emult(_gain_pos_p);

	//叠加位置误差产生的速度期望和速度期望前馈为总速度期望
	ControlMath::addIfNotNanVector3f(_vel_sp, vel_sp_position);
	ControlMath::setZeroIfNanVector3f(vel_sp_position);	// make sure there are no NAN elements for further reference while constraining

	
	//根据设置的最大水平速度限制水平方向期望速度，优先保证满足位置误差引起的期望速度
	_vel_sp.xy() = ControlMath::constrainXY(vel_sp_position.xy(), (_vel_sp - vel_sp_position).xy(), 	_lim_vel_horizontal);


	//根据D向速度最大最小值限制D向速度期望
	_vel_sp(2) = math::constrain(_vel_sp(2), -_lim_vel_up, _lim_vel_down);
}




void PositionControl::_velocityControl(const float dt)
{


	Vector3f pos_error = _pos_sp - _pos;
	Vector3f vel_error = _vel_sp - _vel;

	// Fuzzy part
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


	
	//SMC part
	Vector3f *smc;
	Matrix<float, 3, 1> f_x_result;
	smc = _SMCController(final_value_cx,final_value_cy,final_value_cz,final_value_kx,final_value_ky,final_value_kz);

	//NN part
	f_x_result = _NeuralNetwork(_pos, _pos_sp, _vel, _vel_sp, smc[4],dt);




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



	//		[0]:c  [1]:k [2]:e [3]:edot [4]:S
	///////////////////////// command law for smc!!!!!!!!!!!!!!///////////////////////////////

	Vector3f acc_sp_velocity = smc[4].emult(smc[1])+zeta*ControlMath::sign(smc[4])+f_x_result; //+Total_noise;//

	// old version for smc with artificial noise
	//Vector3f acc_sp_velocity = smc[3].emult(smc[0]) + smc[4].emult(smc[1]);//+Total_noise +f_x_result; 
//refer:Vector3f acc_sp_velocity = e_dot .emult(k)     +   S   .emult(c2)   //+Total_noise;





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
