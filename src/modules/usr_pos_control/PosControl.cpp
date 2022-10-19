/*****************************************************************************/

/**
* @file pos_control.cpp.cpp
* Author:Huazi Cao
*/

#include "PosControl.hpp"
#include <mathlib/mathlib.h>

using namespace matrix;
using namespace time_literals;

void PosControl::setControlParas(const matrix::Vector3f &bm_lambda_p, const matrix::Vector3f &bm_K_v)
{
    _contolParas.bm_lambda_p(0,0) = bm_lambda_p(0);
    _contolParas.bm_lambda_p(1,1) = bm_lambda_p(1);
    _contolParas.bm_lambda_p(2,2) = bm_lambda_p(2);
    _contolParas.bm_Kv(0,0) = bm_K_v(0);
    _contolParas.bm_Kv(1,1) = bm_K_v(1);
    _contolParas.bm_Kv(2,2) = bm_K_v(2);
}

void PosControl::setESOParas(const matrix::Vector3f &ESO_v)
{
    _usr_eso.gain_ESO(0,0)= ESO_v(0);
    _usr_eso.gain_ESO(1,1)= ESO_v(1);
    _usr_eso.gain_ESO(2,2)= ESO_v(2);
}

void PosControl::setVelocityLimits(const float vel_horizontal, const float vel_up, const float vel_down)
{
        _lim_vel_horizontal = vel_horizontal;
        _lim_vel_up = vel_up;
        _lim_vel_down = vel_down;
}

void PosControl::setThrustLimits(const float min, const float max)
{
        // make sure there's always enough thrust vector length to infer the attitude
        _lim_thr_min = math::max(min, 10e-4f);
        _lim_thr_max = max;
}

void PosControl::updateHoverThrust(const float hover_thrust_new)
{
        _vel_int(2) += (hover_thrust_new - _hover_thrust) * (CONSTANTS_ONE_G / hover_thrust_new);
        setHoverThrust(hover_thrust_new);
}

void PosControl::setState(const PositionControlStates &states)
{
        _pos = states.position;
        _vel = states.velocity;
        _yaw = states.yaw;
        _vel_dot = states.acceleration;
}

void PosControl::setInputSetpoint(const vehicle_local_position_setpoint_s &setpoint)
{
        _pos_sp = Vector3f(setpoint.x, setpoint.y, setpoint.z);
        _vel_sp = Vector3f(setpoint.vx, setpoint.vy, setpoint.vz);
        _acc_sp = Vector3f(setpoint.acceleration);
        _yaw_sp = setpoint.yaw;
        _yawspeed_sp = setpoint.yawspeed;
}

// TODO:I will rewrite this function
bool PosControl::update(const float dt)
{
        // x and y input setpoints always have to come in pairs
        const bool valid = (PX4_ISFINITE(_pos_sp(0)) == PX4_ISFINITE(_pos_sp(1)))
                           && (PX4_ISFINITE(_vel_sp(0)) == PX4_ISFINITE(_vel_sp(1)))
                           && (PX4_ISFINITE(_acc_sp(0)) == PX4_ISFINITE(_acc_sp(1)));

        // if(!PX4_ISFINITE(_pos_sp(0))&&!PX4_ISFINITE(_pos_sp(1))){
	// 	if(PX4_ISFINITE(_vel_sp(0)) && PX4_ISFINITE(_vel_sp(1))){
	// 		if (PX4_ISFINITE(_pos(0))&&PX4_ISFINITE(_pos(2))){
	// 			_pos_sp =  _pos + _vel_sp*dt;
	// 		}
	// 	}
	// }
        // if only velocity cmd is inputed, we integret velocity cmd to obtain position cmd.
        // This procedure is necessary, because the whole position controller requires position cmd.
        // unless noly velocity control is required by user
       for(int i = 0; i < 3; i++){
                if ((!PX4_ISFINITE(_pos_sp(i))) && PX4_ISFINITE(_vel_sp(i)) &&PX4_ISFINITE(_pos(i))) {
                        // pos_sp is NAN, vel_sp is not NAN
                        _pos_sp(i) = _pos(i) + _vel_sp(i)*dt;
                }
       }
        _positionControl(dt);

        _yawspeed_sp = PX4_ISFINITE(_yawspeed_sp) ? _yawspeed_sp : 0.f;
        _yaw_sp = PX4_ISFINITE(_yaw_sp) ? _yaw_sp : _yaw; // TODO: better way to disable yaw control

//        PX4_WARN("valid = %d\n", valid);
//        PX4_WARN("_updateSuccessful = %d\n", _updateSuccessful());
        return valid && _updateSuccessful();
}

void PosControl::_positionControl(const float dt)
{
    _autopilot.pos_err       = _pos - _pos_sp;
    // make sure there are no NAN elements for further reference while constraining
    ControlMath::setZeroIfNanVector3f(_autopilot.pos_err);
    // check _vel_sp
    ControlMath::setZeroIfNanVector3f(_vel_sp);
//    // Constrain velocity in x,y,z-direction.
//    _vel_sp(0) = math::constrain(_vel_sp(0), -_lim_vel_horizontal, _lim_vel_horizontal);
//    _vel_sp(1) = math::constrain(_vel_sp(1), -_lim_vel_horizontal, _lim_vel_horizontal);
//    _vel_sp(2) = math::constrain(_vel_sp(2), -_lim_vel_up, _lim_vel_down);
    // check _acc_sp
    ControlMath::setZeroIfNanVector3f(_acc_sp);
    // check integrete item
    ControlMath::setZeroIfNanVector3f(_autopilot.pos_err_integ);
    // TODO: need feedback the desired velocity
    // Algorithm can be found in my paper
    Vector3f vel_sp_position = - (_contolParas.bm_lambda_p*_autopilot.pos_err)*2.0f - _autopilot.pos_err_integ;
    // make sure there are no NAN elements for further reference while constraining
    ControlMath::setZeroIfNanVector3f(vel_sp_position);
    // combine desired velocity
    _autopilot.bm_vr =_vel_sp + vel_sp_position;
    // Constrain horizontal velocity by prioritizing the velocity component along the
    // the desired position setpoint over the feed-forward term.
    _autopilot.bm_vr.xy() = ControlMath::constrainXY(_autopilot.bm_vr.xy(), (_autopilot.bm_vr - vel_sp_position).xy(), _lim_vel_horizontal);
//     // Constrain velocity in z-direction.
    _autopilot.bm_vr(2) = math::constrain(_autopilot.bm_vr(2), -_lim_vel_up, _lim_vel_down);

    _autopilot.bm_vr_deriv= Vector3f(0,0,0) - _contolParas.bm_lambda_p*(_vel - _vel_sp)*2.0f - _contolParas.bm_lambda_p*_contolParas.bm_lambda_p*_autopilot.pos_err;

    _autopilot.bm_sv= _vel - _autopilot.bm_vr;

    //控制律
    // TODO:Interface for position ESO should be writen
    delta_v = _usr_eso.delta_est;//Interface for position ESO
    ControlMath::setZeroIfNanVector3f(delta_v);
   // TEST
    delta_v = Vector3f(0.0f,0.0f,0.0f);
    _autopilot.f_iusl=(_Mb+_SUM_mi)*(g+_contolParas.bm_Kv*_autopilot.bm_sv-_autopilot.bm_vr_deriv+delta_v);//now, force. Need convert force to acceleration.
    Vector3f u_v = g - (1.0f/(_Mb+_SUM_mi) ) * _autopilot.f_iusl;
    PositionESO( _pos, u_v, dt);

    //convert force to acceleration
    _acc_sp = - _autopilot.f_iusl/(_Mb+_SUM_mi)+g;



    // Assume standard acceleration due to gravity in vertical direction for attitude generation
    Vector3f body_z = Vector3f(-_acc_sp(0), -_acc_sp(1), G).normalized();

    ControlMath::limitTilt(body_z, Vector3f(0, 0, 1), _lim_tilt);

    /* Scale thrust assuming hover thrust produces standard gravity
     * from PX4:        float collective_thrust = _acc_sp(2) * (_hover_thrust / CONSTANTS_ONE_G) - _hover_thrust;
     * old version:     float collective_thrust = _acc_sp(2) * (_hover_thrust / G) - _hover_thrust;
     * newest version:  float collective_thrust = (_acc_sp(2) - G) * (_Mb + _SUM_mi);
     * result: PX4 is right. Only the codes from PX4 can run well.
    */
    float collective_thrust = _acc_sp(2) * (_hover_thrust / CONSTANTS_ONE_G) - _hover_thrust;
    // Project thrust to planned body attitude
    collective_thrust /= (Vector3f(0, 0, 1).dot(body_z));
    collective_thrust = math::min(collective_thrust, -_lim_thr_min);
    _thr_sp = body_z * collective_thrust;

    //log f_iusl for limit
    pos_helper.thrust_sp_adj[0]=_thr_sp(0);
    pos_helper.thrust_sp_adj[1]=_thr_sp(1);
    pos_helper.thrust_sp_adj[2]=_thr_sp(2);

    // Saturate maximal vertical thrust
    _thr_sp(2) = math::max(_thr_sp(2) , -_lim_thr_max);

    // Get allowed horizontal thrust after prioritizing vertical control
    const float thrust_max_squared = _lim_thr_max * _lim_thr_max;
    const float thrust_z_squared = _thr_sp(2) * _thr_sp(2) ;
    const float thrust_max_xy_squared = thrust_max_squared - thrust_z_squared;
    float thrust_max_xy = 0;

    if (thrust_max_xy_squared > 0) {
        thrust_max_xy = sqrtf(thrust_max_xy_squared);
    }

    // Saturate thrust in horizontal direction
    matrix::Vector2f thrust_sp_xy(_thr_sp(0),_thr_sp(1));

    float thrust_sp_xy_norm = thrust_sp_xy.norm();

    //Vector3f _thr_sp;
    if (thrust_sp_xy_norm > thrust_max_xy) {
        _thr_sp(0) = thrust_sp_xy(0) / thrust_sp_xy_norm * thrust_max_xy;
        _thr_sp(1) = thrust_sp_xy(1) / thrust_sp_xy_norm * thrust_max_xy;
    }

    // Calculate integral item
    _autopilot.pos_err_integ = _autopilot.pos_err_integ + _contolParas.bm_lambda_p*_contolParas.bm_lambda_p*_autopilot.pos_err*dt;                             //计算积分

    //积分限幅
    // limit integral
        _autopilot.pos_err_integ(0) = math::constrain(_autopilot.pos_err_integ(0), -SUM_max_x, SUM_max_x);
        _autopilot.pos_err_integ(1) = math::constrain(_autopilot.pos_err_integ(1), -SUM_max_y, SUM_max_y);
        _autopilot.pos_err_integ(2) = math::constrain(_autopilot.pos_err_integ(2), -SUM_max_z, SUM_max_z);

    //log
    pos_helper.timestamp = hrt_absolute_time();
    pos_helper.v_r[0]= _usr_eso.delta_est(0);
    pos_helper.v_r[1]= _usr_eso.delta_est(1);
    pos_helper.v_r[2]= _usr_eso.delta_est(2);
    pos_helper_pub.publish(pos_helper);
}

bool PosControl::_updateSuccessful()
{
        bool valid = true;

        // For each controlled state the estimate has to be valid
        for (int i = 0; i <= 2; i++) {
                if (PX4_ISFINITE(_pos_sp(i))) {
                        valid = valid && PX4_ISFINITE(_pos(i));
                }

                if (PX4_ISFINITE(_vel_sp(i))) {
                        valid = valid && PX4_ISFINITE(_vel(i)) && PX4_ISFINITE(_vel_dot(i));
                }
        }

        // There has to be a valid output accleration and thrust setpoint otherwise there was no
        // setpoint-state pair for each axis that can get controlled
        valid = valid && PX4_ISFINITE(_acc_sp(0)) && PX4_ISFINITE(_acc_sp(1)) && PX4_ISFINITE(_acc_sp(2));
        valid = valid && PX4_ISFINITE(_thr_sp(0)) && PX4_ISFINITE(_thr_sp(1)) && PX4_ISFINITE(_thr_sp(2));
        return valid;
}

void PosControl::PositionESO(matrix::Vector3f pos_in,matrix::Vector3f u,float dt)
{
    // check
    ControlMath::setZeroIfNanVector3f(_usr_eso.pos_est);
    ControlMath::setZeroIfNanVector3f(_usr_eso.vel_est);
    ControlMath::setZeroIfNanVector3f(_usr_eso.delta_est);
    ControlMath::setZeroIfNanVector3f(pos_in);
    ControlMath::setZeroIfNanVector3f(u);
    dt = PX4_ISFINITE(dt)? dt:0.0f;
    Vector3f p_est_dot = _usr_eso.vel_est + 3.0f * _usr_eso.gain_ESO * (pos_in - _usr_eso.pos_est);
    Vector3f v_est_dot = u + _usr_eso.delta_est  + 3.0f * _usr_eso.gain_ESO * _usr_eso.gain_ESO * (pos_in - _usr_eso.pos_est);
    Vector3f d_est_dot = _usr_eso.gain_ESO * _usr_eso.gain_ESO * _usr_eso.gain_ESO * (pos_in - _usr_eso.pos_est);
    _usr_eso.pos_est = _usr_eso.pos_est  + p_est_dot*dt;
    _usr_eso.vel_est = _usr_eso.vel_est  + v_est_dot*dt;
    _usr_eso.delta_est = _usr_eso.delta_est  + d_est_dot*dt;
}


void PosControl::getLocalPositionSetpoint(vehicle_local_position_setpoint_s &local_position_setpoint) const
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


void PosControl::getAttitudeSetpoint(vehicle_attitude_setpoint_s &attitude_setpoint) const
{
        ControlMath::thrustToAttitude(_thr_sp, _yaw_sp, attitude_setpoint);
        attitude_setpoint.yaw_sp_move_rate = _yawspeed_sp;
}
