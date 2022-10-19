#include "Att_control.hpp"

using namespace matrix;

void addIfNotNan(float &setpoint, const float addition)
{
	if (PX4_ISFINITE(setpoint) && PX4_ISFINITE(addition)) {
		// No NAN, add to the setpoint
		setpoint += addition;

	} else if (!PX4_ISFINITE(setpoint)) {
		// Setpoint NAN, take addition
		setpoint = addition;
	}

	// Addition is NAN or both are NAN, nothing to do
}

void addIfNotNanVector3f(Vector3f &setpoint, const Vector3f &addition)
{
	for (int i = 0; i < 3; i++) {
		addIfNotNan(setpoint(i), addition(i));
	}
}

void setZeroIfNanVector3f(Vector3f &vector)
{
	// Adding zero vector overwrites elements that are NaN with zero
	addIfNotNanVector3f(vector, Vector3f());
}

void Att_Control::setControllerGain(const matrix::Vector3f &lambda_omega,const matrix::Vector3f &usr_k_omega,const matrix::Vector3f &usr_eso_gain,const float &usr_Kq)
{

     _controller_param.lambda_q(0,0)=lambda_omega(0);
     _controller_param.lambda_q(1,1)=lambda_omega(1);
     _controller_param.lambda_q(2,2)=lambda_omega(2);

     _controller_param.K_w(0,0)=usr_k_omega(0);
     _controller_param.K_w(1,1)=usr_k_omega(1);
     _controller_param.K_w(2,2)=usr_k_omega(2);

     _controller_param.k_q=usr_Kq;

     _usr_eso.bm_gain=usr_eso_gain;
}


void Att_Control:: update(const matrix::Quatf &q, const matrix::Vector3f &rate, const matrix::Vector3f &tau_static,
                          const float &dt, const bool &landed, Vector3f& torque,matrix::Vector3f &rates_sp)
{
      matrix::Vector3f  tau_static_cheack = tau_static;
    setZeroIfNanVector3f(tau_static_cheack);
    tau_static_cheack(0) = math::constrain(tau_static_cheack(0), -2.0f,2.0f);
    tau_static_cheack(1) = math::constrain(tau_static_cheack(1), -2.0f,2.0f);
    tau_static_cheack(2) = math::constrain(tau_static_cheack(2), -2.0f,2.0f);
    runAttitudeControl(q,rate,tau_static_cheack,dt,torque,rates_sp);
}



void  Att_Control::runAttitudeControl(const matrix::Quatf &q, const matrix::Vector3f &rate,
                                      const matrix::Vector3f &tau_static,const float &dt, Vector3f& torque,matrix::Vector3f &rates_sp)
{
    Dcmf R(q);

    Eulerf euler(q);
    Eulerf euler_sp(_attitude_setpoint_q);
    Dcmf R_d(_attitude_setpoint_q);
    Dcmf R_error=R_d.transpose() * R ;

    float trace_R = R_error.trace();
    float q_error_0=0.5f*sqrt(1.0f+trace_R);
    Vector3f q_error;
    Matrix3f t = ((0.25f) / q_error_0)*(R_error-R_error.transpose());
    q_error(0) = t(2,1);
    q_error(1) = t(0,2);
    q_error(2) = t(1,0);

    Vector3f  bm_omega_r{(_controller_param.lambda_q*q_error)*(-2.0f)};//required
    rates_sp = bm_omega_r;

    // Feed forward the yaw setpoint rate.
    // yawspeed_setpoint is the feed forward commanded rotation around the world z-axis,
    // but we need to apply it in the body frame (because _rates_sp is expressed in the body frame).
    // Therefore we infer the world z-axis (expressed in the body frame) by taking the last column of R.transposed (== q.inversed)
    // and multiply it by the yaw setpoint rate (yawspeed_setpoint).
    // This yields a vector representing the commanded rotatation around the world z-axis expressed in the body frame
    // such that it can be added to the rates setpoint.
    if (is_finite(_yawspeed_setpoint)) {
            bm_omega_r += q.inversed().dcm_z() * _yawspeed_setpoint;
    }

    float q_for_skew_symm[9]={0,-q_error(2),q_error(1),q_error(2),0,-q_error(0),-q_error(1),q_error(0),0};
    Matrix<float,3, 3> q_skew_symm(q_for_skew_symm);

    Matrix<float,3, 3> eye_3;
    eye_3.identity();
    Vector3f q_error_v_dot=(eye_3*q_error_0+q_skew_symm)*rate*0.5f;
    Vector3f bm_omega_r_dot{(_controller_param.lambda_q*q_error_v_dot)*(-2.0f)};

    Vector3f s_w=rate-bm_omega_r;
    setZeroIfNanVector3f(_usr_eso.delta_esti);
    torque = _I_b*(bm_omega_r_dot-_usr_eso.delta_esti)+rate%(_I_b*rate)-_controller_param.K_w*s_w- q_error*_controller_param.k_q -tau_static;
    _tau(0) = torque(0);
    _tau(1) = torque(1);
    _tau(2) = torque(2);
    Vector3f u_w=_I_b_inve*(_tau-rate%(_I_b*rate)+ tau_static);
    UsrAttitudeESO(rate,u_w, dt);
    _usr_eso.delta_esti(0) = math::constrain(_usr_eso.delta_esti(0), -5.0f,5.0f);
    _usr_eso.delta_esti(1) = math::constrain(_usr_eso.delta_esti(1), -5.0f,5.0f);
    _usr_eso.delta_esti(2) = math::constrain(_usr_eso.delta_esti(2), -5.0f,5.0f);
    _att_helper.timestamp = hrt_absolute_time();
    _att_helper.data_1[0]  =  _usr_eso.delta_esti(0);
    _att_helper.data_1[1]  =  _usr_eso.delta_esti(1);
    _att_helper.data_1[2]  =  _usr_eso.delta_esti(2);
    att_helper_pub.publish(_att_helper);
}

void Att_Control::UsrAttitudeESO(matrix::Vector3f bm_omega,matrix::Vector3f u,float dt)
{
    Matrix<float,3, 3> ESO_gain;
    ESO_gain.identity();
    ESO_gain(0,0)=_usr_eso.bm_gain(0);
    ESO_gain(1,1)=_usr_eso.bm_gain(1);
    ESO_gain(2,2)=_usr_eso.bm_gain(2);

    // check
    setZeroIfNanVector3f(_usr_eso.delta_esti);
    setZeroIfNanVector3f(_usr_eso.bm_rate_esti);
    setZeroIfNanVector3f(bm_omega);
    setZeroIfNanVector3f(u);
    dt = PX4_ISFINITE(dt)? dt:0.0f;

    _usr_eso.bm_rate_esti_dot=u + _usr_eso.delta_esti + 2.0f*ESO_gain*(bm_omega-_usr_eso.bm_rate_esti);
    _usr_eso.delta_esti_dot=ESO_gain*ESO_gain*(bm_omega-_usr_eso.bm_rate_esti);
    _usr_eso.bm_rate_esti=_usr_eso.bm_rate_esti+_usr_eso.bm_rate_esti_dot*dt;
    _usr_eso.delta_esti=_usr_eso.delta_esti+_usr_eso.delta_esti_dot*dt;
}

void Att_Control::resetESO()
{
    _usr_eso.bm_rate_esti={0.0f,0.0f,0.0f};
    _usr_eso.bm_rate_esti_dot={0.0f,0.0f,0.0f};
    _usr_eso.delta_esti={0.0f,0.0f,0.0f};
    _usr_eso.delta_esti_dot={0.0f,0.0f,0.0f};
    _tau={0.0f,0.0f,0.0f};
}

void Att_Control::getRateControlStatus(rate_ctrl_status_s &rate_ctrl_status)
{
    rate_ctrl_status.rollspeed_integ = _usr_eso.delta_esti(0);
    rate_ctrl_status.pitchspeed_integ = _usr_eso.delta_esti(1);
    rate_ctrl_status.yawspeed_integ = _usr_eso.delta_esti(2);
}

