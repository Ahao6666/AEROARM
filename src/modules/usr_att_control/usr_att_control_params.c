
/**
 * I_xx of quadcopter gazebo 0.03f feihu 0.2198f
 *
 * @min 0.0
 * @max 20.0
 * @decimal 2
 * @group  User Position Control
 */
PARAM_DEFINE_FLOAT(USR_I_XX, 0.2198f);

/**
 * I_yy of quadcopter gazebo 0.03f feihu 0.2167f
 *
 * @min 0.0
 * @max 20.0
 * @decimal 2
 * @group  User Position Control
 */
PARAM_DEFINE_FLOAT(USR_I_YY, 0.2167f);

/**
 * I_zz of quadcopter gazebo 0.03f feihu 0.3097f
 *
 * @min 0.0
 * @max 20.0
 * @decimal 2
 * @group  User Position Control
 */
PARAM_DEFINE_FLOAT(USR_I_ZZ, 0.3097f);

/**
 * I_xy of quadcopter gazebo 0.0f feihu 0.0f
 *
 * @min 0.0
 * @max 20.0
 * @decimal 2
 * @group  User Position Control
 */
PARAM_DEFINE_FLOAT(USR_I_XY, 0.0f);
/**
 * I_xz of quadcopter gazebo 0.0f feihu 0.0f
 *
 * @min 0.0
 * @max 20.0
 * @decimal 2
 * @group  User Position Control
 */
PARAM_DEFINE_FLOAT(USR_I_XZ, 0.0f);
/**
 * I_yz of quadcopter gazebo 0.0f feihu 0.0f
 *
 * @min 0.0
 * @max 20.0
 * @decimal 2
 * @group  User Position Control
 */
PARAM_DEFINE_FLOAT(USR_I_YZ, 0.0f);

/**
 * gain of q_x
 *
 * @min 0.0
 * @max 20.0
 * @decimal 2
 * @group  User Position Control
 */
PARAM_DEFINE_FLOAT(USR_LAMMDA_Q_X, 3.0f);

/**
 * gain of q_y  gazebo 1.0 feihu 0.6
 *
 * @min 0.0
 * @max 20.0
 * @decimal 2
 * @group  User Position Control
 */
PARAM_DEFINE_FLOAT(USR_LAMMDA_Q_Y, 3.0f);

/**
 * gain of q_z gazebo 1.5 feihu 0.6
 *
 * @min 0.0
 * @max 20.0
 * @decimal 2
 * @group  User Position Control
 */
PARAM_DEFINE_FLOAT(USR_LAMMDA_Q_Z, 3.0f);

/**
 * gain of kq gazebo 1.5 feihu 9
 *
 * @min 0.0
 * @max 20.0
 * @decimal 2
 * @group  User Position Control
 */
PARAM_DEFINE_FLOAT(USR_KQ, 18.0f);

/**
 * gains  gazebo 1.0 feihu 0.7
 *
 * @min 0.0
 * @max 20.0
 * @decimal 2
 * @group  User Position Control
 */
PARAM_DEFINE_FLOAT(USR_K_OMEGA_X, 4.0f);

/**
 * gains , gazebo 1.0 feihu 0.7
 *
 * @min 0.0
 * @max 20.0
 * @decimal 2
 * @group  User Position Control
 */
PARAM_DEFINE_FLOAT(USR_K_OMEGA_Y, 4.0f);

/**
 * gains ,  gazebo 1.5 feihu 2.5
 *
 * @min 0.0
 * @max 20.0
 * @decimal 2
 * @group  User Position Control
 */
PARAM_DEFINE_FLOAT(USR_K_OMEGA_Z, 6.0f);

/**
 * gains of ESO,  gazebo 0 feihu 2
 *
 * @min 0.0
 * @max 40.0
 * @decimal 2
 * @group  User Position Control
 */
PARAM_DEFINE_FLOAT(USR_ESO_X, 10.0f);

/**
 * gains of ESO,  gazebo 0 feihu 2
 *
 * @min 0.0
 * @max 40.0
 * @decimal 2
 * @group  User Position Control
 */
PARAM_DEFINE_FLOAT(USR_ESO_Y, 10.0f);

/**
 * gains of ESO,  gazebo 0 feihu 2
 *
 * @min 0.0
 * @max 40.0
 * @decimal 2
 * @group  User Position Control
 */
PARAM_DEFINE_FLOAT(USR_ESO_Z, 10.0f);

/**
 *  sacle factor for torque.
 * @unit norm
 * @min 2
 * @max 50
 * @decimal 1
 * @increment 5
 * @group User Attitude Control
 */
/**
 *sacle factor for torque.  gazebo 5.6 feihu 24
 *
 * @min 0.0
 * @max 50.0
 * @decimal 2
 * @group  User Position Control
 */
PARAM_DEFINE_FLOAT(USR_TAU_COE, 24.0f);


// /**
// * Max roll rate
// *
// * Limit for roll rate in manual and auto modes (except acro).
// * Has effect for large rotations in autonomous mode, to avoid large control
// * output and mixer saturation.
// *
// * This is not only limited by the vehicle's properties, but also by the maximum
// * measurement rate of the gyro.
// *
// * @unit deg/s
// * @min 0.0
// * @max 1800.0
// * @decimal 1
// * @increment 5
// * @group Multicopter Attitude Control
// */

PARAM_DEFINE_FLOAT(USR_ROLLRATE_MAX, 220.0f);

// /**
// * Max pitch rate
// *
// * Limit for pitch rate in manual and auto modes (except acro).
// * Has effect for large rotations in autonomous mode, to avoid large control
// * output and mixer saturation.
// *
// * This is not only limited by the vehicle's properties, but also by the maximum
// * measurement rate of the gyro.
// *
// * @unit deg/s
// * @min 0.0
// * @max 1800.0
// * @decimal 1
// * @increment 5
// * @group Multicopter Attitude Control
// */

PARAM_DEFINE_FLOAT(USR_PITRATE_MAX, 220.0f);

// /**
// * Max yaw rate
// *
// * @unit deg/s
// * @min 0.0
// * @max 1800.0
// * @decimal 1
// * @increment 5
// * @group Multicopter Attitude Control
// */

PARAM_DEFINE_FLOAT(USR_YAWRATE_MAX, 200.0f);



// /**
// * Manual tilt input filter time constant
// *
// * Setting this parameter to 0 disables the filter
// *
// * @unit s
// * @min 0.0
// * @max 2.0
// * @decimal 2
// * @group Multicopter Position Control
// */

PARAM_DEFINE_FLOAT(USR_MAN_TILT_TAU, 0.0f);

/**
 * Battery power level scaler
 *
 * This compensates for voltage drop of the battery over time by attempting to
 * normalize performance across the operating range of the battery. The copter
 * should constantly behave as if it was fully charged with reduced max acceleration
 * at lower battery percentages. i.e. if hover is at 0.5 throttle at 100% battery,
 * it will still be 0.5 at 60% battery.
 *
 * @boolean
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_INT32(USR_BAT_SCALE_EN, 0);
