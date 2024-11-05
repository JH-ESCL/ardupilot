/// @file	AP_MotorsHeli_Single.h
/// @brief	Motor control class for traditional heli
#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>            // ArduPilot Mega Vector/Matrix math Library
#include <SRV_Channel/SRV_Channel.h>
#include "AP_MotorsHeli.h"
#include "AP_MotorsHeli_RSC.h"
#include "AP_MotorsHeli_Swash.h"
#include <AP_Math/vectorN.h>            // ArduPilot Mega Vector/Matrix math Library

// rsc and extgyro function output channels.
#define AP_MOTORS_HELI_SINGLE_EXTGYRO                          CH_7
#define AP_MOTORS_HELI_SINGLE_TAILRSC                          CH_7

// tail types
#define AP_MOTORS_HELI_SINGLE_TAILTYPE_SERVO                      0
#define AP_MOTORS_HELI_SINGLE_TAILTYPE_SERVO_EXTGYRO              1
#define AP_MOTORS_HELI_SINGLE_TAILTYPE_DIRECTDRIVE_VARPITCH       2
#define AP_MOTORS_HELI_SINGLE_TAILTYPE_DIRECTDRIVE_FIXEDPITCH_CW  3
#define AP_MOTORS_HELI_SINGLE_TAILTYPE_DIRECTDRIVE_FIXEDPITCH_CCW 4
#define AP_MOTORS_HELI_SINGLE_TAILTYPE_DIRECTDRIVE_VARPIT_EXT_GOV 5


// direct-drive variable pitch defaults
#define AP_MOTORS_HELI_SINGLE_DDVP_SPEED_DEFAULT               50

// default external gyro gain
#define AP_MOTORS_HELI_SINGLE_EXT_GYRO_GAIN                    350

// COLYAW parameter min and max values
#define AP_MOTORS_HELI_SINGLE_COLYAW_RANGE                     10.0f

// maximum number of swashplate servos
#define AP_MOTORS_HELI_SINGLE_NUM_SWASHPLATE_SERVOS            3

/// @class      AP_MotorsHeli_Single
class AP_MotorsHeli_Single : public AP_MotorsHeli {
public:

    // constructor
    AP_MotorsHeli_Single(uint16_t speed_hz = AP_MOTORS_HELI_SPEED_DEFAULT) :
        AP_MotorsHeli(speed_hz),
        _tail_rotor(SRV_Channel::k_heli_tail_rsc, AP_MOTORS_HELI_SINGLE_TAILRSC),
        _swashplate()
    {
        AP_Param::setup_object_defaults(this, var_info);
    };

    // set update rate to motors - a value in hertz
    void set_update_rate(uint16_t speed_hz) override;

   
    // output_to_motors - sends values out to the motors
    void output_to_motors() override;

    // set_desired_rotor_speed - sets target rotor speed as a number from 0 ~ 1
    void set_desired_rotor_speed(float desired_speed) override;

    // get_main_rotor_speed - estimated rotor speed when no speed sensor or governor is used
    float get_main_rotor_speed() const  override { return _main_rotor.get_rotor_speed(); }

    // get_desired_rotor_speed - gets target rotor speed as a number from 0 ~ 1
    float get_desired_rotor_speed() const  override { return _main_rotor.get_desired_speed(); }

    // rotor_speed_above_critical - return true if rotor speed is above that critical for flight
    bool rotor_speed_above_critical() const  override { return _main_rotor.get_rotor_speed() > _main_rotor.get_critical_speed(); }

    // get_governor_output
    float get_governor_output() const override { return _main_rotor.get_governor_output(); }

    // get_control_output
    float get_control_output() const override{ return _main_rotor.get_control_output(); }

    // calculate_scalars - recalculates various scalars used
    void calculate_scalars() override;

    // calculate_armed_scalars - recalculates scalars that can change while armed
    void calculate_armed_scalars() override;

    // get_motor_mask - returns a bitmask of which outputs are being used for motors or servos (1 means being used)
    //  this can be used to ensure other pwm outputs (i.e. for servos) do not conflict
    uint32_t get_motor_mask() override;

    // ext_gyro_gain - set external gyro gain in range 0 ~ 1000
    void ext_gyro_gain(float gain)  override { if (gain >= 0 && gain <= 1000) { _ext_gyro_gain_std.set(gain); }}

    // has_flybar - returns true if we have a mechical flybar
    bool has_flybar() const  override { return _flybar_mode; }

    // supports_yaw_passthrought - returns true if we support yaw passthrough
    bool supports_yaw_passthrough() const override { return _tail_type == AP_MOTORS_HELI_SINGLE_TAILTYPE_SERVO_EXTGYRO; }

    void set_acro_tail(bool set) override { _acro_tail = set; }

    // parameter_check - returns true if helicopter specific parameters are sensible, used for pre-arm check
    bool parameter_check(bool display_msg) const override;

    // var_info
    static const struct AP_Param::GroupInfo var_info[];


protected:

    // init_outputs - initialise Servo/PWM ranges and endpoints
    bool init_outputs() override;

    // update_motor_controls - sends commands to motor controllers
    void update_motor_control(RotorControlState state) override;

    // heli_move_actuators - moves swash plate and tail rotor
    void move_actuators(float roll_out, float pitch_out, float coll_in, float yaw_out) override;

    // move_yaw - moves the yaw servo
    void move_yaw(float yaw_out);

    // servo_test - move servos through full range of movement
    void servo_test() override;

    // output_test_seq - spin a motor at the pwm value specified
    //  motor_seq is the motor's sequence number from 1 to the number of motors on the frame
    //  pwm value is an actual pwm value that will be output, normally in the range of 1000 ~ 2000
    virtual void _output_test_seq(uint8_t motor_seq, int16_t pwm) override;

    // external objects we depend upon
    AP_MotorsHeli_RSC   _tail_rotor;            // tail rotor
    AP_MotorsHeli_Swash _swashplate;            // swashplate

    // internal variables
    float _oscillate_angle = 0.0f;              // cyclic oscillation angle, used by servo_test function
    float _servo_test_cycle_time = 0.0f;        // cycle time tracker, used by servo_test function
    float _collective_test = 0.0f;              // over-ride for collective output, used by servo_test function
    float collective_out = 0.0f;              // over-ride for collective output, used by servo_test function
    float _roll_test = 0.0f;                    // over-ride for roll output, used by servo_test function
    float _pitch_test = 0.0f;                   // over-ride for pitch output, used by servo_test function
    float _yaw_test = 0.0f;                     // over-ride for yaw output, used by servo_test function
    float _servo1_out = 0.0f;                   // output value sent to motor
    float _servo2_out = 0.0f;                   // output value sent to motor
    float _servo3_out = 0.0f;                   // output value sent to motor
    float _servo4_out = 0.0f;                   // output value sent to motor
    float _servo5_out = 0.0f;                   // output value sent to motor
    uint16_t rc4_in;

    // parameters
    AP_Int16        _tail_type;                 // Tail type used: Servo, Servo with external gyro, direct drive variable pitch or direct drive fixed pitch
    AP_Int16        _ext_gyro_gain_std;         // PWM sent to external gyro on ch7 when tail type is Servo w/ ExtGyro
    AP_Int16        _ext_gyro_gain_acro;        // PWM sent to external gyro on ch7 when tail type is Servo w/ ExtGyro in ACRO
    AP_Float        _collective_yaw_effect;     // Feed-forward compensation to automatically add rudder input when collective pitch is increased. Can be positive or negative depending on mechanics.
    AP_Int8         _flybar_mode;               // Flybar present or not.  Affects attitude controller used during ACRO flight mode
    AP_Int16        _direct_drive_tailspeed;    // Direct Drive VarPitch Tail ESC speed (0 ~ 1000)
    
    // 011324 JH Slow Start 
    AP_Float _slowstart_amplitude; // Max amplitude of IDLE 0.1->50pwm [%]
    AP_Float _slowstart_time; // each motor time to Max [s]
    bool check_init = 1;
    float time_div_startime = 0.0f;
    float init_servo4_out = 0.0f;
    float init_servo4_out_ = 0.0f;
    float _servo4_out_ = 0.0f;
    bool check_ignition = 1;
    float time_init_init = 0.0f;

    // 010724 JH Excitation
    float _servo4_out_1 =  0.0f;
    float _servo4_out_2 =  0.0f;
    float _servo4_out_3 =  0.0f;
    float _servo4_out_4 =  0.0f;
    float _servo4_out_5 =  0.0f; //for NSE
    float _servo4_out_6 =  0.0f; //for NSE
    AP_Float _excitation_time; // Frequency of excitation
    AP_Float _excitation_amplitude; // Amplitude of excitation
    AP_Int8 _excitation_enabled;    // Whether excitation is enabled
    AP_Int8 _fault_inj; // Inject Fault
    AP_Float _fault_percent; // Inject Fault percentage
    bool            _acro_tail = false;

    // 031724 JH Chirp signal for id
    bool _inject_chirp = false;
    AP_Int8 _id_chirp_on;
    float _chirp_start_time = 0.0f;

    // 100724 JH feedforward
    // AP_Float _ff_gain;
    float Bv_inv[4] = {0.25, 0.25,0.25,0.25};
    float Bn[4][3] = {
        {1, -1, 0},
        {0, 1, -1},
        {0, 0, 1},
        {-1, 0, 0}
    };
    AP_Float _tu; // for no nse
    float Tn = 7*0.0025; // nse time
    float last_debug_time = 0.0f;

};
