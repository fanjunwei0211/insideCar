#ifndef _PITCH_MOTOR_H
#define _PITCH_MOTOR_H
#include "struct_typedef.h"
#include "can.h"

typedef struct 
{
	uint16_t motor_id;
	uint8_t INS_code;		//instruction code.
	uint8_t motor_fbd;	//motor CAN communication feedback.
}motor4310_feedback_t;

typedef struct 
{
	uint16_t angle_actual_int;
	uint16_t angle_desired_int;
	int16_t speed_actual_int;
	int16_t speed_desired_int;
	int16_t current_actual_int;
	int16_t current_desired_int;
	float 	speed_actual_rad;
	float 	speed_desired_rad;
	float 	angle_actual_rad;	
	float   angle_desired_rad;
	uint16_t	motor_id;
	uint8_t 	temperature;
	uint8_t		error;
	float     angle_actual_float;
	float 		speed_actual_float;
	float 		current_actual_float;
	float     angle_desired_float;
	float 		speed_desired_float;
	float 		current_desired_float;
	float			power;
	uint16_t	acceleration;
	uint16_t	linkage_KP;
	uint16_t 	speed_KI;
	uint16_t	feedback_KP;
	uint16_t	feedback_KD;
}motor4310_data_t;

void set_motor4310_comm_mode(uint16_t motor_id,uint8_t cmd);
void reset_motor4310_id(void);
void set_motor4310_id(uint16_t motor_id,uint16_t motor_id_new);
void read_motor4310_comm_mode(uint16_t motor_id);
void read_motor4310_id(void);
void set_motors4310_current(motor4310_data_t motor4310_data[8],uint8_t motor_quantity);
void send_motor4310_ctrl_cmd(uint16_t motor_id,float kp,float kd,float pos,float spd,float tor);
void set_motor4310_position(uint16_t motor_id,float pos,uint16_t spd,uint16_t cur,uint8_t ack_status);
void set_motor4310_speed(uint16_t motor_id,float spd,uint16_t cur,uint8_t ack_status);
void set_motor4310_cur_tor(uint16_t motor_id,int16_t cur_tor,uint8_t ctrl_status,uint8_t ack_status);
void set_motor4310_acceleration(uint16_t motor_id,uint16_t acc,uint8_t ack_status);
void set_motor4310_linkage_speedKI(uint16_t motor_id,uint16_t linkage,uint16_t speedKI,uint8_t ack_status);
void set_motor4310_feedback_tKP_KD(uint16_t motor_id,uint16_t fdbKP,uint16_t fdbKD,uint8_t ack_status);
void get_motor4310_parameter(uint16_t motor_id,uint8_t param_cmd);
void get_motor4310_data(CAN_RxHeaderTypeDef *RxMessage,uint8_t RxData[],uint8_t comm_mode);


void Pitch_Motor_Control(void);
void pitch_motor_init(void);

void  Pitch_Motor_Control_in_RC_Mode(void);
void 	Pitch_Motor_Control_in_Autoaim_Mode(void);

void calculate_pitch_motor_current_or_torque_with_autoaim_target_INS_angle(void);
void calculate_pitch_motor_current_or_torque_with_target_INS_speed(void);
void calculate_pitch_motor_current_or_torque_with_target_INS_angle(void);

void calculate_pitch_motor_current_or_torque_with_target_mouse_INS_speed(void);
void calculate_pitch_motor_current_or_torque_with_target_mouse_INS_angle(void);
#endif

