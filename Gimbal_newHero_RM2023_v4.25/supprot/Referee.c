#include "referee.h"


/* Private variables ---------------------------------------------------------*/
/* protocol包头结构体 */
frame_header_struct_t Referee_Receive_Header;

/* 0x000X */
ext_game_status_t   Game_Status;
ext_game_result_t   Game_Result;
ext_game_robot_HP_t Game_Robot_HP;

/* 0x010X */
ext_event_data_t                Event_Data;
ext_supply_projectile_action_t  Supply_Projectile_Action;
ext_supply_projectile_booking_t Supply_Projectile_Booking;
ext_referee_warning_t           Referee_Warning;
ext_dart_remaining_time_t       Dart_Remaining_Time;

/* 0x020X */
ext_game_robot_state_t Game_Robot_State;
ext_power_heat_data_t  Power_Heat_Data;
ext_game_robot_pos_t   Game_Robot_Pos;
ext_buff_musk_t        Buff_Musk;
aerial_robot_energy_t  Aerial_Robot_Energy;
ext_robot_hurt_t       Robot_Hurt;
ext_shoot_data_t       Shoot_Data;
ext_bullet_remaining_t Bullet_Remaining;
ext_rfid_status_t      RFID_Status;
ext_dart_client_cmd_t  Dart_Client_Cmd;

/* 0x030X */
ext_student_interactive_header_data_t Student_Interactive_Header_Data;
robot_interactive_data_t              Robot_Interactive_Data;
ext_robot_command_t                   Robot_Command;
ext_client_map_command_t              Client_Map_Command;
