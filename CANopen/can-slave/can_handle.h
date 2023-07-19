#ifndef CAN_HANDLE_H
#define CAN_HANDLE_H
#include "data.h"
#include "main.h"
#include "can.h"
#include "master_node.h"
#include "master_node_aux.h"
#include "canFestival.h"
/*********************Э�鲿��************************/
/*				
0x01	�������		
0x02	ֹͣ���		
0x03	��λ			
0x04	ֹͣ����		
0x05	Ť��ģʽ		
0x06	�ٶ�ģʽ		
0x07	λ��ģʽ		
0x08	�������		
0x09	����������		
*/

#define MOTOR_DISABLE	  					(uint16_t)0x0//���ģʽ����
#define MOTOR_ENABLE   						(uint16_t)0x1
#define MOTOR_RESET     					(uint16_t)0x2
#define MOTOR_UPSTOP    					(uint16_t)0x3
#define MOTOR_TORQUE_MODE 				(uint16_t)0x4
#define MOTOR_SPEED_MODE 					(uint16_t)0x5
#define MOTOR_POSITION_MODE  			(uint16_t)0x6
#define MOTOR_ERROR_HANDLE 				(uint16_t)0x7
#define MOTOR_ENCODER_ALIGN 			(uint16_t)0x8

#define MOTOR_READ_TORQUE 				(uint16_t)0x1//��ȡ Ť�ء��ٶȡ�λ��
#define MOTOR_READ_SPEED  				(uint16_t)0x2
#define MOTOR_READ_POSITION  			(uint16_t)0x3

#define MOTOR_WRITE_TORQUE 				(uint16_t)0x1//д�� Ť�ء��ٶȡ�λ��
#define MOTOR_WRITE_SPEED  				(uint16_t)0x2
#define MOTOR_WRITE_POSITION  		(uint16_t)0x3

#define NO_SUCH_MODE							0X01//������

/*typedef struct last_command{  
   uint16_t motor_mode,
					 motor_speed,
					 motor_torque,
					 motor_position;
}last_control_cmd;
*/

void motor_Reset();
void motor_MotorEnable();
void motor_MotorDisable();
void motor_TorqueMode();
void motor_UpStop();
void motor_SpeedMode();
void motor_PositionMode();
void motor_ErrorHandle();
void motor_EncoderAlign();

uint16_t _GetTorque();
uint16_t _GetSpeed();
uint16_t _GetPosition();

void _SetTorque(uint16_t);
void _SetSpeed(uint16_t);
void _SetPosition(uint16_t);

void _SetMode(uint16_t);
void _motor_control();
void send_motor_perem(void);

void errorHandle(uint16_t);
void SelectSlaveDeviceId(uint32_t id);
#endif
