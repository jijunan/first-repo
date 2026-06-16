#ifndef _VISION_H_
#define _VISION_H_

#include "main.h"
#include "usbd_cdc_if.h"
#include "stdbool.h"
#include "MY_Define.h"
#include "RUI_MATH.h"
#include "DBUS.h"
#include "Referee.h"
#include "IMU_Task.h"
#include "Motors.h"
#include "VT13.h"


union RUI_U_VISION_RECEIVE
{
  struct
  {
    // uint8_t HEAD;
    float PIT_DATA;
    float YAW_DATA;
    bool TARGET;
    bool fire;
    bool state;
		float Yaw_plan;
		float Pitch_plan;
  };
  uint8_t DATA[23];
};
// #pragma pack(pop)

union RUI_U_VISION_SEND
{
  struct
  {
    float PIT_DATA;
    float YAW_DATA;
		float ROLL_DATA;
    float INIT_FIRING_RATE; // ����
    int FLAG;            // ��������������л���־λ
    bool COLOR;             // TRUE����ɫ��FALSE�Ǻ�ɫ
		 uint8_t is_buff; // 0:���� 1:���

    uint32_t TIME;
    uint8_t bulletSpeed;//
	    float YawOmega;
      float PitchOmega;

//		uint8_t 
  };
  uint8_t DATA[24];
};

typedef struct TYPEDEF_VISION
{
  union RUI_U_VISION_RECEIVE RECEIVE;
  union RUI_U_VISION_SEND SEND;
  uint8_t OriginData[19];
  uint8_t RECV_FLAG[2];
  int RECV_OutTime;
  uint32_t block_Time;
}TYPEDEF_VISION;

typedef union ReceiveDataUnion_typedef							//������(���ڽ��ܸ�������)(�Ӿ���������)
	{    
			uint8_t U[4];
			float F;
			uint32_t I;
	}VisionTemp;



uint8_t VISION_F_Cal(uint8_t *RxData, uint8_t type,TYPEDEF_VISION *VISION_DATA);
int ControltoVision(union RUI_U_VISION_SEND*  Send_t , uint8_t *buff, uint8_t type,User_Data_T *users,VT13_Typedef *VT13_DBUS,IMU_Data_t *IMU_Data,TYPEDEF_VISION *VISION_V_DATA,MOTOR_Typdef *ALL_MOTOR,uint8_t bspeed);
void VisionSendInit(union RUI_U_VISION_SEND*  Send_t,TYPEDEF_VISION *VISION_DATA,User_Data_T*User_Data_aaa,VT13_Typedef *DBUS_sss,IMU_Data_t *IMU_Data,MOTOR_Typdef*Motor_t,uint8_t bspeed);
void VISION_F_Monitor(TYPEDEF_VISION *VISION_V_DATA);


	
	
	
	///////Top.h
	
	
	typedef struct TYPEDEF_TOP_DATA
{
    float REALITY_ANGLE; // ��ʵ�Ƕ�//��Ȧ��
    int16_t YAW_ANGLE[2];
    float YAW_ANGLE_F;
    int16_t YAW_SPEED[2];
    int16_t PIT_ANGLE[2];
    float PIT_ANGLE_F;
    int16_t PIT_SPEED[2];
    int16_t ROUND;
} TYPEDEF_TOP_DATA;

typedef union TYPEDEF_TOP_DATA_UNION // ������(���ڽ��ܸ�������)(�Ӿ���������)
{
    struct
    {
        int16_t YAW_ANGLE;
        int16_t PIT_ANGLE;
        int16_t YAW_SPEED;
        int16_t PIT_SPEED;
    };
    uint8_t GET_DATA[8];
} TYPEDEF_TOP_DATA_UNION;


typedef struct TYPEDEF_TOP
{
    float yaw[6];   // add by yu 1-now 0-last 2-laps 3-infinite 4-0-close/1-open
    float pitch[6]; // add by yu 1-now 0-last 2-laps 3-infinite 4-0-close/1-open
    float roll[6];  // add by yu 1-now 0-last 2-laps 3-infinite 4-0-close/1-open
}TYPEDEF_TOP;

extern float yaw, pitch, roll;
//extern QEKF_INS_t QEKF_INS;

// extern float Top[5];
extern TYPEDEF_TOP TOP;
extern float currentAngle; // ������yawƫ��Ƕ�
extern void TOP_T_Cal();
extern void TOP_T_Monitor();
extern uint64_t Self_color;
void TOP_T_Cal_T();








//////////////////////////////
/////
/////     ����Ϊ����Ԥ����Ӿ�ͨѶ
/////
//////////////////////////////










//typedef union
//{
//  uint8_t Data[4];
//  float Data_f;
//  uint32_t Data_u32;
//}VisionTemp;


//typedef struct
//{
//    uint8_t OriginData[21];
//    struct Data
//    {
//      uint8_t Head_frame;
//      uint8_t End_frame;
//      float PitchAngle;
//      float YawAngle;
//      float PitchOmega;
//      float YawOmega;
//      float VisionTime;
//      uint16_t OffCounter; // ���߼��
//      uint8_t  isOnline;
//			bool TARGET;
//			bool fire;
//			bool state;
//			} Data;
//}VisionRxDataUnion;

//typedef struct
//{
//  uint8_t data[22];

//  uint8_t Head_frame;
//  float PitchAngle;
//  float YawAngle;
//  float PitchOmega;
//  float YawOmega;
//  float VisionTime;
//  uint8_t End_frame;
//}VisionTxDataUnion;
//typedef  struct
//{
//	VisionTxDataUnion VISION_TX;
//	VisionRxDataUnion VISION_RX;
//}VISION_TYPEDEF;
//int8_t Vision_Rx_Data(uint8_t* buffer, VisionRxDataUnion *VisionRx);
//void Vision_Tx_Data(MOTOR_Typdef *motor,IMU_Data_t *IMU,CONTAL_Typedef *CONTAL_A);
//void Vision_Monitor(VisionRxDataUnion *VisionRx);

extern  uint8_t v_state;


#endif
