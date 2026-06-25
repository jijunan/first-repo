#include "Chassis_Task.h"


uint8_t chassis_task(CONTAL_Typedef *CONTAL,
                     RUI_ROOT_STATUS_Typedef *Root,
                     User_Data_T *User_data,
                     MOTOR_Typdef *MOTOR)
{
    /*目标值赋值*/
    MOTOR->DJI_3508_Chassis_1.DATA.Aim = CONTAL->BOTTOM.wheel1;
    MOTOR->DJI_3508_Chassis_2.DATA.Aim = CONTAL->BOTTOM.wheel2;
    MOTOR->DJI_3508_Chassis_3.DATA.Aim = CONTAL->BOTTOM.wheel3;
    MOTOR->DJI_3508_Chassis_4.DATA.Aim = CONTAL->BOTTOM.wheel4;

    Feedforward_Calculate(&MOTOR->DJI_3508_Chassis_1.PID_F,
                          MOTOR->DJI_3508_Chassis_1.DATA.Aim);
    PID_Calculate(&MOTOR->DJI_3508_Chassis_1.PID_S,
                  (float)MOTOR->DJI_3508_Chassis_1.DATA.Speed_now,
                  MOTOR->DJI_3508_Chassis_1.DATA.Aim);

    /*Chassis_2*/
    Feedforward_Calculate(&MOTOR->DJI_3508_Chassis_2.PID_F,
                          MOTOR->DJI_3508_Chassis_2.DATA.Aim);
    PID_Calculate(&MOTOR->DJI_3508_Chassis_2.PID_S,
                  (float)MOTOR->DJI_3508_Chassis_2.DATA.Speed_now,
                  MOTOR->DJI_3508_Chassis_2.DATA.Aim);

    /*Chassis_3*/
    Feedforward_Calculate(&MOTOR->DJI_3508_Chassis_3.PID_F,
                          MOTOR->DJI_3508_Chassis_3.DATA.Aim);
    PID_Calculate(&MOTOR->DJI_3508_Chassis_3.PID_S,
                  (float)MOTOR->DJI_3508_Chassis_3.DATA.Speed_now,
                  MOTOR->DJI_3508_Chassis_3.DATA.Aim);

    /*Chassis_4*/
    Feedforward_Calculate(&MOTOR->DJI_3508_Chassis_4.PID_F,
                          MOTOR->DJI_3508_Chassis_4.DATA.Aim);
    PID_Calculate(&MOTOR->DJI_3508_Chassis_4.PID_S,
                  (float)MOTOR->DJI_3508_Chassis_4.DATA.Speed_now,
                  MOTOR->DJI_3508_Chassis_4.DATA.Aim);

    /*总输出计算*/
    float tmp_C[4];

    tmp_C[0] = MOTOR->DJI_3508_Chassis_1.PID_F.Output +
               MOTOR->DJI_3508_Chassis_1.PID_S.Output;

    tmp_C[1] = MOTOR->DJI_3508_Chassis_2.PID_F.Output +
               MOTOR->DJI_3508_Chassis_2.PID_S.Output;

    tmp_C[2] = MOTOR->DJI_3508_Chassis_3.PID_F.Output +
               MOTOR->DJI_3508_Chassis_3.PID_S.Output;

    tmp_C[3] = MOTOR->DJI_3508_Chassis_4.PID_F.Output +
               MOTOR->DJI_3508_Chassis_4.PID_S.Output;

    chassis_power_control(&RUI_V_CONTAL,
                           User_data,
                            &model,
                            &cap,
                           &ALL_MOTOR);

    DJI_Current_Ctrl(&hcan2,
                     0x200,
                     (int16_t)MOTOR->DJI_3508_Chassis_1.PID_S.Output,
                     (int16_t)MOTOR->DJI_3508_Chassis_2.PID_S.Output,
                     (int16_t)MOTOR->DJI_3508_Chassis_3.PID_S.Output,
                     (int16_t) MOTOR->DJI_3508_Chassis_4.PID_S.Output);
}