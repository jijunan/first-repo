#include "WHW_IRQN.h"
#include "Robot.h"

//34ms,画UI任务
void StartRobotUITask(void const * argument)
{
    portTickType currentTimeRobotUI;
    currentTimeRobotUI = xTaskGetTickCount();

    for (;;)
    {

    	currentTimeRobotUI += 40;
    	osDelayUntil(currentTimeRobotUI);

    }
}

//运动控制任务
void StartMoveTask(void const * argument)
{
    portTickType currentTimeMove;
    currentTimeMove = xTaskGetTickCount();

    for (;;)
    {
    DJI_Current_Ctrl(&hcan1,0x200,0,0,400,0);
    	vTaskDelay (1);
    }
}

//对抗控制任务(电容,发射)
void StartDefiantTask(void const * argument)
{
    portTickType currentTimeDefiant;
    currentTimeDefiant = xTaskGetTickCount();
    for(;;)
    {

    	currentTimeDefiant += 2;
    	osDelayUntil(currentTimeDefiant);
    }
}

//陀螺仪解算与自瞄发送任务
void StartIMUTask(void const * argument)
{
    portTickType currentTimeIMU;
    currentTimeIMU = xTaskGetTickCount();

    static uint32_t dt_pc = 0;
    static uint32_t INS_DWT_Count = 0;

    //陀螺仪初始化
    const float imu_temp_PID[3] = TEMPERATURE_PID;
    PID_init(&imu_temp, PID_POSITION, imu_temp_PID,
             TEMPERATURE_PID_MAX_OUT, TEMPERATURE_PID_MAX_IOUT);
//    IMU_QuaternionEKF_Init(10, 0.001f, 10000000, 1, 0.001f,0); //ekf初始化
	IMU_QuaternionEKF_Init(10, 0.0001f, 10000000, 1, 0.001f,0); //ekf初始化
	mahony_init(&mahony_filter, 2.0f, 0.01f, 0.9f,0.001f);

    HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_1);
    while(BMI088_init()){}

    for(;;)
    {
        INS_Task(&IMU_Data, &imu_temp);
        dt_pc = (uint32_t)DWT_GetDeltaT(&INS_DWT_Count);

    	currentTimeIMU += 1;
    	osDelayUntil(currentTimeIMU);
    }
}

//整车监控任务
void StartRootTask(void const * argument)
{
    portTickType currentTimeRoot;
    currentTimeRoot = xTaskGetTickCount();

    for(;;)
    {

    	currentTimeRoot += 5;
    	osDelayUntil(currentTimeRoot);
    }
}
float dt_pc = 0; 
static uint32_t INS_DWT_Count = 0;
void BSP_TIM_IRQHandler(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM7) {

	}	

}

/************************************************************万能分隔符**************************************************************
 * 	@author:			//小瑞
 *	@performance:	    //CAN接收函数
 *	@parameter:		    //
 *	@time:				//22-11-23 20:42
 *	@ReadMe:			//
 *  @LastUpDataTime:    //2023-04-20 02:52    bestrui
 *  @UpData：           //更新成共用体
 *  @LastUpDataTime:    //2023-05-06 20:23    bestrui
 *  @UpData：           //更新判断逻辑
 ************************************************************万能分隔符**************************************************************/
uint8_t test[8];
uint8_t rx_data[8];

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* hcan)
{
	CAN_RxHeaderTypeDef can_rx;

    HAL_CAN_GetRxMessage(hcan , CAN_RX_FIFO0 , &can_rx , rx_data);
    
	if (hcan == &hcan1)		
	{
		//CAN1
		switch (can_rx.StdId)
		{
			case 0x203:
				MOTOR_CAN_RX_2006RM(&ALL_MOTOR.DJI_3508_Shoot_M.DATA, rx_data);
				break;
			default:
				break;
        }
			
	}
	if (hcan == &hcan2)		
	{
		//CAN2
		switch (can_rx.StdId)
		{
            case 0x201://摩擦1
                MOTOR_CAN_RX_3508RM(&ALL_MOTOR.DJI_3508_Shoot_L.DATA, rx_data);
                break;

            case 0x202://摩擦2
                MOTOR_CAN_RX_3508RM(&ALL_MOTOR.DJI_3508_Shoot_R.DATA, rx_data);
                break;
			default:
				break;
        }
	}
}

void WHW_MCP2515_Callback(SPI_HandleTypeDef *hspi, uCAN_MSG *rxMessage)
{
    if (hspi->Instance == SPI2) {
        uint8_t CAN_SPI_2_Data[8];
        memcpy(CAN_SPI_2_Data, &rxMessage->array[6], 8);
        switch (rxMessage->frame.id)
        {
            case 0x201:

                break;
        }
    }
}


void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size){
	uint8_t *pData = huart->pRxBuffPtr;
	if (huart->Instance == USART3){
		if (Size == 18){
			DBUS_Resolved(DBUS_RX_DATA, &DBUS);
			__HAL_DMA_DISABLE_IT(huart3.hdmarx, DMA_IT_HT);
		}
	}
	if (huart->Instance == USART6){
		if (Size == 21){
			VT13_Resolved(VT13_RX_DATA, &VT13);
			__HAL_DMA_DISABLE_IT(huart6.hdmarx, DMA_IT_HT);
		}
	}
	if (huart->Instance == USART1){
		uint8_t *next_buf = (pData == Referee_Rx_Buf[0]) ? Referee_Rx_Buf[1] : Referee_Rx_Buf[0];
		HAL_UARTEx_ReceiveToIdle_DMA(huart, next_buf, REFEREE_RXFRAME_LENGTH);
		__HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT);//关闭 DMA 半传中断
		Referee_System_Frame_Update(pData,Size);
	}
}

