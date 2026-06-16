#include "All_Init.h"

//总控制结构体
CONTAL_Typedef RUI_V_CONTAL = { 0 };

//遥控相关变量
uint8_t DBUS_RX_DATA[18] = { 0 };
uint8_t VT13_RX_DATA[21]={0};
DBUS_Typedef DBUS = { 0 };
VT13_Typedef VT13={0};
//裁判系统相关变量

User_Data_T User_data;
uint8_t Referee_Rx_Buf[2][REFEREE_RXFRAME_LENGTH];

//测试
uint8_t RX[20];
TD_t TDDDD;
//电机
MOTOR_Typdef ALL_MOTOR;

//陀螺仪
pid_type_def imu_temp;
IMU_Data_t IMU_Data;

//运动控制监测结构体
RUI_ROOT_STATUS_Typedef RUI_ROOT_STATUS;

//视觉
uint8_t VisionRx[30];
uint8_t sd_v_buff[24];

//电容
CAPDATE_TYPDEF CAPDATE;

//电源电压
float voltage;

//功率限制
model_t model;

uint8_t flag2;


float vision_state;

DBUS_UNION_TypeDef RUI_V_DBUS_UNION = { 0 };
VT13_UNION_Typdef VT13_UNION = {0};
		
TYPEDEF_VISION VISION_V_DATA = {0};////普通视觉
// VisionRxDataUnion VISION_V_DATA ;////加预测视觉

float heat_state;
float huanchongnengliang;
uint8_t bulltspeed;
void Everying_Init(void)
{
    //DWT初始化
    DWT_Init(168);

    //CAN初始化
	CAN_Filter_Init();
	
	//MCP2515初始化
	flag2 = CANSPI_Initialize(&hspi2);

    //定时器初始化
	HAL_TIM_Base_Start_IT(&htim3);
	HAL_TIM_Base_Start_IT(&htim7);
	HAL_TIM_Base_Start_IT(&htim9);
    //蜂鸣器PWM初始化
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);

    HAL_DMA_DeInit(&hdma_usart1_rx);
    HAL_DMA_DeInit(&hdma_usart1_tx);
    HAL_DMA_Init(&hdma_usart1_rx);
    HAL_DMA_Init(&hdma_usart1_tx);
    HAL_UART_DMAStop(&huart1);
	HAL_UARTEx_ReceiveToIdle_DMA(&huart1, Referee_Rx_Buf[0], REFEREE_RXFRAME_LENGTH);//裁判系统串口
	__HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT);//关闭 DMA 半传中断
	
    HAL_DMA_DeInit(&hdma_usart3_rx);
    HAL_DMA_Init(&hdma_usart3_rx);
    HAL_UART_DMAStop(&huart3);
    HAL_UARTEx_ReceiveToIdle_DMA(&huart3,(uint8_t *)DBUS_RX_DATA,18);
	__HAL_DMA_DISABLE_IT(huart3.hdmarx, DMA_IT_HT);//关闭 DMA 半传中断

    HAL_DMA_DeInit(&hdma_usart6_rx);
    HAL_DMA_DeInit(&hdma_usart6_tx);
    HAL_DMA_Init(&hdma_usart6_rx);
    HAL_DMA_Init(&hdma_usart6_tx);
    HAL_UART_DMAStop(&huart6);
	HAL_UARTEx_ReceiveToIdle_DMA(&huart6, (uint8_t *)VT13_RX_DATA,sizeof(VT13_RX_DATA));  //重启开始DMA传输
	__HAL_DMA_DISABLE_IT(huart6.hdmarx, DMA_IT_HT);//关闭 DMA 半传中断

	//USB初始化
	MX_USB_DEVICE_Init();

    //蜂鸣器指示初始化完成
    TIM4->CCR3 = 50;
    HAL_Delay(500);
    TIM4->CCR3 = 0;

}

void StartDefaultTask(void const * argument)
{
	Everying_Init();
    int cnt = 0;
	for(;;)
	{
//		playSong_heimaojingzhang();
//		playSong_huluwa();
//		playSong_jiaofu();
        cnt++;
        LOGI("Hello world: %d", cnt);
        LOGE("This is an error message!");
        LOGW("This is a warning message!");
		osDelay(1);
	}
}

