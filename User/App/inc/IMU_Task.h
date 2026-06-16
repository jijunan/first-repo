#ifndef __IMU_TASK_H
#define __IMU_TASK_H

#include "main.h"
#include "tim.h"
#include "QuaternionEKF.h"
#include "pid_temp.h"
#include "bsp_imu_pwm.h"
#include "kalman_filter.h"
#include "BMI088driver.h"

// 引入全新的状态机枚举
typedef enum
{
	TEMP_INIT = 0,   // 温控状态初始化，初始化变量、清零
	TEMP_PID_CTRL,   // PID 控制加热阶段
	TEMP_STABLE,     // 温度稳定判定状态
	GYRO_CALIB,      // 陀螺仪校准阶段
	FUSION_RUN,      // 姿态融合正常运行状态
	ERROR_STATE      // 错误状态（原逻辑暂不涉及，保留框架）
} IMU_CTRL_STATE_e;

// 状态机标志位
typedef struct
{
	uint8_t temp_reached;      // 到达目标温度
	uint8_t temp_stable;       // 温度稳定
	uint8_t gyro_calib_done;   // 陀螺仪零漂完成
	uint8_t fusion_enabled;    // 融合算法使能
} IMU_CTRL_FLAG_t;

typedef struct
{
	float gyro_correct[3];
	float gyro[3];
	float accel[3];
	float temp;

	float q[4];

	float pitch;
	float roll;
	float yaw;
	float YawTotalAngle;
	float guolingjiesuan_yaw;

	// 原态度标志位 attitude_flag 已被外部状态机 imu_ctrl_state 替代
	uint32_t correct_times;
} IMU_Data_t;

extern IMU_CTRL_STATE_e imu_ctrl_state;
extern IMU_CTRL_FLAG_t imu_ctrl_flag;

void IMU_Temperature_Ctrl(IMU_Data_t *IMU, pid_type_def *imu_temp_pid);
void IMU_Gyro_Zero_Calibration_Task(IMU_Data_t *IMU);
void INS_Task(IMU_Data_t *IMU, pid_type_def *imu_temp_pid);

#endif