#include "IMU_Task.h"
#include "mahony_filter.h"
#include <math.h>
#include <string.h>

#define correct_Time_define 1000    // 上电去0飘 5000次取平均
#define temp_times 300              // 探测温度阈值
#define Destination_TEMPERATURE 28.375f

static uint32_t temp_Ticks = 0;

IMU_CTRL_STATE_e imu_ctrl_state = TEMP_INIT; // 当前控制状态
IMU_CTRL_FLAG_t  imu_ctrl_flag  = {0};       // 控制状态标志

/**
  * @brief          bmi088温度控制
  * @param[in]      argument: NULL
  * @retval         none
  */
void IMU_Temperature_Ctrl(IMU_Data_t *IMU, pid_type_def *imu_temp_pid)
{
    uint16_t tempPWM;
    // pid calculate. PID计算
    PID_calc(imu_temp_pid, IMU->temp, Destination_TEMPERATURE);
    if (imu_temp_pid->out < 0.0f)
    {
       imu_temp_pid->out = 0.0f;
    }
    tempPWM = (uint16_t)imu_temp_pid->out;
    SPI1_imu_pwm_set(tempPWM);
}

/**
 * @brief 陀螺仪零偏校准任务 (剥离封装，匹配目标架构)
 */
void IMU_Gyro_Zero_Calibration_Task(IMU_Data_t *IMU)
{
    // 用于计算方差的静态累加器
    static float gyro_sq_sum[3] = {0};
    static float accel_sq_sum[3] = {0};
    static float accel_sum[3] = {0}; // 仅用于判定稳定性，不修改原加速度输出

    if (imu_ctrl_flag.gyro_calib_done) return;

#ifdef DEBUG_MODE
    // 1. 累加数据与平方和
    for (int i = 0; i < 3; i++)
    {
        IMU->gyro_correct[i] += IMU->gyro[i];
        accel_sum[i]         += IMU->accel[i];

        gyro_sq_sum[i]  += IMU->gyro[i] * IMU->gyro[i];
        accel_sq_sum[i] += IMU->accel[i] * IMU->accel[i];
    }

    IMU->correct_times++;

    if (IMU->correct_times < correct_Time_define) return;

    const float div = 1.0f / (float)correct_Time_define;
    uint8_t is_stable = 1;

    for (int i = 0; i < 3; i++)
    {

        float mean_g = IMU->gyro_correct[i] * div;
        float mean_a = accel_sum[i] * div;
        // 方差公式：Var = E(x²) - (E(x))²
        float gyro_var  = (gyro_sq_sum[i] * div) - (mean_g * mean_g);
        float accel_var = (accel_sq_sum[i] * div) - (mean_a * mean_a);
        // 判定阈值，如果超过阈值（如0.01），认为云台在晃动，数据不稳定
        if (gyro_var > 0.01f || accel_var > 0.01f)
        {
            is_stable = 0;
            break;
        }
    }

    if (is_stable)
    {
        for (int i = 0; i < 3; i++)
        {
            IMU->gyro_correct[i] *= div;
        }
        imu_ctrl_flag.gyro_calib_done = 1;
    }
    else
    {
        for (int i = 0; i < 3; i++)
        {
            IMU->gyro_correct[i] = 0.0f;
            accel_sum[i]         = 0.0f;
            gyro_sq_sum[i]       = 0.0f;
            accel_sq_sum[i]      = 0.0f;
        }
        IMU->correct_times = 0; // 重置计数器
    }
#endif

// 获得零飘直接赋值 (Release 模式不进行动态校准)
#ifdef RELEASE_MODE
    IMU->gyro_correct[0] = 0.000710101391f;
    IMU->gyro_correct[1] = 0.00253147446f;
    IMU->gyro_correct[2] = -0.00386138656f;
    imu_ctrl_flag.gyro_calib_done = 1;
#endif
}

/**
 * @brief IMU核心任务 (以状态机重构运行流)
 */
void INS_Task(IMU_Data_t *IMU, pid_type_def *imu_temp_pid)
{
    static uint32_t count = 0;

    // 1000Hz ins update 获取底层数据
    if ((count % 1) == 0)
    {
        BMI088_read(IMU->gyro, IMU->accel, &IMU->temp);
    }

    // 100Hz temperature control & 温度状态机跳转判断
    if ((count % 10) == 0)
    {
        if (imu_ctrl_state != TEMP_INIT && imu_ctrl_state != ERROR_STATE)
        {
            // 执行100hz的温度控制PID
            IMU_Temperature_Ctrl(IMU, imu_temp_pid);
        }

        switch (imu_ctrl_state)
        {
            case TEMP_PID_CTRL:
#ifdef DEBUG_MODE
                // 接近额定温度之差小于0.5° 开始稳定计数
                if(fabsf(IMU->temp - Destination_TEMPERATURE - 0.5f) < 1.0f)
                {
                    imu_ctrl_flag.temp_reached = 1;
                    imu_ctrl_state = TEMP_STABLE;
                }
#endif
#ifdef RELEASE_MODE
                imu_ctrl_flag.temp_reached = 1;
                imu_ctrl_state = TEMP_STABLE; // 快速初始化
#endif
                break;

            case TEMP_STABLE:
                temp_Ticks++;
                if(temp_Ticks > temp_times)   // 计数达到一定次数后说明温度已经稳定，进入0飘初始化
                {
                    imu_ctrl_flag.temp_stable = 1;
                    imu_ctrl_state = GYRO_CALIB;
                }
                break;

            default:
                break;
        }
    }

    // 1000Hz 核心执行状态机
    switch (imu_ctrl_state)
    {
        case TEMP_INIT:
            IMU->correct_times = 0;
            temp_Ticks = 0;
            IMU->gyro_correct[0] = 0.0f;
            IMU->gyro_correct[1] = 0.0f;
            IMU->gyro_correct[2] = 0.0f;
            memset(&imu_ctrl_flag, 0, sizeof(IMU_CTRL_FLAG_t));

            imu_ctrl_state = TEMP_PID_CTRL;
            break;

        case TEMP_PID_CTRL:
        case TEMP_STABLE:
            // 等待温度到达目标并稳定，此阶段暂不进行姿态解算
            break;

        case GYRO_CALIB:
            IMU_Gyro_Zero_Calibration_Task(IMU);
            if (imu_ctrl_flag.gyro_calib_done)
            {
                imu_ctrl_state = FUSION_RUN;
            }
            break;

        case FUSION_RUN:
            // 0.8-0.99
            QEKF_INS.accLPFcoef = 0.9f;

            // 减去陀螺仪0飘
            IMU->gyro[0] -= IMU->gyro_correct[0];
            IMU->gyro[1] -= IMU->gyro_correct[1];
            IMU->gyro[2] -= IMU->gyro_correct[2];

            //===========================================================================
            // ekf姿态解算部分
            IMU_QuaternionEKF_Update(
            IMU->gyro[0], IMU->gyro[1], IMU->gyro[2],
            IMU->accel[0], IMU->accel[1], IMU->accel[2]);

            mahony_update(&mahony_filter,
                IMU->gyro[0], IMU->gyro[1], IMU->gyro[2],
                IMU->accel[0], IMU->accel[1], IMU->accel[2], 0.001f);
            mahony_output(&mahony_filter);

            //===============================================================================
            // ekf获取姿态角度函数
            IMU->pitch = Get_Pitch(); // 获得pitch
            IMU->roll  = Get_Roll();  // 获得roll
            IMU->yaw   = Get_Yaw();   // 获得yaw
            IMU->YawTotalAngle = Get_YawTotalAngle();

            memcpy(IMU->q, QEKF_INS.q, 16);

            imu_ctrl_flag.fusion_enabled = 1;
            break;

        case ERROR_STATE:
            // 原代码无冗余异常监控，暂留接口
            break;

        default:
            break;
    }
    count++;
}