#include "Robot.h"
#include <math.h>

#include "All_Init.h"

//==============================================================================
// 小陀螺 & 陀螺行进 参数
//==============================================================================
#define CHASSIS_SPIN_SPEED_DPS   450.0f   // 底盘自旋目标角速度 (deg/s)
#define CHASSIS_SPIN_KP          10.0f    // 自旋角速度 PID Kp
#define CHASSIS_SPIN_KI          0.02f    // 自旋角速度 PID Ki
#define CHASSIS_SPIN_KD          0.0f     // 自旋角速度 PID Kd
#define CHASSIS_SPIN_PID_MAXOUT  16384.0f  // 自旋 PID 最大输出 (mA)
#define CHASSIS_SPIN_PID_MAXI    2000.0f  // 自旋 PID 最大积分输出

/* 底盘物理参数 (与 Bottom.c MecanumInit 保持一致) */
#define CHASSIS_WHEELBASE       300.0f   // 轴距 (mm)
#define CHASSIS_WHEELTRACK      300.0f   // 轮距 (mm)
#define CHASSIS_HALF_SUM        ((CHASSIS_WHEELBASE + CHASSIS_WHEELTRACK) / 2.0f)  // (L+R)/2
/* 旋转项缩放系数: raid = half_sum / 57.3 (= half_sum * π/180), 与 Bottom.c 中 MecanumInit 一致 */
#define CHASSIS_RAID            (CHASSIS_HALF_SUM / 57.3f)

/* 速度换算系数：遥控器通道 -> mm/s 或 deg/s */
#define RC_CHASSIS_SPEED_RATIO  10.0f    // 通道 660 满量程 → 约 6600 对应速度单位

/* 自旋 PID 输出→角速度修正换算系数 (mA → deg/s) */
#define SPIN_PID_WZ_SCALE       (60.0f / CHASSIS_SPIN_PID_MAXOUT)  // 6000mA → 60deg/s 修正

/* 云台自稳参数 */
#define GIMBAL_YAW_MIT_KP        80.0f    // MIT 模式位置刚度 Kp
#define GIMBAL_YAW_MIT_KD        0.8f     // MIT 模式速度阻尼 Kd
#define GIMBAL_YAW_CAN_ID        0x308    // 云台 yaw DM4310 的 CAN ID
#define GIMBAL_PITCH_CAN_ID      0x309    // 云台 pitch DM4310 的 CAN ID
#define GIMBAL_PITCH_MIT_KP      80.0f    // 云台 pitch MIT 模式位置刚度 Kp
#define GIMBAL_PITCH_MIT_KD      0.8f     // 云台 pitch MIT 模式速度阻尼 Kd

float monitor_X;
float monitor_Y;
float monitor_W;
float yaw_TD;
float pitch_TD;
float VISION_connect;

/* ---- 自旋控制静态变量 ---- */
static PID_t  chassis_spin_pid;      // 底盘偏航角速度环 PID
static uint8_t       spin_pid_inited = 0;
static uint8_t       spin_mode       = 0;  // 0=底盘跟随云台  1=小陀螺(自旋+平移)

/* ---- 底盘跟随模式 PID ---- */
static PID_t  chassis_follow_pid;    // 底盘跟随云台 yaw 角 PID
static uint8_t       follow_pid_inited = 0;
#define CHASSIS_FOLLOW_KP   35.0f   // 跟随比例增益 (将角度误差转为角速度指令)
#define CHASSIS_FOLLOW_KI   0.0f
#define CHASSIS_FOLLOW_KD   0.0f
#define CHASSIS_FOLLOW_MAX  3000.0f   // 跟随 PID 最大输出 (deg/s)

//==============================================================================
// @brief  麦克纳姆轮运动学解算 — 将云台系速度指令变换到底盘系并解算轮速
//
//  输入:
//    vx_g, vy_g  — 云台坐标系下的平移速度指令 (操作手第一人称视野)
//                  vx>0=前进, vy>0=右移
//    wz_dps      — 底盘目标偏航角速度 (deg/s, >0=顺时针)
//    rel_deg     — 云台相对底盘的偏航角 (deg): rel = θ_gimbal − θ_chassis
//                  rel>0 表示云台在底盘顺时针方向
//  输出:
//    wheel_speed[4] — 四轮目标速度 [RF, LF, LR, RR]，与 Bottom.c 映射一致
//
//  坐标系变换原理:
//    操作手指令 v_g 定义在云台坐标系中（操作手通过图传看到的方向）。
//    底盘在旋转，需要将 v_g 变换到底盘坐标系才能套用麦轮运动学:
//
//      世界系:   v_w = R(θ_g) · v_g           (云台系 → 世界系)
//      底盘系:   v_c = R(−θ_c) · v_w          (世界系 → 底盘系)
//
//    合并:  v_c = R(−θ_c)·R(θ_g)·v_g = R(θ_g − θ_c)·v_g = R(rel)·v_g
//
//    其中 R(θ) = [cosθ, −sinθ; sinθ,  cosθ]  (标准二维逆时针旋转)
//    展开:  vx_c = cos(rel)·vx_g − sin(rel)·vy_g
//            vy_c = sin(rel)·vx_g + cos(rel)·vy_g
//
//  麦轮逆运动学 (与 Bottom.c MecanumResolve 一致):
//    RF: −Vx + Vy + W_term     LF: +Vx + Vy + W_term
//    LR: +Vx − Vy + W_term     RR: −Vx − Vy + W_term
//==============================================================================
static void Chassis_Resolve(float vx_g, float vy_g,
                            float wz_dps, float rel_deg,
                            float wheel_speed[4])
{
    float rel_rad = rel_deg * 0.0174532925f;  // deg → rad
    float cos_r = cosf(rel_rad);
    float sin_r = sinf(rel_rad);

    /* 云台系 → 底盘系: v_c = R(rel) · v_g */
    float vx_c = cos_r * vx_g - sin_r * vy_g;
    float vy_c = sin_r * vx_g + cos_r * vy_g;

    /* 旋转项: wz (deg/s) 转换为平移速度同单位的量
     * W_term = wz * (L+W)/2 / 57.3°，与 Bottom.c MecanumInit 一致 */
    float W_term = wz_dps * CHASSIS_RAID;

    /* 麦轮逆运动学 — 符号与 Bottom.c MecanumResolve 一致 */
    wheel_speed[0] = -vx_c + vy_c + W_term;   // 右前 RF (0x201)
    wheel_speed[1] =  vx_c + vy_c + W_term;   // 左前 LF (0x202)
    wheel_speed[2] =  vx_c - vy_c + W_term;   // 左后 LR (0x203)
    wheel_speed[3] = -vx_c - vy_c + W_term;   // 右后 RR (0x204)
}

void RobotTask(uint8_t mode,
               DBUS_Typedef *DBUS,
               CONTAL_Typedef *CONTAL,
               User_Data_T *User_data,
               CAPDATE_TYPDEF *CAP_DATA,
               TYPEDEF_VISION *Vision/* 普通视觉*/
							/*	VisionRxDataUnion *Vision 加预测视觉*/,
               RUI_ROOT_STATUS_Typedef *Root,
               MOTOR_Typdef *MOTOR,
               IMU_Data_t *IMU_Data,
							 TD_t *TDDD,
							VT13_Typedef* VT13_DBUS)
{
    switch (mode) {

        case 1: // 底盘
        {
            /*--------------------------------------------------------------
             *  底盘模式切换
             *  S2 == 1 (上位): 底盘跟随云台模式  — 底盘 yaw 跟随云台
             *  S2 == 3 (中位): 小陀螺模式         — 自旋 + 云台第一人称平移
             *  S2 == 2 (下位): 小陀螺模式
             *--------------------------------------------------------------*/
            spin_mode = (DBUS->Remote.S2 == 3 || DBUS->Remote.S2 == 2) ? 1 : 0;

            /* ---- 首次进入小陀螺模式时初始化自旋 PID ---- */
            if (spin_mode == 1 && !spin_pid_inited)
            {
                float kpid[3] = { CHASSIS_SPIN_KP, CHASSIS_SPIN_KI, CHASSIS_SPIN_KD };
                PID_Init(&chassis_spin_pid,
                         CHASSIS_SPIN_PID_MAXOUT,   // max_out
                         CHASSIS_SPIN_PID_MAXI,     // intergral_limit
                         kpid,                      // {Kp, Ki, Kd}
                         0.0f, 0.0f,                // CoefA, CoefB
                         0.0f,                      // output_lpf_rc
                         0.0f,                      // derivative_lpf_rc
                         0,                         // ols_order
                         Integral_Limit);           // improve
                spin_pid_inited = 1;
            }

            /* ---- 首次进入跟随模式时初始化跟随 PID ---- */
            if (spin_mode == 0 && !follow_pid_inited)
            {
                float kpid_follow[3] = { CHASSIS_FOLLOW_KP, CHASSIS_FOLLOW_KI, CHASSIS_FOLLOW_KD };
                PID_Init(&chassis_follow_pid,
                         CHASSIS_FOLLOW_MAX,             // max_out
                         CHASSIS_FOLLOW_MAX * 0.3f,      // intergral_limit
                         kpid_follow,                    // {Kp, Ki, Kd}
                         0.0f, 0.0f,                     // CoefA, CoefB
                         0.0f,                           // output_lpf_rc
                         0.0f,                           // derivative_lpf_rc
                         0,                              // ols_order
                         Integral_Limit);                // improve
                follow_pid_inited = 1;
            }

            /* ---- 读取云台相对底盘的偏航角 (单位: 度, float 精度) ----
             * 来源: DM4310 云台 yaw 电机编码器
             * rel = θ_gimbal − θ_chassis，rel>0 表示云台在底盘顺时针方向
             * 两种模式共用: 跟随模式下作为 PID 的被控量；
             * 小陀螺模式下作为云台系→底盘系旋转矩阵的角度 */
            float relative_angle = ALL_MOTOR.m_dm4310_y_t.DATA.ralativeAngle;
            CONTAL->CG.RELATIVE_ANGLE = (int16_t)relative_angle;  // 同步到 CONTAL

            /* ---- 根据模式计算速度指令 (云台坐标系) ---- */
            float vx_cmd, vy_cmd, wz_cmd;

            if (spin_mode == 0)
            {
                /* 底盘跟随模式: 跟随 PID → 底盘角速度指令
                 * PID_Calculate(measure, ref): error = ref − measure = −relative_angle
                 * 取反使 wz_cmd 方向正确: rel>0 时底盘朝云台方向旋转 */
                float follow_raw = PID_Calculate(&chassis_follow_pid,
                                                 relative_angle, 0.0f);
                wz_cmd = -follow_raw;

                /* 遥控器摇杆输入 — 操作手通过图传看到的方向 (云台系) */
                vx_cmd = (float)DBUS->Remote.CH2 * RC_CHASSIS_SPEED_RATIO / 660.0f;
                vy_cmd = (float)DBUS->Remote.CH3 * RC_CHASSIS_SPEED_RATIO / 660.0f;
            }
            else
            {
                /* 小陀螺模式: 自旋角速度闭环 PID (deg/s 环)
                 * 测量值 = IMU gyro[2] (rad/s → deg/s)，目标 = CHASSIS_SPIN_SPEED_DPS
                 * PID 输出量纲为 mA，通过 SPIN_PID_WZ_SCALE 换算为 deg/s 修正量 */
                float yaw_rate_dps = IMU_Data->gyro[2] * 57.2957795f;
                float spin_pid_out = PID_Calculate(&chassis_spin_pid,
                                                   yaw_rate_dps,

                                                   CHASSIS_SPIN_SPEED_DPS);
                wz_cmd = CHASSIS_SPIN_SPEED_DPS + spin_pid_out * SPIN_PID_WZ_SCALE;
                vx_cmd = (float)DBUS->Remote.CH2 * RC_CHASSIS_SPEED_RATIO / 660.0f;
                vy_cmd = (float)DBUS->Remote.CH3 * RC_CHASSIS_SPEED_RATIO / 660.0f;
                /* ---- 云台自稳 (小陀螺模式) ----
                 * 用底盘陀螺仪测量底盘角速度，云台 yaw/pitch 轴电机输出反向运动，
                 * 保证云台相对世界坐标系不变。
                 *
                 * Yaw 轴:
                 *   位置目标 = 当前相对角度 (跟随操作手意图，不主动改变 yaw 指向)
                 *   速度前馈 = -gyro[2] (rad/s) → 抵消底盘偏航旋转
                 *
                 * Pitch 轴:
                 *   位置目标 = motor0 - (IMU_pitch - IMU_pitch0)
                 *     - 进入小陀螺时记录 IMU 绝对俯仰角 pitch0 和电机初始角 motor0
                 *     - 底盘上坡 Δθ° → IMU_pitch 增加 Δθ° → 电机目标 = motor0 - Δθ°
                 *     - 电机反向转动使云台保持水平，MIT Kp 提供对抗重力的保持力矩
                 *   速度前馈 = -gyro[1] (rad/s) → 抵消底盘俯仰旋转
                 *
                 * 控制方式: MIT 模式 — 电机内部 PD 环以 10~40kHz 高频执行
                 *   力矩 = Kp·(pos_target - pos) + Kd·(vel_target - vel)                */
                /* ---- 读取传感器数据 ---- */
                float yaw_current_deg   = ALL_MOTOR.m_dm4310_y_t.DATA.ralativeAngle; // 度
                float pitch_current_deg = ALL_MOTOR.m_dm4310_p_t.DATA.ralativeAngle; // 度
                float chassis_wz_radps  = IMU_Data->gyro[2];  // rad/s, 底盘偏航角速度
                float chassis_wy_radps  = IMU_Data->gyro[1];  // rad/s, 底盘俯仰角速度
                /* 速度前馈 (rad/s): 电机输出反向角速度抵消底盘旋转 */
                float yaw_vel_ff_radps   = -chassis_wz_radps;
                float pitch_vel_ff_radps = -chassis_wy_radps;
                /* ---- 首次进入时初始化 MIT 模式并记录 pitch 参考值 ---- */
                static uint8_t  gimbal_stab_inited  = 0;
                static float    pitch_ref_imu_deg   = 0.0f; // 进入时 IMU 绝对 pitch (度)
                static float    pitch_ref_motor_deg = 0.0f; // 进入时 pitch 电机相对角 (度)
                if (!gimbal_stab_inited)
                {
                    motor_mode(&hcan1, GIMBAL_YAW_CAN_ID,   MIT_MODE, DM_CMD_MOTOR_MODE);
                    motor_mode(&hcan1, GIMBAL_PITCH_CAN_ID, MIT_MODE, DM_CMD_MOTOR_MODE);
                    pitch_ref_imu_deg   = IMU_Data->pitch;      // 记录初始 IMU 绝对俯仰角 (度)
                    pitch_ref_motor_deg = pitch_current_deg;     // 记录初始电机相对角 (度)
                    gimbal_stab_inited  = 1;
                }

                /* ---- Yaw 轴 MIT 控制: 位置=当前角, 速度=前馈 ---- */
                float yaw_target_rad = yaw_current_deg * 0.0174532925f;  // deg → rad
                if (Root->MOTOR_HEAD_Yaw == RUI_DF_ONLINE)
                {
                    PID_Calculate(&MOTOR->m_dm4310_y_t.PID_P,yaw_current_deg,yaw_target_rad);
                    PID_Calculate(&MOTOR->m_dm4310_y_t.PID_S,yaw_vel_ff_radps,MOTOR->m_dm4310_y_t.PID_P.Output);
                    mit_ctrl(&hcan1, GIMBAL_YAW_CAN_ID,0, 0,0, 0,MOTOR->m_dm4310_y_t.PID_S.Output );

                }
                /* ---- Pitch 轴 MIT 控制: 基于 IMU 绝对俯仰角计算目标位置 ----
                 * 例: 底盘上坡 15° → IMU pitch 从 2° 变到 17° → Δ = 15°
                 *      电机目标 = motor0 - 15° → 电机相对底盘转 -15° → 云台保持水平  */
                float imu_pitch_delta_deg = IMU_Data->pitch - pitch_ref_imu_deg;       // 底盘俯仰变化量 (度)
                float pitch_target_deg    = pitch_ref_motor_deg - imu_pitch_delta_deg;  // 补偿后目标角 (度)
                float pitch_target_rad    = pitch_target_deg * 0.0174532925f;           // deg → rad
                if (Root->MOTOR_HEAD_Pitch == RUI_DF_ONLINE)
                {
                    PID_Calculate(&MOTOR->m_dm4310_p_t.PID_P,pitch_current_deg,pitch_target_rad);
                    PID_Calculate(&MOTOR->m_dm4310_p_t.PID_S,pitch_vel_ff_radps,MOTOR->m_dm4310_p_t.PID_P.Output);
                    mit_ctrl(&hcan1, GIMBAL_PITCH_CAN_ID,0, 0,0, 0,MOTOR->m_dm4310_p_t.PID_S.Output );
                }

                /* 存入 CONTAL 供其他模块读取 */
                CONTAL->HEAD.Yaw   = yaw_current_deg;
                CONTAL->HEAD.Pitch = pitch_current_deg;

            }

            /* ---- 麦轮逆运动学解算 (云台系 → 底盘系 → 四轮速度) ----
             * Chassis_Resolve 内部用 R(relative_angle) 将云台系速度旋转到底盘系，
             * 再套用与 Bottom.c 一致的麦轮逆运动学公式 */
            float target_wheel[4];
            Chassis_Resolve(vx_cmd, vy_cmd, wz_cmd, relative_angle, target_wheel);

            /* 存入 CONTAL 供其他模块读取 */
            CONTAL->BOTTOM.wheel1 = target_wheel[0];
            CONTAL->BOTTOM.wheel2 = target_wheel[1];
            CONTAL->BOTTOM.wheel3 = target_wheel[2];
            CONTAL->BOTTOM.wheel4 = target_wheel[3];
            CONTAL->BOTTOM.VX    = vx_cmd;
            CONTAL->BOTTOM.VY    = vy_cmd;
            CONTAL->BOTTOM.VW    = wz_cmd;

        } break;

        case 2: // 云台
        {

        } break;

        case 3://电容
        {

        } break;

        case 4://发射
        {
					
        } break;
    }
}

/************************************************************万能分隔符**************************************************************
 * 	@author:			//瑞
 *	@performance:	    //
 *	@parameter:		    //
 *	@time:				//24-2-25 下午1:24
 *	@ReadMe:			//获取摩擦轮的目标值
 ************************************************************万能分隔符**************************************************************/
float Shoot_Speed_P(float Kp, float measure, float ref, float OUT_Lim)
{
    float error = ref - measure;
    /*比例输出*/
    float ALL_Out = error * Kp;
    /*总输出限幅*/
    ALL_Out = MATH_Limit_float(OUT_Lim, -OUT_Lim, ALL_Out);

    return ALL_Out;
}

float RUI_F_GET_FIRE_WIPE_SPEED(CONTAL_Typedef *CONTAL, DBUS_Typedef *DBUS, User_Data_T *User_data, RUI_ROOT_STATUS_Typedef *Root)
{
    static uint8_t LOCK = 0, MOD = 0, KEYBOARD_LOCK = 0, JUDGE_LOCK = 0;
    static float AIM = 0.0f, TEMP = 0.0f, SPEED_NOW = 0.0f, SPEED_LAST = 0.0f;
    MOD = DBUS->Remote.S2-1 | DBUS->Mouse.L_State | LOCK;
    //停止
    if (MOD == 0 || DBUS->Remote.S2-1 == 1 || Root->RM_DBUS == RUI_DF_OFFLINE)
    {
        AIM = 0.0f;//0.0
        LOCK = 0;
    } else
    {
        LOCK = 1;
        SPEED_LAST = SPEED_NOW;
        SPEED_NOW = User_data->shoot_data.initial_speed;

        /*弹速PID,仅限于修正摩擦轮发热所带来的弹速变化,优化为打一发弹算一次*/
        if (CONTAL->SHOOT_Bask.Shoot_Number - CONTAL->SHOOT_Bask.Shoot_Number_Last > 0)
        {
            if(MATH_ABS_float(SPEED_NOW - SPEED_LAST) > 0.1f)
                TEMP = Shoot_Speed_P(5.0f, SPEED_NOW, 29.5f, 50);
            CONTAL->SHOOT_Bask.Shoot_Number_Last = CONTAL->SHOOT_Bask.Shoot_Number;
        }

        AIM = (float) CONTAL->SHOOT.Shoot_Speed + TEMP;

    }
    return AIM;
}


/************************************************************万能分隔符**************************************************************
 * 	@author:			//瑞
 *	@performance:	    //
 *	@parameter:		    //
 *	@time:				//24-5-8 上午9:44
 *	@ReadMe:			//获取最大目标值
 ************************************************************万能分隔符**************************************************************/
float RUI_F_CHASSIS_GET_MAX_TARGET(float MAX_POWER)
{
    // 200w 0.04f
    // 100w 0.065f
    //  90w 0.065f
    //  80w 0.07f
    //  75w 0.08f
    //  70w 0.09f
    //  65w 0.1f
    //  60w 0.08f
    //  55w 0.06f
    //  50w 0.04f
    //  45w 0.02f
    if (MAX_POWER == 45)
    {
        return 0.03f * MAX_POWER;
    } else if (MAX_POWER == 50 || MAX_POWER == 200)
    {
        return 0.04f * MAX_POWER;
    } else if (MAX_POWER == 55)
    {
        return 0.06f * MAX_POWER;
    } else if (MAX_POWER == 60 || MAX_POWER == 75)
    {
        return 0.08f * MAX_POWER;
    } else if (MAX_POWER == 65)
    {
        return 0.1f * MAX_POWER;
    } else if (MAX_POWER == 70)
    {
        return 0.09f * MAX_POWER;
    } else if (MAX_POWER == 80)
    {
        return 0.07f * MAX_POWER;
    } else if (MAX_POWER == 90 || MAX_POWER == 100)
    {
        return 0.065f * MAX_POWER;
    } else
    {
        return 0.1f * MAX_POWER;
    }
}

/************************************************************万能分隔符**************************************************************
 * 	@author:			//瑞
 *	@performance:	    //底盘走直线单环PID
 *	@parameter:		    //
 *	@time:				//23-12-17 18:08
 *	@ReadMe:			//
 ************************************************************万能分隔符**************************************************************/
float RUI_F_CHASSIS_PID(int16_t RELATIVE_ANGLE, float KP, float KI, float KD)
{
    static float INTEGRAL = 0.0;
    float ERROR[2] = { 0 }, DERIVATIVE;
    ERROR[ 1 ] = (float) RELATIVE_ANGLE;
    //积分
    INTEGRAL += ( ERROR[ 1 ] * KI );
    INTEGRAL = MATH_Limit_float(100, -100, INTEGRAL);

    //微分
    DERIVATIVE = ( ERROR[ 1 ] - ERROR[ 0 ] ) * KD;

    ERROR[ 0 ] = ERROR[ 1 ];
    float OUTPUT = MATH_Limit_float(3000, -3000, ( KP * ERROR[ 1 ] + INTEGRAL + DERIVATIVE ));

    return OUTPUT;
}
