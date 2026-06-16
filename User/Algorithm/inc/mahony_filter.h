/**
 * @file mahony_filter.h
 * @brief Mahony��̬�˲��㷨ͷ�ļ�
 * @author CaoKangqi
 * @date 2026/2/11
 */

#ifndef G4_FRAMEWORK_MAHONY_FILTER_H
#define G4_FRAMEWORK_MAHONY_FILTER_H

#include <math.h>
#include "All_Init.h"

typedef struct {
    float x;  // X��
    float y;  // Y��
    float z;  // Z��
} Axis3f;

/**
 * @brief ������ƽ����
 * @param x ����ֵ
 * @return ��ƽ��������ֵ
 */
float invSqrt(float x);

// ǰ������
struct MAHONY_FILTER_t;

/**
 * @brief Mahony�˲����ṹ��
 */
struct MAHONY_FILTER_t
{
    // �������
    float Kp, Ki;          // ����/��������
    float alpha;           // ���ٶȼƵ�ͨ�˲�ϵ��
    Axis3f acc_lpf;        // �洢�˲���ļ��ٶ�ֵ
    float dt;              // �������
    Axis3f  gyro, acc;     // ������/���ٶȼ�����
    Axis3f gyro_bias;      // ��������ƫ
    float acc_norm;

    // ���̲���
    float exInt, eyInt, ezInt;
    float q0, q1, q2, q3;
    float rMat[3][3];

    // �������
    float pitch, roll, yaw;
    float last_yaw,YawTotalAngle;
};

extern struct MAHONY_FILTER_t mahony_filter;

void mahony_init(struct MAHONY_FILTER_t *f, float Kp, float Ki, float alpha,float dt);
void mahony_input(struct MAHONY_FILTER_t *mahony_filter, Axis3f gyro, Axis3f acc);
void mahony_update(struct MAHONY_FILTER_t *mahony_filter,
                   float gx, float gy, float gz,
                   float ax, float ay, float az,float dt);
void mahony_output(struct MAHONY_FILTER_t *mahony_filter);
void RotationMatrix_update(struct MAHONY_FILTER_t *mahony_filter);

#endif //G4_FRAMEWORK_MAHONY_FILTER_H