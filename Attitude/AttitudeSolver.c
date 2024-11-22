#include "AttitudeSolver.h"
#include "MadgWick.h"
#include <math.h>

// 输出的姿态角（全局变量）


// 初始化函数
void AttitudeSolver_Init(float sample_frequency, float gain) {
    beta = gain;                 // 设置 Madgwick 算法增益
    q0 = 1.0f; q1 = 0.0f;        // 初始化四元数
    q2 = 0.0f; q3 = 0.0f;
}

// 使用加速度计和陀螺仪更新姿态
void AttitudeSolver_UpdateIMU(float gx, float gy, float gz, float ax, float ay, float az) {
    MadgwickAHRSupdateIMU(gx, gy, gz, ax, ay, az);
}

// 使用加速度计、陀螺仪和磁力计更新姿态
void AttitudeSolver_Update(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz) {
    MadgwickAHRSupdate(gx, gy, gz, ax, ay, az, mx, my, mz);
}

// 获取姿态角（欧拉角形式）
void AttitudeSolver_GetEulerAngles(float *roll, float *pitch, float *yaw, int count) {
    *roll = atan2f(2.0f * (q0 * q1 + q2 * q3), 1.0f - 2.0f * (q1 * q1 + q2 * q2)) * 57.2958f;  // 转换为角度
    *pitch = asinf(2.0f * (q0 * q2 - q3 * q1)) * 57.2958f;
    *yaw = atan2f(2.0f * (q0 * q3 + q1 * q2), 1.0f - 2.0f * (q2 * q2 + q3 * q3)) * 57.2958f;
		*yaw = *yaw - (K * count + B); // 线性回归矫正
}
