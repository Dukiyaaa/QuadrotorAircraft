#include "Anonymity.h"
#include "MySerial.h"
#include "MPU6050.h" // 引入MPU6050相关函数头文件

#define BYTE0(dwTemp)  (*(char*)(&dwTemp))
#define BYTE1(dwTemp)  (*((char*)(&dwTemp) + 1))
// 外部变量声明
//extern float Roll, Pitch, Yaw;
//extern quaternion q;
//extern volatile float q0, q1, q2, q3;

/**
 * @brief 发送六个轴数据 (ID: 0x01, 19字节)
 * @param ax 加速度计 X 轴数据指针
 * @param ay 加速度计 Y 轴数据指针
 * @param az 加速度计 Z 轴数据指针
 * @param gx 陀螺仪 X 轴数据指针
 * @param gy 陀螺仪 Y 轴数据指针
 * @param gz 陀螺仪 Z 轴数据指针
 */
void SendToAno01(float ax, float ay, float az, float gx, float gy, float gz) {
    unsigned char ANO_BUFF[19];

    // 转换为整数格式发送（放大1000倍）
    int16_t AccX = (int16_t)(ax * 1000);
    int16_t AccY = (int16_t)(ay * 1000);
    int16_t AccZ = (int16_t)(az * 1000);
    int16_t GyroX = (int16_t)(gx * 1000);
    int16_t GyroY = (int16_t)(gy * 1000);
    int16_t GyroZ = (int16_t)(gz * 1000);

    ANO_BUFF[0] = 0xAA; // 帧头
    ANO_BUFF[1] = 0xFF;
    ANO_BUFF[2] = 0x01; // 功能字
    ANO_BUFF[3] = 13;   // 数据长度

    // 数据部分
    ANO_BUFF[4] = AccX >> 8;
    ANO_BUFF[5] = AccX & 0xff;
    ANO_BUFF[6] = AccY >> 8;
    ANO_BUFF[7] = AccY & 0xff;
    ANO_BUFF[8] = AccZ >> 8;
    ANO_BUFF[9] = AccZ & 0xff;
    ANO_BUFF[10] = GyroX >> 8;
    ANO_BUFF[11] = GyroX & 0xff;
    ANO_BUFF[12] = GyroY >> 8;
    ANO_BUFF[13] = GyroY & 0xff;
    ANO_BUFF[14] = GyroZ >> 8;
    ANO_BUFF[15] = GyroZ & 0xff;
    ANO_BUFF[16] = 0x01; // 震动状态（占位）

    // 校验部分
    int8_t sumcheck = 0;
    int8_t addcheck = 0;
    for (int i = 0; i < ANO_BUFF[3] + 4; i++) {
        sumcheck += ANO_BUFF[i];
        addcheck += sumcheck;
    }
    ANO_BUFF[17] = sumcheck;
    ANO_BUFF[18] = addcheck;

    // 逐个发送
    for (int i = 0; i < 19; i++) {
        printf("%c", ANO_BUFF[i]);
    }
}

/**
* @brief 发送姿态角数据 (ID: 0x03, 13字节) 单位为°
 * @param Roll 指向 Roll 角数据的指针
 * @param Pitch 指向 Pitch 角数据的指针
 * @param Yaw 指向 Yaw 角数据的指针
 */
void SendToAno03(float Roll, float Pitch, float Yaw) {
    unsigned char ANO_BUFF[13];
		int rate = 100;
    // 转换为整数格式发送（放大100倍）
    int16_t RollM = (int16_t)(Roll * rate);
    int16_t PitchM = (int16_t)(Pitch * rate);
    int16_t YawM = (int16_t)(Yaw * rate);
	
    ANO_BUFF[0] = 0xAA; // 帧头
    ANO_BUFF[1] = 0xFF;
    ANO_BUFF[2] = 0x03; // 功能字
    ANO_BUFF[3] = 7;    // 数据长度

    // 数据部分 低字节在前高字节在后

		ANO_BUFF[4] = BYTE0(RollM);
		ANO_BUFF[5] = BYTE1(RollM);
		ANO_BUFF[6] = BYTE0(PitchM);
		ANO_BUFF[7] = BYTE1(PitchM);    
		ANO_BUFF[8] = BYTE0(YawM);
		ANO_BUFF[9] = BYTE1(YawM);

//    ANO_BUFF[4] = RollM & 0xff;
//    ANO_BUFF[5] = (RollM >> 8) & 0xff;
//    ANO_BUFF[6] = PitchM & 0xff;    
//    ANO_BUFF[7] = (PitchM >> 8) & 0xff;
//    ANO_BUFF[8] = YawM & 0xff;
//    ANO_BUFF[9] = (YawM >> 8) & 0xff;

		
    ANO_BUFF[10] = 0x01; // 融合姿态标志（占位）

    // 校验部分
    int8_t sumcheck = 0;
    int8_t addcheck = 0;
    for (int i = 0; i < ANO_BUFF[3] + 4; i++) {
        sumcheck += ANO_BUFF[i];
        addcheck += sumcheck;
    }
    ANO_BUFF[11] = sumcheck;
    ANO_BUFF[12] = addcheck;

    // 逐个发送
    for (int i = 0; i < 13; i++) {
        printf("%c", ANO_BUFF[i]);
    }
}


/**
 * @brief 发送姿态四元数数据 (ID: 0x04, 15字节)
 * @param q0 四元数的第0个分量
 * @param q1 四元数的第1个分量
 * @param q2 四元数的第2个分量
 * @param q3 四元数的第3个分量
 */
void SendToAno04(float q0,float q1,float q2,float q3) {
		printf("q0: %.2f, q1: %.2f, q2: %.2f, q3: %.2f\n", q0, q1, q2, q3);
    unsigned char ANO_BUFF[15];

    // 转换为整数格式发送（放大10000倍）
//    int16_t Q0 = (int16_t)(q0 * 10000);
//    int16_t Q1 = (int16_t)(q1 * 10000);
//    int16_t Q2 = (int16_t)(q2 * 10000);
//    int16_t Q3 = (int16_t)(q3 * 10000);
		
	  int16_t Q0 = (int16_t)(q0);
    int16_t Q1 = (int16_t)(q1);
    int16_t Q2 = (int16_t)(q2);
    int16_t Q3 = (int16_t)(q3);
	
		printf("Q0: %d, Q1: %d, Q2: %d, Q3: %d\n", Q0, Q1, Q2, Q3);
    ANO_BUFF[0] = 0xAA; // 帧头
    ANO_BUFF[1] = 0xFF;
    ANO_BUFF[2] = 0x04; // 功能字
    ANO_BUFF[3] = 9;    // 数据长度

    // 数据部分
    ANO_BUFF[4] = Q0 >> 8;
    ANO_BUFF[5] = Q0 & 0xff;
    ANO_BUFF[6] = Q1 >> 8;
    ANO_BUFF[7] = Q1 & 0xff;
    ANO_BUFF[8] = Q2 >> 8;
    ANO_BUFF[9] = Q2 & 0xff;
    ANO_BUFF[10] = Q3 >> 8;
    ANO_BUFF[11] = Q3 & 0xff;

    ANO_BUFF[12] = 0x01; // 融合姿态标志（占位）

    // 校验部分
    int8_t sumcheck = 0;
    int8_t addcheck = 0;
    for (int i = 0; i < ANO_BUFF[3] + 4; i++) {
        sumcheck += ANO_BUFF[i];
        addcheck += sumcheck;
    }
    ANO_BUFF[13] = sumcheck;
    ANO_BUFF[14] = addcheck;

//    // 逐个发送
//    for (int i = 0; i < 15; i++) {
//        printf("%c", ANO_BUFF[i]);
//    }
}

