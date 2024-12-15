#include "Receiver.h"
#include "tim.h"
#include "MySerial.h"

// 数据存储
static uint32_t risingEdgeTime[CHANNEL_COUNT] = {0};  // 存储每个通道的上升沿捕获时间
static uint32_t fallingEdgeTime[CHANNEL_COUNT] = {0}; // 存储每个通道的下降沿捕获时间
static uint8_t isRisingEdge[CHANNEL_COUNT] = {1};     // 每个通道的标志位：1表示检测上升沿，0表示检测下降沿
static uint32_t pwmWidth[CHANNEL_COUNT] = {0};        // 存储每个通道的脉宽（单位：计数值）
static float curMapVal[CHANNEL_COUNT] = {0.5,0,0.5,0.5};          //存储当前通道的map值
static float pwmMapVal[CHANNEL_COUNT] = {0};          // 存储每个通道映射到控制值的结果（0.0 到 1.0）

//pwmMapVal规定
/*
pwmMapVal[0]:通道一  右手左右   控制航向
pwmMapVal[1]:通道二  右手上下   控制升降
pwmMapVal[2]:通道三  左手上下   控制俯仰
pwmMapVal[3]:通道四  左手左右   控制横滚
1.升降会控制四个电机，即通道2脉宽增大将会导致四个通道的PWM输出占空比增大
2.横滚会控制分别控制通道13和24，向右拨滑杆，飞机沿x轴顺时针转，24通道占空比增加，13通道占空比减小
3.俯仰分别控制通道12和34，向上拨滑杆，飞机沿y轴顺时针旋转，12占空比增加，34减少
4.偏航分别控制通道14和23，向右拨滑杆，飞机沿z轴顺时针转，14占空比增加，23减少
*/
// 函数声明
static uint32_t CalculatePWMWidth(uint32_t risingEdge, uint32_t fallingEdge, uint32_t period);
static void MapPWMWidthToValue(uint32_t width, uint32_t channelIndex);
static uint32_t GetChannelIndex(TIM_HandleTypeDef *htim);
static void MapPWMToAngle(uint32_t channelIndex, float pwmVal);
/**
 * @brief 计算脉宽
 * @param risingEdge 上升沿捕获的计数值
 * @param fallingEdge 下降沿捕获的计数值
 * @param period 定时器的自动重装载值（ARR）
 * @return 脉宽值（单位：计数值）
 * 
 * 该函数根据上升沿和下降沿时间点计算脉宽（高电平时间）。
 * 如果发生计数器溢出，考虑溢出的补偿周期。
 */
static uint32_t CalculatePWMWidth(uint32_t risingEdge, uint32_t fallingEdge, uint32_t period) {
    if (fallingEdge >= risingEdge) {
        return fallingEdge - risingEdge;  // 没有溢出，直接计算差值
    } else {
        return (period - risingEdge) + fallingEdge; // 溢出时补偿
    }
}

/**
 * @brief 映射脉宽到控制值
 * @param width 脉宽值（单位：计数值）
 * @param channelIndex 通道索引
 * 
 * 根据不同通道的范围（MIN_MOTORVAL、MAX_MOTORVAL）将脉宽值映射到 0.0 到 1.0 的范围。
 * 特定通道的映射范围通过 `channelIndex` 确定。
 */
static void MapPWMWidthToValue(uint32_t width, uint32_t channelIndex) {
    float MIN_MOTORVAL, MAX_MOTORVAL, SUB_MOTORVAL;

    // 根据通道索引选择不同的映射范围
    switch (channelIndex) {
        case CHANNEL3_INDEX:
            MIN_MOTORVAL = MIN_MOTORVAL3;
            MAX_MOTORVAL = MAX_MOTORVAL3;
            SUB_MOTORVAL = SUB_MOTORVAL3;
            break;
        case CHANNEL2_INDEX:
            MIN_MOTORVAL = MIN_MOTORVAL2;
            MAX_MOTORVAL = MAX_MOTORVAL2;
            SUB_MOTORVAL = SUB_MOTORVAL2;
            break;
        default:  // 偏航控制：CHANNEL14_INDEX，俯仰控制：CHANNEL12_INDEX，横滚控制：CHANNEL24_INDEX
            MIN_MOTORVAL = MIN_MOTORVAL14;
            MAX_MOTORVAL = MAX_MOTORVAL14;
            SUB_MOTORVAL = SUB_MOTORVAL14;
            break;
    }

    // 限制脉宽在有效范围内
    if (width < MIN_MOTORVAL) {
        width = MIN_MOTORVAL;
    }
    if (width > MAX_MOTORVAL) {
        width = MAX_MOTORVAL;
    }

    // 映射值计算
    float mappedValue = ((float)(width - MIN_MOTORVAL)) / SUB_MOTORVAL;
//    pwmMapVal[channelIndex] = mappedValue;
		
		float deta = 0;
		float tmp ;
		
		if(channelIndex == CHANNEL2_INDEX) {  // 升降控制 
			tmp	= curMapVal[CHANNEL2_INDEX]; //记录上次中断时通道2的值
				curMapVal[CHANNEL2_INDEX] = mappedValue;
				deta = mappedValue - tmp;
				
			//保证了在右手上下不变的情况下，通道2不参与转速调整
				pwmMapVal[CHANNEL1_INDEX] += deta; // 电机1增加
				pwmMapVal[CHANNEL2_INDEX] += deta; // 电机2增加
				pwmMapVal[CHANNEL3_INDEX] += deta; // 电机3增加
				pwmMapVal[CHANNEL4_INDEX] += deta; // 电机4增加
		}else if(channelIndex == CHANNEL3_INDEX || channelIndex == CHANNEL4_INDEX){
//			pwmMapVal[channelIndex] = mappedValue;
			MapPWMToAngle(channelIndex, mappedValue);//0-1映射为-30-30度，表示期望的角度倾斜
		}

//				case CHANNEL4_INDEX:  // 横滚控制
//					  tmp	= curMapVal[channelIndex];
//						curMapVal[channelIndex] = mappedValue;
//						deta = mappedValue - tmp;
//						pwmMapVal[CHANNEL1_INDEX] -= deta; // 电机13减小
//						pwmMapVal[CHANNEL3_INDEX] -= deta; // 电机13减小
//						pwmMapVal[CHANNEL2_INDEX] += deta;      // 电机24增加
//						pwmMapVal[CHANNEL4_INDEX] += deta;      // 电机24增加
//						break;

//				case CHANNEL3_INDEX:  // 俯仰控制
//					  tmp	= curMapVal[channelIndex];
//						curMapVal[channelIndex] = mappedValue;
//						deta = mappedValue - tmp;
//						pwmMapVal[CHANNEL1_INDEX] += deta; // 电机12增加
//						pwmMapVal[CHANNEL2_INDEX] += deta; // 电机12增加
//						pwmMapVal[CHANNEL3_INDEX] -= deta; // 电机34减小
//						pwmMapVal[CHANNEL4_INDEX] -= deta; // 电机34减小
//						break;

//				case CHANNEL1_INDEX:  // 偏航控制
//					  tmp	= curMapVal[channelIndex];
//						curMapVal[channelIndex] = mappedValue;
//						deta = mappedValue - tmp;
//						pwmMapVal[CHANNEL1_INDEX] += deta; // 电机14增加
//						pwmMapVal[CHANNEL4_INDEX] += deta; // 电机14增加
//						pwmMapVal[CHANNEL2_INDEX] -= deta; // 电机23减小
//						pwmMapVal[CHANNEL3_INDEX] -= deta; // 电机23减小
//						break;
   
}




/**
 * @brief 获取当前通道索引
 * @param htim 定时器句柄
 * @return 通道索引（0 ~ CHANNEL_COUNT-1），或 INVALID_CHANNEL 表示无效通道
 * 
 * 根据定时器通道，返回对应的通道索引。该索引用于索引捕获数据的数组。
 */
static uint32_t GetChannelIndex(TIM_HandleTypeDef *htim) {
    switch (htim->Channel) {
        case HAL_TIM_ACTIVE_CHANNEL_1: return CHANNEL1_INDEX; // 通道1
        case HAL_TIM_ACTIVE_CHANNEL_2: return CHANNEL2_INDEX; // 通道2
        case HAL_TIM_ACTIVE_CHANNEL_3: return CHANNEL3_INDEX; // 通道3
        case HAL_TIM_ACTIVE_CHANNEL_4: return CHANNEL4_INDEX; // 通道4
        default: return INVALID_CHANNEL;  // 无效通道
    }
}

/**
 * @brief 定时器输入捕获中断回调函数
 * @param htim 定时器句柄
 * 
 * 该函数在定时器捕获事件发生时触发。
 * 它根据当前通道索引读取捕获值，计算脉宽，并更新映射值。
 * 上升沿和下降沿捕获交替进行。
 */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM4) {  // 检查是否为 TIM4
        uint32_t channelIndex = GetChannelIndex(htim);  // 获取通道索引
        if (channelIndex == INVALID_CHANNEL) return;    // 无效通道直接返回

        // 读取捕获值
        uint32_t capturedValue = HAL_TIM_ReadCapturedValue(htim, channelIndex * 4); // 修正参数传递错误

        if (isRisingEdge[channelIndex]) {  // 上升沿捕获
            risingEdgeTime[channelIndex] = capturedValue;  
            __HAL_TIM_SET_CAPTUREPOLARITY(htim, channelIndex * 4, TIM_INPUTCHANNELPOLARITY_FALLING); // 切换到下降沿
        } else {  // 下降沿捕获
            fallingEdgeTime[channelIndex] = capturedValue;
            pwmWidth[channelIndex] = CalculatePWMWidth(risingEdgeTime[channelIndex], fallingEdgeTime[channelIndex], TIM4->ARR); // 计算脉宽
            MapPWMWidthToValue(pwmWidth[channelIndex], channelIndex); // 映射脉宽到控制值
            __HAL_TIM_SET_CAPTUREPOLARITY(htim, channelIndex * 4, TIM_INPUTCHANNELPOLARITY_RISING); // 切换回上升沿
        }
        isRisingEdge[channelIndex] = !isRisingEdge[channelIndex]; // 切换边沿标志位
    }
}

/**
 * @brief 接收机初始化
 * 
 * 启动定时器捕获中断，用于捕获 4 个通道的信号。
 */
void Receiver_Init(void) {
    HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_1); // 启动通道1中断
    HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_2); // 启动通道2中断
    HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_3); // 启动通道3中断
    HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_4); // 启动通道4中断
}

/**
 * @brief 映射值接口
 * 
 * @return 返回对应通道的映射值
 */
float Receiver_GetMappedValue(uint32_t channelIndex) 
{
		return pwmMapVal[channelIndex];
}

// 存储映射后的角度值
static float angleMapVal[CHANNEL_COUNT] = {0.0, 0.0, 0.0, 0.0}; // 初始角度均为0

void Receiver_SetMappedValue(uint32_t channelIndex, float deta) {
	pwmMapVal[channelIndex] += deta;
}
/**
 * @brief 映射通道控制值到角度
 * 
 * @param channelIndex 通道索引
 * @param pwmVal 映射到0-1范围的控制值
 * 
 * 对于通道3（俯仰控制）和通道4（横滚控制），将它们的0-1映射值转换为-30°到30°的角度值。
 */
static void MapPWMToAngle(uint32_t channelIndex, float pwmVal) {
    // 校验通道
    if (channelIndex == CHANNEL3_INDEX || channelIndex == CHANNEL4_INDEX) {
        // 映射公式： 角度 = (控制值 - 0.5) * 60°  
        // 例如，控制值为0.0时，角度为-30°；控制值为1.0时，角度为30°
        angleMapVal[channelIndex] = (pwmVal - 0.5) * 60.0f;
    }
}

/**
 * @brief 获取映射后的角度值
 * 
 * @param channelIndex 通道索引
 * @return 映射后的角度值（-30°到30°）
 */
float Receiver_GetMappedAngle(uint32_t channelIndex) {
    if (channelIndex == CHANNEL3_INDEX || channelIndex == CHANNEL4_INDEX) {
        return angleMapVal[channelIndex];
    }
    return 0.0f; // 如果不是有效通道，返回0°
}