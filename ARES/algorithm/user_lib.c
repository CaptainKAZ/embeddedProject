#include "user_lib.h"
#include "arm_math.h"
#include <float.h>


//快速开方
float invSqrt(float num) {
  float halfnum = 0.5f * num;
  float y       = num;
  long i       = *(long *)&y;
  i            = 0x5f3759df - (i >> 1);
  y            = *(float *)&i;
  y            = y * (1.5f - (halfnum * y * y));
  return y;
}

/**
  * @brief    计算二维空间中两点之间的距离
  * 
  * @param    x1        点1的x坐标
  * @param    y1        点1的y坐标
  * @param    x2        点2的x坐标
  * @param    y2        点2的y坐标
  * @return   float      两点之间的距离
  */
float distance_2d(float x1, float y1, float x2, float y2) { return 1 / invSqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2)); }

/**
 * @brief          斜波函数初始化
 * @author         RM
 * @param[in]      斜波函数结构体
 * @param[in]      间隔的时间，单位 s
 * @param[in]      最大值
 * @param[in]      最小值
 * @retval         返回空
 */
void ramp_init(ramp_function_source_t *ramp_source_type, float frame_period, float max, float min) {
  ramp_source_type->frame_period = frame_period;
  ramp_source_type->max_value    = max;
  ramp_source_type->min_value    = min;
  ramp_source_type->input        = 0.0f;
  ramp_source_type->out          = 0.0f;
}

/**
 * @brief          斜波函数计算，根据输入的值进行叠加， 输入单位为 /s
 * 即一秒后增加输入的值
 * @author         RM
 * @param[in]      斜波函数结构体
 * @param[in]      输入值
 * @param[in]      滤波参数
 * @retval         返回空
 */
void ramp_calc(ramp_function_source_t *ramp_source_type, float input) {
  ramp_source_type->input = input;
  ramp_source_type->out += ramp_source_type->input * ramp_source_type->frame_period;
  if (ramp_source_type->out > ramp_source_type->max_value) {
    ramp_source_type->out = ramp_source_type->max_value;
  } else if (ramp_source_type->out < ramp_source_type->min_value) {
    ramp_source_type->out = ramp_source_type->min_value;
  }
}
/**
 * @brief          一阶低通滤波初始化
 * @author         RM
 * @param[in]      一阶低通滤波结构体
 * @param[in]      间隔的时间，单位 s
 * @param[in]      滤波参数
 * @retval         返回空
 */
void first_order_filter_init(first_order_filter_type_t *first_order_filter_type, float frame_period, const float num[1]) {
  first_order_filter_type->frame_period = frame_period;
  first_order_filter_type->num[0]       = num[0];
  first_order_filter_type->input        = 0.0f;
  first_order_filter_type->out          = 0.0f;
}

/**
 * @brief          一阶低通滤波计算
 * @author         RM
 * @param[in]      一阶低通滤波结构体
 * @param[in]      间隔的时间，单位 s
 * @retval         返回空
 */
void first_order_filter_cali(first_order_filter_type_t *first_order_filter_type, float input) {
  first_order_filter_type->input = input;
  first_order_filter_type->out =
      first_order_filter_type->num[0] / (first_order_filter_type->num[0] + first_order_filter_type->frame_period) *
          first_order_filter_type->out +
      first_order_filter_type->frame_period / (first_order_filter_type->num[0] + first_order_filter_type->frame_period) *
          first_order_filter_type->input;
}

//绝对限制
void abs_limit(float *num, float Limit) {
  if (*num > Limit) {
    *num = Limit;
  } else if (*num < -Limit) {
    *num = -Limit;
  }
}

//判断符号位
float sign(float value) {
  if (value >= 0.0f) {
    return 1.0f;
  } else {
    return -1.0f;
  }
}

//浮点死区
float float_deadline(float Value, float minValue, float maxValue) {
  if (Value < maxValue && Value > minValue) {
    Value = 0.0f;
  }
  return Value;
}

// int16死区
int16_t int16_deadline(int16_t Value, int16_t minValue, int16_t maxValue) {
  if (Value < maxValue && Value > minValue) {
    Value = 0;
  }
  return Value;
}

//限幅函数
float float_constrain(float Value, float minValue, float maxValue) {
  if (Value < minValue)
    return minValue;
  else if (Value > maxValue)
    return maxValue;
  else
    return Value;
}

//限幅函数
int16_t int16_constrain(int16_t Value, int16_t minValue, int16_t maxValue) {
  if (Value < minValue)
    return minValue;
  else if (Value > maxValue)
    return maxValue;
  else
    return Value;
}

//循环限幅函数
float loop_float_constrain(float Input, float minValue, float maxValue) {
  if (maxValue < minValue) {
    return Input;
  }

  if (Input > maxValue) {
    float len = maxValue - minValue;
    while (Input > maxValue) {
      Input -= len;
    }
  } else if (Input < minValue) {
    float len = maxValue - minValue;
    while (Input < minValue) {
      Input += len;
    }
  }
  return Input;
}


float atan2_fast(float x, float y) {
  float ax = __fabs(x), ay = __fabs(y);
  float a = fminf(ax, ay) / (fmaxf(ax, ay) + FLT_EPSILON);
  float s = a * a;
  float r = ((-0.0464964749f * s + 0.15931422f) * s - 0.327622764f) * s * a + a;
  if (ay > ax)
    r = 1.57079637f - r;
  if (x < 0)
    r = 3.14159274f - r;
  if (y < 0)
    r = -r;
  return r;
}

//弧度格式化为-PI~PI

//角度格式化为-180~180
float theta_format(float Ang) { return loop_float_constrain(Ang, -180.0f, 180.0f); }

float maxabs4f(float value0, float value1, float value2, float value3) {
  float ret = 0;

  if (_fabsf(value0) > ret) {
    ret = __fabsf(value0);
  }
  if (_fabsf(value1) > ret) {
    ret = __fabsf(value1);
  }
  if (_fabsf(value2) > ret) {
    ret = __fabsf(value2);
  }
  if (_fabsf(value3) > ret) {
    ret = __fabsf(value3);
  }
  return ret;
}
