#include "user_lib.h"
#include "arm_math.h"
#include <float.h>


//���ٿ���
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
  * @brief    �����ά�ռ�������֮��ľ���
  * 
  * @param    x1        ��1��x����
  * @param    y1        ��1��y����
  * @param    x2        ��2��x����
  * @param    y2        ��2��y����
  * @return   float      ����֮��ľ���
  */
float distance_2d(float x1, float y1, float x2, float y2) { return 1 / invSqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2)); }

/**
 * @brief          б��������ʼ��
 * @author         RM
 * @param[in]      б�������ṹ��
 * @param[in]      �����ʱ�䣬��λ s
 * @param[in]      ���ֵ
 * @param[in]      ��Сֵ
 * @retval         ���ؿ�
 */
void ramp_init(ramp_function_source_t *ramp_source_type, float frame_period, float max, float min) {
  ramp_source_type->frame_period = frame_period;
  ramp_source_type->max_value    = max;
  ramp_source_type->min_value    = min;
  ramp_source_type->input        = 0.0f;
  ramp_source_type->out          = 0.0f;
}

/**
 * @brief          б���������㣬���������ֵ���е��ӣ� ���뵥λΪ /s
 * ��һ������������ֵ
 * @author         RM
 * @param[in]      б�������ṹ��
 * @param[in]      ����ֵ
 * @param[in]      �˲�����
 * @retval         ���ؿ�
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
 * @brief          һ�׵�ͨ�˲���ʼ��
 * @author         RM
 * @param[in]      һ�׵�ͨ�˲��ṹ��
 * @param[in]      �����ʱ�䣬��λ s
 * @param[in]      �˲�����
 * @retval         ���ؿ�
 */
void first_order_filter_init(first_order_filter_type_t *first_order_filter_type, float frame_period, const float num[1]) {
  first_order_filter_type->frame_period = frame_period;
  first_order_filter_type->num[0]       = num[0];
  first_order_filter_type->input        = 0.0f;
  first_order_filter_type->out          = 0.0f;
}

/**
 * @brief          һ�׵�ͨ�˲�����
 * @author         RM
 * @param[in]      һ�׵�ͨ�˲��ṹ��
 * @param[in]      �����ʱ�䣬��λ s
 * @retval         ���ؿ�
 */
void first_order_filter_cali(first_order_filter_type_t *first_order_filter_type, float input) {
  first_order_filter_type->input = input;
  first_order_filter_type->out =
      first_order_filter_type->num[0] / (first_order_filter_type->num[0] + first_order_filter_type->frame_period) *
          first_order_filter_type->out +
      first_order_filter_type->frame_period / (first_order_filter_type->num[0] + first_order_filter_type->frame_period) *
          first_order_filter_type->input;
}

//��������
void abs_limit(float *num, float Limit) {
  if (*num > Limit) {
    *num = Limit;
  } else if (*num < -Limit) {
    *num = -Limit;
  }
}

//�жϷ���λ
float sign(float value) {
  if (value >= 0.0f) {
    return 1.0f;
  } else {
    return -1.0f;
  }
}

//��������
float float_deadline(float Value, float minValue, float maxValue) {
  if (Value < maxValue && Value > minValue) {
    Value = 0.0f;
  }
  return Value;
}

// int16����
int16_t int16_deadline(int16_t Value, int16_t minValue, int16_t maxValue) {
  if (Value < maxValue && Value > minValue) {
    Value = 0;
  }
  return Value;
}

//�޷�����
float float_constrain(float Value, float minValue, float maxValue) {
  if (Value < minValue)
    return minValue;
  else if (Value > maxValue)
    return maxValue;
  else
    return Value;
}

//�޷�����
int16_t int16_constrain(int16_t Value, int16_t minValue, int16_t maxValue) {
  if (Value < minValue)
    return minValue;
  else if (Value > maxValue)
    return maxValue;
  else
    return Value;
}

//ѭ���޷�����
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

//���ȸ�ʽ��Ϊ-PI~PI

//�Ƕȸ�ʽ��Ϊ-180~180
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
