#ifndef USER_LIB_H
#define USER_LIB_H
#include "main.h"

typedef struct {
  float input;        //��������
  float out;          //�������
  float min_value;    //�޷���Сֵ
  float max_value;    //�޷����ֵ
  float frame_period; //ʱ����
} ramp_function_source_t;

typedef struct {
  float input;        //��������
  float out;          //�˲����������
  float num[1];       //�˲�����
  float frame_period; //�˲���ʱ���� ��λ s
} first_order_filter_type_t;
//���ٿ���
extern float invSqrt(float num);

//б��������ʼ��
void ramp_init(ramp_function_source_t *ramp_source_type, float frame_period, float max, float min);

//б����������
void ramp_calc(ramp_function_source_t *ramp_source_type, float input);
//һ���˲���ʼ��
extern void first_order_filter_init(first_order_filter_type_t *first_order_filter_type, float frame_period, const float num[1]);
//һ���˲�����
extern void first_order_filter_cali(first_order_filter_type_t *first_order_filter_type, float input);
//��������
extern void abs_limit(float *num, float Limit);
//�жϷ���λ
extern float sign(float value);
//��������
extern float float_deadline(float Value, float minValue, float maxValue);
//int26����
extern int16_t int16_deadline(int16_t Value, int16_t minValue, int16_t maxValue);
//�޷�����
extern float float_constrain(float Value, float minValue, float maxValue);
//�޷�����
extern int16_t int16_constrain(int16_t Value, int16_t minValue, int16_t maxValue);
//ѭ���޷�����
extern float loop_float_constrain(float Input, float minValue, float maxValue);
//�Ƕ� ���޷� 180 ~ -180
extern float theta_format(float Ang);
//�������ֵ
#define ABS(x) ((x) > 0 ? (x) : -(x))
//��������֮��ľ���
extern float distance_2d(float x1, float y1, float x2, float y2);

extern float atan2_fast(float x, float y);

#ifndef DEG2RAD
#define DEG2RAD(x) ((x)*0.01745329251994329576923690768489f)
#endif
#ifndef RAD2DEG
#define RAD2DEG(x) ((x) * 57.295779513082320876798154814105f)
#endif

//���ȸ�ʽ��Ϊ-PI~PI
#define rad_format(Ang) loop_float_constrain((Ang), -PI, PI)

//ͨ����λ�ж�һ��ֵ�ķ��� ������1 ������-1
#define SIGN(x) (((signed char *)&x)[sizeof(x) - 1] >> 7 | 1)
#define SIGNBIT(x) (((signed char *)&x)[sizeof(x) - 1] >> 7)

extern float maxabs4f(float value0, float value1, float value2, float value3);

#define __fabs(x) ((x)>0?(x):-(x))
#define __fabsf(x) ((x)>0?(x):-(x))

#define CLAMP(input, min, max)    \
  {                               \
    if ((input) > (max)) {        \
      (input) = (max);            \
    } else if ((input) < (min)) { \
      (input) = (min);            \
    }                             \
  }
#endif
