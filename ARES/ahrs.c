#include "cmsis_os.h"
#include "data_builder.h"
#include "eMPL_outputs.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "invensense.h"
#include "invensense_adv.h"
#include "main.h"
#include "mltypes.h"
#include "mpl.h"
#include "mpu.h"
#include "quaternion_supervisor.h"
// q30，q16格式,long转float时的除数.
#define q30 1073741824.0f
#define q16 65536.0f

//陀螺仪方向设置
static signed char gyro_orientation[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
//磁力计方向设置
static signed char comp_orientation[9] = {0, 1, 0, 1, 0, 0, 0, 0, -1};
// MPU9250自测试
//返回值:0,正常
//    其他,失败
uint8_t run_self_test(void) {
  int result;
  // char test_packet[4] = {0};
  long gyro[3], accel[3];
  result = mpu_run_6500_self_test(gyro, accel, 0);
  if (result == 0x7) {
    /* Test passed. We can trust the gyro data here, so let's push it down
     * to the DMP.
     */
    unsigned short accel_sens;
    float gyro_sens;

    mpu_get_gyro_sens(&gyro_sens);
    gyro[0] = (long)(gyro[0] * gyro_sens);
    gyro[1] = (long)(gyro[1] * gyro_sens);
    gyro[2] = (long)(gyro[2] * gyro_sens);
    // inv_set_gyro_bias(gyro, 3);
    dmp_set_gyro_bias(gyro);
    mpu_get_accel_sens(&accel_sens);
    accel[0] *= accel_sens;
    accel[1] *= accel_sens;
    accel[2] *= accel_sens;
    // inv_set_accel_bias(accel, 3);
    dmp_set_accel_bias(accel);
    return 0;
  } else
    return 1;
}

//方向转换
unsigned short inv_row_2_scale(const signed char *row) {
  unsigned short b;

  if (row[0] > 0)
    b = 0;
  else if (row[0] < 0)
    b = 4;
  else if (row[1] > 0)
    b = 1;
  else if (row[1] < 0)
    b = 5;
  else if (row[2] > 0)
    b = 2;
  else if (row[2] < 0)
    b = 6;
  else
    b = 7;  // error
  return b;
}

#define DEFAULT_MPU_HZ 200
#define COMPASS_READ_MS 50

// mpu6050,dmp初始化
//返回值:0,正常
//    其他,失败
uint8_t mpu_dmp_init(void) {
  uint8_t res = 0;
  struct int_param_s int_param;
  unsigned char accel_fsr;
  unsigned short gyro_rate, gyro_fsr;
  unsigned short compass_fsr;

  if (mpu_init(&int_param) == 0)  //初始化MPU9250
  {
    res = inv_init_mpl();  //初始化MPL
    if (res) return 1;
    inv_enable_quaternion();
    inv_enable_9x_sensor_fusion();
    inv_enable_fast_nomot();
    inv_enable_gyro_tc();
    inv_enable_vector_compass_cal();
    inv_enable_magnetic_disturbance();
    inv_enable_eMPL_outputs();
    res = inv_start_mpl();  //开启MPL
    if (res) return 1;
    res = mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL |
                          INV_XYZ_COMPASS);  //设置所需要的传感器
    if (res) return 2;
    res = mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL);  //设置FIFO
    if (res) return 3;
    res = mpu_set_sample_rate(1000);  //设置采样率
    if (res) return 4;
    res =
        mpu_set_compass_sample_rate(1000 / COMPASS_READ_MS);  //设置磁力计采样率
    if (res) return 5;
    mpu_get_sample_rate(&gyro_rate);
    mpu_get_gyro_fsr(&gyro_fsr);
    mpu_get_accel_fsr(&accel_fsr);
    mpu_get_compass_fsr(&compass_fsr);
    inv_set_gyro_sample_rate(1000);
    inv_set_accel_sample_rate(1000);
    inv_set_compass_sample_rate(COMPASS_READ_MS * 1000L);
    inv_set_gyro_orientation_and_scale(
        inv_orientation_matrix_to_scalar(gyro_orientation),
        (long)gyro_fsr << 15);
    inv_set_accel_orientation_and_scale(
        inv_orientation_matrix_to_scalar(gyro_orientation),
        (long)accel_fsr << 15);
    inv_set_compass_orientation_and_scale(
        inv_orientation_matrix_to_scalar(comp_orientation),
        (long)compass_fsr << 15);

    res = dmp_load_motion_driver_firmware();  //加载dmp固件
    if (res) return 6;
    res = dmp_set_orientation(
        inv_orientation_matrix_to_scalar(gyro_orientation));  //设置陀螺仪方向
    if (res) return 7;
    res = dmp_enable_feature(
        DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_TAP |  //设置dmp功能
        DMP_FEATURE_ANDROID_ORIENT | DMP_FEATURE_SEND_RAW_ACCEL |
        DMP_FEATURE_SEND_CAL_GYRO | DMP_FEATURE_GYRO_CAL);
    if (res) return 8;
    res = dmp_set_fifo_rate(DEFAULT_MPU_HZ);  //设置DMP输出速率(最大不超过200Hz)
    if (res) return 9;
    res = run_self_test();  //自检
    if (res) return 10;
    res = mpu_set_dmp_state(1);  //使能DMP
    if (res) return 11;
    return 0;
  } else {
    return -1;
  }
}
//得到dmp处理后的数据(注意,本函数需要比较多堆栈,局部变量有点多)
// pitch:俯仰角 精度:0.1°   范围:-90.0° <---> +90.0°
// roll:横滚角  精度:0.1°   范围:-180.0°<---> +180.0°
// yaw:航向角   精度:0.1°   范围:-180.0°<---> +180.0°
//返回值:0,正常
//    其他,失败
uint8_t mpu_dmp_get_data(float *pitch, float *roll, float *yaw) {
  float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
  unsigned long sensor_timestamp;
  short gyro[3], accel[3], sensors;
  unsigned char more;
  long quat[4];
  if (dmp_read_fifo(gyro, accel, quat, &sensor_timestamp, &sensors, &more))
    return 1;
  /* Gyro and accel data are written to the FIFO by the DMP in chip frame and
   *hardware units. This behavior is convenient because it keeps the gyro and
   *accel outputs of dmp_read_fifo and mpu_read_fifo consistent.
   **/
  /*if (sensors & INV_XYZ_GYRO )
  send_packet(PACKET_TYPE_GYRO, gyro);
  if (sensors & INV_XYZ_ACCEL)
  send_packet(PACKET_TYPE_ACCEL, accel); */
  /* Unlike gyro and accel, quaternions are written to the FIFO in the body
   *frame, q30. The orientation is set by the scalar passed to
   *dmp_set_orientation during initialization.
   **/
  if (sensors & INV_WXYZ_QUAT) {
    q0 = quat[0] / q30;  // q30格式转换为浮点数
    q1 = quat[1] / q30;
    q2 = quat[2] / q30;
    q3 = quat[3] / q30;
    //计算得到俯仰角/横滚角/航向角
    *pitch = asin(-2 * q1 * q3 + 2 * q0 * q2) * 57.3;  // pitch
    *roll = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2 * q2 + 1) *
            57.3;  // roll
    *yaw =
        atan2(2 * (q1 * q2 + q0 * q3), q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3) *
        57.3;  // yaw
  } else
    return 2;
  return 0;
}

//得到mpl处理后的数据(注意,本函数需要比较多堆栈,局部变量有点多)
// pitch:俯仰角 精度:0.1°   范围:-90.0° <---> +90.0°
// roll:横滚角  精度:0.1°   范围:-180.0°<---> +180.0°
// yaw:航向角   精度:0.1°   范围:-180.0°<---> +180.0°
//返回值:0,正常
//    其他,失败
uint8_t mpu_mpl_get_data(float *pitch, float *roll, float *yaw) {
  unsigned long sensor_timestamp, timestamp;
  short gyro[3], accel_short[3], compass_short[3], sensors;
  unsigned char more;
  long compass[3], accel[3], quat[4], temperature;
  long data[9];
  int8_t accuracy;

  if (dmp_read_fifo(gyro, accel_short, quat, &sensor_timestamp, &sensors,
                    &more))
    return 1;

  if (sensors & INV_XYZ_GYRO) {
    inv_build_gyro(gyro, sensor_timestamp);  //把新数据发送给MPL
    mpu_get_temperature(&temperature, &sensor_timestamp);
    inv_build_temp(temperature,
                   sensor_timestamp);  //把温度值发给MPL，只有陀螺仪需要温度值
  }

  if (sensors & INV_XYZ_ACCEL) {
    accel[0] = (long)accel_short[0];
    accel[1] = (long)accel_short[1];
    accel[2] = (long)accel_short[2];
    inv_build_accel(accel, 0, sensor_timestamp);  //把加速度值发给MPL
  }

  if (!mpu_get_compass_reg(compass_short, &sensor_timestamp)) {
    compass[0] = (long)compass_short[0];
    compass[1] = (long)compass_short[1];
    compass[2] = (long)compass_short[2];
    inv_build_compass(compass, 0, sensor_timestamp);  //把磁力计值发给MPL
  }
  inv_execute_on_data();
  inv_get_sensor_type_euler(data, &accuracy, &timestamp);

  *roll = (data[0] / q16);
  *pitch = -(data[1] / q16);
  *yaw = -data[2] / q16;
  return 0;
}
#include "feedback_task.h"
#include "i2c.h"
float pitch, roll, yaw;
float yaw_mag;
short compass_short[3];
float compass_float[3];
float xmin=999, xmax=-999, ymin=999, ymax=-999;
float x = 0, y = 0;
void ahrs_task(void) {
  #ifdef MY_IIC
  IIC_Init();
  #endif
  osDelay(1024);
  while (mpu_dmp_init()) {
    osDelay(1);
  }
  feedback_register(&compass_float[0], 0);
  feedback_register(&compass_float[1], 1);
  feedback_register(&compass_float[2], 2);
  unsigned long sensor_timestamp;
  while (1) {
    mpu_dmp_get_data(&pitch, &roll, &yaw);
    mpu_get_compass_reg(compass_short, &sensor_timestamp);
    for (int i = 0; i < 3; i++) {
      compass_float[i] = (float)compass_short[i];
    }
    if (compass_short[0] > xmax) xmax +=1;
    if (compass_short[0] < xmin) xmin -=1;
    if (compass_short[1] > ymax) ymax +=1;
    if (compass_short[1] < ymin) ymin -=1;
    
    if (xmax > xmin) x = (float)(compass_short[0] - (xmax+xmin)*0.5) / (xmax - xmin);
    if (ymax > ymin) y = (float)(compass_short[1] - (ymax+ymin)*0.5) / (ymax - ymin);
    yaw_mag =
        180.0 / 3.141592653589 * atan2(y, x);
    osDelay(5);
  }
}
