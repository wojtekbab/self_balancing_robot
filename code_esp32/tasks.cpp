/**
 * @file tasks.cpp
 * @author Wojciech Babicki (wojciech.babicki@op.pl)
 * @brief
 * @version 0.1
 * @date 2024-07-15
 *
 * @copyright Copyright (c) 2024
 *
 */
#include <Adafruit_MPU6050.h>
#include <freertos/semphr.h>

#include "globals.hpp"
#include "tasks.hpp"
#include "defines.hpp"
#include "utilities.hpp"
#include "freertos/event_groups.h"

void mpu_measurement_task(void *pvParameters)
{
  Adafruit_MPU6050 mpu;
  sensors_event_t acc;
  sensors_event_t gyro;
  sensors_event_t temp;

  // initialize mpu
  if (!mpu.begin())
  {
    Serial.println("Failed to find MPU6050 chip");
    while (1)
    {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  Serial.print("Accelerometer range set to: ");
  switch (mpu.getAccelerometerRange())
  {
  case MPU6050_RANGE_2_G:
    Serial.println("+-2G");
    break;
  case MPU6050_RANGE_4_G:
    Serial.println("+-4G");
    break;
  case MPU6050_RANGE_8_G:
    Serial.println("+-8G");
    break;
  case MPU6050_RANGE_16_G:
    Serial.println("+-16G");
    break;
  }
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  Serial.print("Gyro range set to: ");
  switch (mpu.getGyroRange())
  {
  case MPU6050_RANGE_250_DEG:
    Serial.println("+- 250 deg/s");
    break;
  case MPU6050_RANGE_500_DEG:
    Serial.println("+- 500 deg/s");
    break;
  case MPU6050_RANGE_1000_DEG:
    Serial.println("+- 1000 deg/s");
    break;
  case MPU6050_RANGE_2000_DEG:
    Serial.println("+- 2000 deg/s");
    break;
  }

  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
  Serial.print("Filter bandwidth set to: ");
  switch (mpu.getFilterBandwidth())
  {
  case MPU6050_BAND_260_HZ:
    Serial.println("260 Hz");
    break;
  case MPU6050_BAND_184_HZ:
    Serial.println("184 Hz");
    break;
  case MPU6050_BAND_94_HZ:
    Serial.println("94 Hz");
    break;
  case MPU6050_BAND_44_HZ:
    Serial.println("44 Hz");
    break;
  case MPU6050_BAND_21_HZ:
    Serial.println("21 Hz");
    break;
  case MPU6050_BAND_10_HZ:
    Serial.println("10 Hz");
    break;
  case MPU6050_BAND_5_HZ:
    Serial.println("5 Hz");
    break;
  }
  // inform setup()
  xSemaphoreGive(semaphore_mpu_initialized);

  while (1)
  {
    // measure values
    mpu.getEvent(&acc, &gyro, &temp);

    // calibration
    for (int i = 0; i < 3; i++)
    {
      acc.acceleration.v[i] -= calibration_acc_array[i];
      gyro.gyro.v[i] -= calibration_gyro_array[i];
    }

    // write values
    xSemaphoreTake(mutex_mpu_measurement, portMAX_DELAY);
    MPU_measurement_global.acc = acc;
    MPU_measurement_global.gyro = gyro;
    xSemaphoreGive(mutex_mpu_measurement);

    vTaskDelay(1 / portTICK_PERIOD_MS);
  }
}

void theta_obs_task(void *pvParameters)
{
  MPU_measurement_struct MPU_measurement_struct;

  // variances
  float const var_gyro = 0.01;
  float const var_measure = 1.0f;

  // consts
  float const F = 1.0f;
  float const H = 1.0f;

  // inits
  float x_post = 0.0f;
  float x_apriori = 0.0f;
  float P_apriori = 0.0f;
  float P_post = 0.0f;
  float S = 0.0f;
  float K = 0.0f;
  float u = 0.0f;
  float z = 0.0f;
  unsigned long last_time = 0;
  float dt = 0.0f;
  while (1)
  {
    // read values
    xSemaphoreTake(mutex_mpu_measurement, portMAX_DELAY);
    MPU_measurement_struct = MPU_measurement_global;
    xSemaphoreGive(mutex_mpu_measurement);

    // calculations
    dt = (float)(micros() - last_time) / 1000000.0f;
    last_time = micros();

    // axis x of gyro sensor is y_b axis from mathematical model
    u = -MPU_measurement_struct.gyro.gyro.x;

    // axis y of accel sensor is x_b axis of model, z sensor's is z_b model's
    z = atan2(-MPU_measurement_struct.acc.acceleration.y, MPU_measurement_struct.acc.acceleration.z);

    x_apriori = F * x_post + dt * u;
    P_apriori = F * P_post * F + (dt * var_gyro);

    S = H * P_apriori * H + (var_measure);
    K = P_apriori * H * (1.0f / S);
    x_post = x_apriori + K * (z - H * x_apriori);
    P_post = (1.0f - K * H) * P_apriori;

    // write values
    xSemaphoreTake(mutex_state_variables, portMAX_DELAY);
    state_variables_global.theta_p = x_post;
    state_variables_global.theta_p_dot = u;
    xSemaphoreGive(mutex_state_variables);

    vTaskDelay(5 / portTICK_PERIOD_MS);
  }
}

void x_psi_obs_task(void *pvParameters)
{
  // local consts
  u_int16_t const IMPULSES_PER_ROT = 3200; // [IMPULSES]
  float const WHEEL_RADIUS = 0.045;        // [m]
  float const WHEELS_DISTANCE = 0.245f;    // [m]

  // local vars
  long encoder_counter_mot1 = 0.0f;
  long encoder_counter_mot2 = 0.0f;

  float theta_rot_R = 0.0f; // rotor (relative) angle (right motor)
  float theta_rot_L = 0.0f;
  float theta_rot_R_last = 0.0f;
  float theta_rot_L_last = 0.0f;
  float omega_rot_R = 0.0f; // rotor (relative) angular speed (right motor)
  float omega_rot_L = 0.0f;

  State_variables_struct state_variables;

  unsigned long last_time = 0;
  float dt = 0.0f;

  while (1)
  {

    dt = (float)(micros() - last_time) / 1000000.0f;
    last_time = micros();

    // read values
    portENTER_CRITICAL(&spinlock); // disable interrupts for a while
    encoder_counter_mot1 = encoder_counter_mot1_global;
    encoder_counter_mot2 = encoder_counter_mot2_global;
    portEXIT_CRITICAL(&spinlock);
    xSemaphoreTake(mutex_state_variables, portMAX_DELAY);
    state_variables = state_variables_global;
    xSemaphoreGive(mutex_state_variables);

    // calculations
    theta_rot_R = ((float)encoder_counter_mot1 / IMPULSES_PER_ROT) * 2.0f * PI;
    theta_rot_L = ((float)encoder_counter_mot2 / IMPULSES_PER_ROT) * 2.0f * PI;
    omega_rot_R = (theta_rot_R - theta_rot_R_last) / dt;
    omega_rot_L = (theta_rot_L - theta_rot_L_last) / dt;
    theta_rot_R_last = theta_rot_R;
    theta_rot_L_last = theta_rot_L;

    state_variables.x = WHEEL_RADIUS * ((theta_rot_R + theta_rot_L) / 2.0f + state_variables.theta_p);
    state_variables.x_dot = WHEEL_RADIUS * ((omega_rot_R + omega_rot_L) / 2.0f + state_variables.theta_p_dot);

    state_variables.psi = (theta_rot_R - theta_rot_L) * WHEEL_RADIUS / WHEELS_DISTANCE;
    state_variables.psi_dot = (omega_rot_R - omega_rot_L) * WHEEL_RADIUS / WHEELS_DISTANCE;

    // write values
    xSemaphoreTake(mutex_state_variables, portMAX_DELAY);
    state_variables_global = state_variables;
    omega_rot_R_global = omega_rot_R;
    omega_rot_L_global = omega_rot_L;
    xSemaphoreGive(mutex_state_variables);

    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

void reference_task(void *pvParameters)
{
  // create local variables
  float reference_x = 0.0f;
  float reference_x_dot = 0.0f;
  float reference_psi = 0.0f;
  float reference_psi_dot = 0.0f;
  State_variables_struct reference_variabes;
  Reference_velocities_struct reference_velocities_struct;

  unsigned long last_time = 0;
  float dt = 0.0f;

  while (1)
  {
    // check start/stop button event
    EventBits_t uxBits = xEventGroupGetBits(xEventGroup);
    if (false == (uxBits & START_STOP_BIT))
    {
      vTaskDelay(50 / portTICK_PERIOD_MS);
      last_time = micros();
      continue;
    }

    // read values
    xQueueReceive(queue_reference_velocities, &reference_velocities_struct, 0);

    // calculations - integrators
    dt = (float)(micros() - last_time) / 1000000.0f;
    last_time = micros();

    reference_x_dot = (reference_velocities_struct.reference_x_dot / 100.0f) * REFERENCE_X_DOT_MAX;
    reference_psi_dot = (reference_velocities_struct.reference_psi_dot / 100.0f) * REFERENCE_PSI_DOT_MAX;

    reference_x_dot = 0.1885; //(reference_velocities_struct.reference_x_dot / 100.0f) * REFERENCE_X_DOT_MAX;
    reference_psi_dot = 1.2566;

    reference_x += reference_x_dot * dt;
    reference_psi += reference_psi_dot * dt;

    // write values
    reference_variabes.x = reference_x;
    reference_variabes.x_dot = reference_x_dot;
    reference_variabes.theta_p = 0.0f;
    reference_variabes.theta_p_dot = 0.0f;
    reference_variabes.psi = reference_psi;
    reference_variabes.psi_dot = reference_psi_dot;

    xSemaphoreTake(mutex_reference_variables, portMAX_DELAY);
    reference_variables_global = reference_variabes;
    xSemaphoreGive(mutex_reference_variables);

    vTaskDelay(50 / portTICK_PERIOD_MS);
  }
}

void motors_task(void *pvParameters)
{
  // setting PWM properties
  const int freq = 5000;
  const int chanel_mot1_A = 0;
  const int chanel_mot1_B = 1;
  const int chanel_mot2_A = 2;
  const int chanel_mot2_B = 3;
  const int resolution = 10;

  State_variables_struct reference_variabes;

  State_variables_struct state_variables;

  // values for matrix K taken from Matlab script with lqr(A,B,Q,R)
  const float K_LQR[2][6] = {

      -1.0000, -1.6047, -5.9747, -1.3647, 0.0000, 0.0000, -0.0000, -0.0000, -0.0000, -0.0000, 1.0000, 1.0034};

  // motor params and friction
  float b_Coul_L = 0.1f;        // friction static [Nm] (FOR ESTIMATE)
  float b_Coul_R = 0.1f;        // friction static [Nm] (FOR ESTIMATE)
  const float b_visc_R = 0.01f; // friction viscotic  [Nms/rad] (FOR ESTIMATE)
  const float b_visc_L = 0.01f; // friction viscotic [Nms/rad] (FOR ESTIMATE)
  const float R = 2.22;         // resistance  [Ω]
  const float k_e = 0.56;       // electric motor const [Vs/rad]
  const float k_m = 0.39;       // torque motor const [Nm/A]

  float omega_rot_R = 0.0f; // rotor (relative) angular speed
  float omega_rot_L = 0.0f;

  float U_R = 0.0f;
  float U_L = 0.0f;

  // configure LED PWM functionalities
  ledcSetup(chanel_mot1_A, freq, resolution);
  ledcSetup(chanel_mot1_B, freq, resolution);
  ledcSetup(chanel_mot2_A, freq, resolution);
  ledcSetup(chanel_mot2_B, freq, resolution);

  // attach the channel to the GPIO to be controlled
  ledcAttachPin(GPIO_MOT1_PWM_R, chanel_mot1_A);
  ledcAttachPin(GPIO_MOT1_PWM_L, chanel_mot1_B);
  ledcAttachPin(GPIO_MOT2_PWM_R, chanel_mot2_A);
  ledcAttachPin(GPIO_MOT2_PWM_L, chanel_mot2_B);

  while (1)
  {
    // check start/stop button event
    EventBits_t uxBits = xEventGroupGetBits(xEventGroup);
    if (false == (uxBits & START_STOP_BIT))
    {
      vTaskDelay(50 / portTICK_PERIOD_MS);
      ledcWrite(chanel_mot1_A, 0);
      ledcWrite(chanel_mot1_B, 0);
      ledcWrite(chanel_mot2_A, 0);
      ledcWrite(chanel_mot2_B, 0);
      continue;
    }

    // read values
    xSemaphoreTake(mutex_state_variables, portMAX_DELAY);
    state_variables = state_variables_global;
    omega_rot_R = omega_rot_R_global;
    omega_rot_L = omega_rot_L_global;
    xSemaphoreGive(mutex_state_variables);

    xSemaphoreTake(mutex_reference_variables, portMAX_DELAY);
    reference_variabes = reference_variables_global;
    xSemaphoreGive(mutex_reference_variables);

    float array_of_error[6];

    float T_theta_psi_array[2] = {0.0f, 0.0f};

    calculate_difference_of_structs(&reference_variabes, &state_variables, array_of_error);

    multiplyVectorByMatrix(array_of_error, K_LQR, T_theta_psi_array);

    // convert T_theta, T_psi into T_R and T_L
    float T_R = (T_theta_psi_array[0] + T_theta_psi_array[1]) / 2;
    float T_L = (T_theta_psi_array[0] - T_theta_psi_array[1]) / 2;

    if (abs(reference_variabes.x_dot) > 0.05)
    {
      b_Coul_R = 0.0f;
      b_Coul_L = 0.0f;
    }

    // convert T_R, T_L into U_R and U_L
    U_R = R / k_m * (T_R - (-k_m * k_e / R - b_visc_R) * omega_rot_R + b_Coul_R * signum(omega_rot_R));
    U_L = R / k_m * (T_L - (-k_m * k_e / R - b_visc_L) * omega_rot_L + b_Coul_L * signum(omega_rot_L));

    // saturate signal with limit  +/-12V
    U_R = saturation(U_R, 12.0f);
    U_L = saturation(U_L, 12.0f);

    // write values
    xSemaphoreTake(mutex_control_values, portMAX_DELAY);
    U_R_global = U_R;
    U_L_global = U_L;
    xSemaphoreGive(mutex_control_values);

    // Write PWM to driver MDD3A, its require 2 PWMs for each motor
    if (U_R >= 0)
    {
      ledcWrite(chanel_mot1_A, (int)U_R * pow(2, resolution) / 12.0f);
      ledcWrite(chanel_mot1_B, (int)0);
    }
    else
    {
      ledcWrite(chanel_mot1_A, (int)0);
      ledcWrite(chanel_mot1_B, (int)-U_R * pow(2, resolution) / 12.0f);
    }

    if (U_L >= 0)
    {
      ledcWrite(chanel_mot2_A, (int)U_L * pow(2, resolution) / 12.0f);
      ledcWrite(chanel_mot2_B, (int)0);
    }
    else
    {
      ledcWrite(chanel_mot2_A, (int)0);
      ledcWrite(chanel_mot2_B, (int)-U_L * pow(2, resolution) / 12.0f);
    }

    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

void printing_task(void *pvParameters)
{
  // create local variables
  State_variables_struct state_variables;

  float U_R = 0.0f;
  float U_L = 0.0f;

  State_variables_struct reference_variabes;

  while (1)
  {
    // read values
    xSemaphoreTake(mutex_state_variables, portMAX_DELAY);
    state_variables = state_variables_global;
    xSemaphoreGive(mutex_state_variables);
    xSemaphoreTake(mutex_control_values, portMAX_DELAY);
    U_R = U_R_global;
    U_L = U_L_global;
    xSemaphoreGive(mutex_control_values);

    xSemaphoreTake(mutex_reference_variables, portMAX_DELAY);
    reference_variabes = reference_variables_global;
    xSemaphoreGive(mutex_reference_variables);

    // print values
    char buffer[256];

    sprintf(buffer, "%lu %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f ",
            millis(),
            state_variables.x,
            state_variables.x_dot,
            state_variables.theta_p,
            state_variables.theta_p_dot,
            state_variables.psi,
            state_variables.psi_dot,
            U_R,
            U_L,
            reference_variabes.x,
            reference_variabes.x_dot,
            reference_variabes.psi,
            reference_variabes.psi_dot);

    Serial.println(buffer);
    vTaskDelay(20 / portTICK_PERIOD_MS);
  }
}
