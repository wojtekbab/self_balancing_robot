/**
 * @file globals.hpp
 * @author Wojciech Babicki (wojciech.babicki@op.pl)
 * @brief 
 * @version 0.1
 * @date 2024-07-15
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#ifndef GLOBALS_HPP
#define GLOBALS_HPP

#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <ESPAsyncWebServer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>
#include <freertos/queue.h>

/**
 * @brief struct contains reference data from html page
 * @struct
 */
typedef struct {
  float reference_x_dot;
  float reference_psi_dot;
} Reference_velocities_struct;

/**
 * @brief struct contains imu data
 * 
 */
typedef struct {
  sensors_event_t acc;
  sensors_event_t gyro;
} MPU_measurement_struct;

/**
 * @brief struct contains state variables 
 * 
 */
typedef struct {
  float x;
  float x_dot;
  float theta_p;
  float theta_p_dot;
  float psi;
  float psi_dot;
} State_variables_struct;

// FreeRTOS kernel objects
extern portMUX_TYPE spinlock;
extern SemaphoreHandle_t mutex_state_variables;
extern SemaphoreHandle_t mutex_reference_variables;
extern SemaphoreHandle_t mutex_mpu_measurement;
extern SemaphoreHandle_t mutex_control_values;
extern QueueHandle_t queue_reference_velocities;
extern xSemaphoreHandle semaphore_mpu_initialized;

// calibration values calculated from stationary system
extern const float calibration_acc_array[];
extern const float calibration_gyro_array[];

// global structs 
extern State_variables_struct state_variables_global;
extern State_variables_struct reference_variables_global;
extern Reference_velocities_struct JoystickData_global;
extern MPU_measurement_struct MPU_measurement_global;

// relative speed od motor axle
extern float omega_rot_R_global;
extern float omega_rot_L_global;

// voltage for motors
extern float U_R_global;
extern float U_L_global;

// values used by interrupts
extern volatile long encoder_counter_mot1_global;
extern volatile long encoder_counter_mot2_global;

// wifi credentials
extern const char *ssid;
extern const char *password;

// web server object
extern AsyncWebServer server;

#endif // GLOBALS_HPP
