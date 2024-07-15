/**
 * @file globals.cpp
 * @author Wojciech Babicki (wojciech.babicki@op.pl)
 * @brief 
 * @version 0.1
 * @date 2024-07-15
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#include "globals.hpp"

// FreeRTOS kernel objects
portMUX_TYPE spinlock = portMUX_INITIALIZER_UNLOCKED;
SemaphoreHandle_t mutex_state_variables = NULL;
SemaphoreHandle_t mutex_reference_variables = NULL;
SemaphoreHandle_t mutex_mpu_measurement = NULL;
SemaphoreHandle_t mutex_control_values = NULL;
QueueHandle_t queue_reference_velocities = NULL;
xSemaphoreHandle semaphore_mpu_initialized = NULL;

// calibration values calculated from stationary system
const float calibration_acc_array[] = {0.943387f, -0.027174f, 0.670654f};
const float calibration_gyro_array[] = {-0.034664f, -0.000245f, -0.01180f};

// global structs 
State_variables_struct state_variables_global = {};
State_variables_struct reference_variables_global = {};
Reference_velocities_struct JoystickData_global = {};
MPU_measurement_struct MPU_measurement_global = {};

// relative speed od motor axle
float omega_rot_R_global = 0.0f;
float omega_rot_L_global = 0.0f;

// voltage for motors
float U_R_global = 0.0f;
float U_L_global = 0.0f;

// values used by interrupts
volatile long encoder_counter_mot1_global = 0;
volatile long encoder_counter_mot2_global = 0;

// wifi credentials
const char *ssid = "SSID";
const char *password = "PASSWORD";

// web server object
AsyncWebServer server(80);
