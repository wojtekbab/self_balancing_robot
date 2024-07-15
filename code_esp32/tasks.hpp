/**
 * @file tasks.hpp
 * @author Wojciech Babicki (wojciech.babicki@op.pl)
 * @brief
 * @version 0.1
 * @date 2024-07-15
 *
 * @copyright Copyright (c) 2024
 *
 */
#ifndef TASKS_HPP
#define TASKS_HPP
/**
 * @brief task initialize and frequently read acc and gyro values from MPU6050
 *
 * @param pvParameters
 */
void mpu_measurement_task(void *pvParameters);
/**
 * @brief task calculate theta_p and theta_p_dot based on acc and gyro measurement with Kalman Filter
 * 
 * @param pvParameters 
 */
void theta_obs_task(void *pvParameters);
/**
 * @brief task calculate x, x_dot, psi, psi_dot based on encoder impulses and values of theta_p and theta_p_dot
 * 
 * @param pvParameters 
 */
void x_psi_obs_task(void *pvParameters);
/**
 * @brief task calculate vector of reference state variables
 * 
 * @param pvParameters 
 */
void reference_task(void *pvParameters);
/**
 * @brief task configure PWM signal, implements LQR regulator and sends control signal to motors
 * 
 * @param pvParameters 
 */
void motors_task(void *pvParameters);
/**
 * @brief task print current values of state variables, control, and reference input
 * 
 * @param pvParameters 
 */
void printing_task(void *pvParameters);

#endif // TASKS_HPP
