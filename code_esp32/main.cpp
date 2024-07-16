/**
 * @file main.cpp
 * @author Wojciech Babicki (wojciech.babicki@op.pl)
 * @brief
 * @version 0.1
 * @date 2024-07-15
 *
 * @copyright Copyright (c) 2024
 *
 */
#include "defines.hpp"
#include "globals.hpp"
#include "tasks.hpp"
#include "web_server.hpp"
#include "interrupts.hpp"
#include "utilities.hpp"

/**
 * @brief function initialize serial, set interrupts, initialize FreeRTOS objects, connect to wifi, initialize tasks
 *
 */
void setup(void)
{
  // serial
  Serial.begin(115200);
  while (!Serial)
  {
    delay(10);
  }
  Serial.println("Serial initialized!");

  // interrupts
  pinMode(GPIO_MOT1_ENCA, INPUT_PULLUP);
  pinMode(GPIO_MOT1_ENCB, INPUT_PULLUP);
  pinMode(GPIO_MOT2_ENCA, INPUT_PULLUP);
  pinMode(GPIO_MOT2_ENCB, INPUT_PULLUP);

  attachInterrupt(GPIO_MOT1_ENCA, ISR_callback_mot1, CHANGE);
  attachInterrupt(GPIO_MOT1_ENCB, ISR_callback_mot1, CHANGE);
  attachInterrupt(GPIO_MOT2_ENCA, ISR_callback_mot2, CHANGE);
  attachInterrupt(GPIO_MOT2_ENCB, ISR_callback_mot2, CHANGE);

  // FreeRTOS objects
  mutex_state_variables = xSemaphoreCreateMutex();
  mutex_reference_variables = xSemaphoreCreateMutex();
  mutex_mpu_measurement = xSemaphoreCreateMutex();
  mutex_control_values = xSemaphoreCreateMutex();
  queue_reference_velocities = xQueueCreate(QUEUE_JOYSTICK_LENGTH, sizeof(Reference_velocities_struct));
  semaphore_mpu_initialized = xSemaphoreCreateBinary();
  xEventGroup = xEventGroupCreate();

  // wifi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(1000);
    Serial.println("Connecting to WiFi..");
  }
  Serial.println("Connected to WiFi");
  Serial.println(WiFi.localIP());

  // server
  setupWebServer();

  // tasks
  Serial.println("Initializing tasks...");
  xTaskCreatePinnedToCore(mpu_measurement_task, "mpu_measurement_task", 4096, NULL, 2, NULL, app_cpu);
  xSemaphoreTake(semaphore_mpu_initialized, portMAX_DELAY);
  xTaskCreatePinnedToCore(theta_obs_task, "theta_obs_task", 4096, NULL, 3, NULL, app_cpu);
  xTaskCreatePinnedToCore(x_psi_obs_task, "x_psi_obs_task", 4096, NULL, 1, NULL, app_cpu);
  xTaskCreatePinnedToCore(reference_task, "reference_task", 2048, NULL, 1, NULL, app_cpu);
  xTaskCreatePinnedToCore(motors_task, "motors_task", 4096, NULL, 4, NULL, app_cpu);
  xTaskCreatePinnedToCore(printing_task, "printing_task", 4096, NULL, 5, NULL, app_cpu);

  Serial.println("All tasks initalized");
  delay(500);
}

/**
 * @brief function contains only non-block delay to avoid problem with watchdog timer
 *
 */
void loop()
{
  vTaskDelay(1000 / portTICK_PERIOD_MS);
}
