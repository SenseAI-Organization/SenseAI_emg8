/*******************************************************************************
 * @file ADXL345_example.cpp
 * @brief File to test the ADXL345 class.
 *
 * @version 0.1.3
 * @date 2024-12-10
 * @author Sense AI
 *******************************************************************************
 *******************************************************************************/

 #include "esp_system.h"
 #include "freertos/FreeRTOS.h"
 #include "freertos/task.h"
 #include "freertos/queue.h"
 
 #include "ADXL345.hpp"
 
 bool accelToggle = false;
 
 /* Callbacks*/
 static void callbackInt1(void *args)
 {
     accelToggle = true;
 }
 
 /* I2C Port Brain v0.2 --> SDA GPIO_NUM_4, SCL GPIO_NUM_5
    Serial port Brain v0.2 --> SDA GPIO_NUM_17, SCL GPIO_NUM_18 */
 constexpr gpio_num_t kSDA1 = GPIO_NUM_4;
 constexpr gpio_num_t kSCL1 = GPIO_NUM_5;
 I2C i2c1(I2C_NUM_0, kSDA1, kSCL1, 400000, false); // Change I2C pins
 
 extern "C" void app_main()
 {
     static bool asleep = false;
 
     printf("\n\n<< ADXL345 Example has started! >>\n\n");
     ADXL345 accel_i2c(i2c1, ADXL345::kAddressAltLow);

     /**
      * Peripheral Initialization
      */
     esp_err_t err = i2c1.init();
     if (err)
     {
         printf("Error initializing I2C1: %s\n", esp_err_to_name(err));
         while (1);
     }
     /**
      * Device Initialization
      */
     err = accel_i2c.init();
     if (err)
     {
         printf("Error initializing ADXL345: %s. Trying again. Status: ",
                esp_err_to_name(err));
         err = accel_i2c.init();
         printf("%s\n", esp_err_to_name(err));
     }
     /**
      * Device calibration
      */
 
     // Enter to Standby Mode before config
 
     printf("Calibrating the ADXL345 sensor (i2c). Don't move it!\n");
     err = accel_i2c.calibrate();
     if (err)
     {
         printf("Error calibrating ADXL345: %s.\n", esp_err_to_name(err));
         while (1)
         {
             vTaskDelay(portMAX_DELAY);
         }
     }
     else
     {
         printf("ADXL345 calibrated\n");
         vTaskDelay(pdMS_TO_TICKS(10));
         accel_i2c.disableMeasure(); // Enter to Standby Mode before config
         vTaskDelay(pdMS_TO_TICKS(10));
     }
 
     /**
      * Device configuration
      */
     
     err = accel_i2c.configureActivityThreshold(30);
     err = accel_i2c.configureInactivityThreshold(10);
     err = accel_i2c.configureInactivityTime(2);
     if (err)
     {
         printf("Error while configuring ActInact thresholds ADXL345: %s. Status: ",
                esp_err_to_name(err));
     }
 
     err = accel_i2c.configurePowerControl(ADXL345::PowerControlBits::kLink 
         | ADXL345::PowerControlBits::kAutoSleep 
         | ADXL345::PowerControlBits::kWakeUp8Hz);
     if (err)
     {
         printf("Error while configuring Power ADXL345: %s. Status: ",
                esp_err_to_name(err));
     }
 
     err = accel_i2c.configureOutPutDataRate(ADXL345::OutputDataRate::k400Hz);
     if (err)
     {
         printf("Error while configuring ODR ADXL345: %s. Status: ",
                esp_err_to_name(err));
     }
 
     err = accel_i2c.setFullResolution();
     if (err)
     {
         printf("Error while setting Full resolution ADXL345: %s. Status: ",
                esp_err_to_name(err));
     }
 
     err = accel_i2c.configureAccelerationRange(ADXL345::AccelerationRange::k4g);
     if (err)
     {
         printf("Error while configuring accel g-range ADXL345: %s. Status: ",
                esp_err_to_name(err));
     }
 
     err = accel_i2c.configureActivityControl(ADXL345::ActivityControl::kActZ 
         | ADXL345::ActivityControl::kInactZ);
     if (err)
     {
         printf("Error while configuring Activity ADXL345: %s. Status: ",
                esp_err_to_name(err));
     }
     /**
      * FOLLOW THIS ORDER TO SET INTERUPTIONS
      * This is the esp-idf order to set interruptions
      */
     err = accel_i2c.assignInterrupts(ADXL345::InterruptFlags::kActivity 
         | ADXL345::InterruptFlags::kInactivity, ADXL345::Interrupt::INT1);
     if (err)
     {
         printf("Error while configuring assigning int ADXL345: %s. Status: ",
                esp_err_to_name(err));
     }
     err |= accel_i2c.configurePin(GPIO_NUM_19, ADXL345::Interrupt::INT1,
                                 ActiveLevel::kHigh, false);
 
     err = accel_i2c.attachInterrupt(ADXL345::Interrupt::INT1,
         (adxl345Callback_t) callbackInt1, &accel_i2c);
     if (err)
     {
         printf("Error while attaching int: %s. Status: ",
                esp_err_to_name(err));
     }

     err |= accel_i2c.enableInterrupts(
        ADXL345::InterruptFlags::kActivity | ADXL345::InterruptFlags::kInactivity);
 
     accel_i2c.enableMeasure();
 
     vTaskDelay(pdMS_TO_TICKS(200));
 /**
  * Here starts the implementation!!!
  */
     while (true)
     {
         if (accelToggle)
         {
             accelToggle = false;
             printf("Accel sleep state toggles\n");
 
             bool aux = accel_i2c.isIntFlagUp(ADXL345::InterruptFlags::kActivity);
             uint8_t auxMask = 0b1; // Activity and Inactivity
 
             if ((aux >> 3) & (auxMask))
             {
                 printf("Accel is Asleep!\n");
                 asleep = true;
             }
             if ((aux >> 4) & (auxMask))
             {
                 printf("Accel is Awake!\n");
                 asleep = false;
             }
         }
 
         if(!asleep)
         {
             // Accel connected to the i2c port
             while (!accel_i2c.checkDataReady())
                 ; // Wait for data to be ready (polling)
 
             err = accel_i2c.measure();
             if (err)
             {
                 printf("Error measuring the ADXL345: %d\n", err);
             }
 
             int16_t data[3] = {0};
             accel_i2c.getRawData(data);
             printf("%i, %i, %i\n", data[0], data[1], data[2]);
 
             vTaskDelay(pdMS_TO_TICKS(10));
         }
         else{
             printf("Accel has fall asleep!\n");
             vTaskDelay(pdMS_TO_TICKS(100));
         }
     }
 }