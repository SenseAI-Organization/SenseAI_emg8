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

bool FIFO_ready = false;

/* Callbacks*/
static void callbackInt2(void *args)
{
    FIFO_ready = true;
}

/* I2C Port Brain v0.2 --> SDA GPIO_NUM_4, SCL GPIO_NUM_5
   Serial port Brain v0.2 --> SDA GPIO_NUM_17, SCL GPIO_NUM_18 */
constexpr gpio_num_t kSDA1 = GPIO_NUM_4;
constexpr gpio_num_t kSCL1 = GPIO_NUM_5;
constexpr gpio_num_t kInt2Pin = GPIO_NUM_20;

constexpr uint8_t samplesNum = 30;


I2C i2c1(I2C_NUM_0, kSDA1, kSCL1, 400000, false); // Change I2C pins

extern "C" void app_main()
{

    printf("\n\n<< ADXL345 Example has started! >>\n\n");

    ADXL345 accel_i2c(i2c1, ADXL345::kAddressAltLow); // SDO to GND
    /**
     * Peripheral Initialization
     */
    esp_err_t err = i2c1.init();
    if (err)
    {
        printf("Error initializing I2C1: %s\n", esp_err_to_name(err));
        while (1)
            ;
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

    // accel_i2c.disableMeasure(); // Enter to Standby Mode before config

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
        accel_i2c.disableMeasure(); // Enter to Standby Mode before config
        vTaskDelay(pdMS_TO_TICKS(5));
    }

    /**
     * Device configuration
     */

    err = accel_i2c.configureOutPutDataRate(ADXL345::OutputDataRate::k800Hz);
    if (err)
    {
        printf("Error while configuring ODR ADXL345: %s. Status: ",
               esp_err_to_name(err));
    }

    err = accel_i2c.setFullResolution();
    if (err)
    {
        printf("Error while set Full Resolution ADXL345: %s. Status: ",
               esp_err_to_name(err));
    }

    err = accel_i2c.configureAccelerationRange(ADXL345::AccelerationRange::k8g);
    if (err)
    {
        printf("Error while configuring accel range ADXL345: %s. Status: ",
               esp_err_to_name(err));
    }
    
    /**
     * To retrieve 30 samples from FIFO
     */
    err = accel_i2c.configureFIFO(ADXL345::FifoMode::kStream,
        samplesNum, ADXL345::Interrupt::INT2); // If not trigger mode, last parameter will be ignored
    if (err)
    {
        printf("Error while configuring FIFO: %s. Status: ",
               esp_err_to_name(err));
    }
    
    /**
     * FOLLOW THIS ORDER TO SET INTERUPTIONS
     * This is the esp-idf order to set interruptions
     */
    err = accel_i2c.assignInterrupts(ADXL345::InterruptFlags::kWatermark, ADXL345::Interrupt::INT2);
    if (err)
    {
        printf("Error while configuring INT ADXL345: %s. Status: ",
               esp_err_to_name(err));
    }

    err = accel_i2c.enableInterrupts(ADXL345::InterruptFlags::kWatermark);

    err = accel_i2c.configurePin(kInt2Pin, ADXL345::Interrupt::INT2, ActiveLevel::kHigh,true);

    err = accel_i2c.attachInterrupt(ADXL345::Interrupt::INT2,
        (adxl345Callback_t) callbackInt2, &accel_i2c);
    if (err)
    {
        printf("Error while attaching int: %s. Status: ",
               esp_err_to_name(err));
    }
    err = accel_i2c.enablePinInterrupt(ADXL345::Interrupt::INT2);
    if (err)
    {
        printf("Error while enabling int: %s. Status: ",
               esp_err_to_name(err));
    }
    printf("Starting measurements!");
    accel_i2c.enableMeasure();

    vTaskDelay(pdMS_TO_TICKS(100));

/**
 * Here starts the implementation!!!
 */
    while (true)
    {
        if(FIFO_ready)
        {
            FIFO_ready = false;
            // Each one of the 32 sample sets holds data of the 3 axis
            int16_t data[samplesNum*3] = {0};
            uint8_t setCounter = 0;

            for(uint8_t i = 0; i < samplesNum; ++i){
                // Accel connected to the i2c port

                err = accel_i2c.measure();
                if (err)
                {
                    printf("Error measuring the ADXL345: %d\n", err);
                }
                accel_i2c.getRawData(data+setCounter);
                setCounter+=3;
            }
            for(uint8_t i = 0; i< samplesNum; ++i){
                printf("%i, %i, %i\n", data[0+i*3], data[1+i*3], data[2+i*3]);
            }
            vTaskDelay(pdMS_TO_TICKS(5));
            
        }
        else{
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }
}
