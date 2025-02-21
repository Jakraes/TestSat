#include <Arduino.h>
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "usb_serial.h"  // Teensy 4.1 USB Serial
#include "FXUtil.h"		// read_ascii_line(), hex file support
extern "C" {
#include "FlashTxx.h"		// TLC/T3x/T4x/TMM flash primitives
}

///home/alexandre/.platformio/packages/framework-arduinoteensy-ts/teensy4/imxrt1062_t41.ld, change ld TODO Changed .ld Dont forget !!!! previos = 7936K
#define LED_PIN 13
#define QUEUE_LENGTH 32
#define ITEM_SIZE sizeof(char)

// Queue handle
QueueHandle_t xSerialQueue;
uint32_t buffer_addr, buffer_size;
// Serial Read Task
void vUpdateTask(void *pvParameters) {

    // create flash buffer to hold new firmware
    if (firmware_buffer_init( &buffer_addr, &buffer_size ) == 0) {
        Serial.printf( "unable to create buffer\n" );
        Serial.flush();
        for (;;) {}
    }

    Serial.printf( "created buffer = %1luK %s (%08lX - %08lX)\n",
          buffer_size/1024, IN_FLASH(buffer_addr) ? "FLASH" : "RAM",
          buffer_addr, buffer_addr + buffer_size );
    vTaskDelay(pdMS_TO_TICKS(10));
    /*
    if (update_firmware(&Serial, &Serial, buffer_addr, buffer_size)) {
        Serial.printf("Firmware update failed!\n");
        firmware_buffer_free(buffer_addr, buffer_size);
        Serial.flush();
        REBOOT;
    }
    */
    vTaskDelete(NULL);
}

void vCommitUpdate(void *pvParameters) {
    commit_update( &Serial, &Serial,buffer_addr);
    vTaskDelay(pdMS_TO_TICKS(10));
    REBOOT;
    vTaskDelete(NULL);
}

void vSerialReadTask(void *pvParameters) {
    char receivedChar;
    while (1) {
        if (usb_serial_available()) {
            receivedChar = usb_serial_getchar();  // Read from USB serial
            if(receivedChar == '1') {
                xTaskCreate(vUpdateTask, "vUpdate", 512, NULL, 5, NULL);
            }
            else if(receivedChar == '3') {
                //xTaskCreate(vCommitUpdate, "vCommitUpdate", 512, NULL, 5, NULL);
            }else {
                xQueueSend(xSerialQueue, &receivedChar, portMAX_DELAY);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1));  // Small delay to avoid CPU overload
    }
}

// Serial Write Task
void vSerialWriteTask(void *pvParameters) {
    char receivedChar;
    while (1) {
        if (xQueueReceive(xSerialQueue, &receivedChar, portMAX_DELAY)) {
            usb_serial_putchar(receivedChar);  // Echo received character
        }
    }
}



void vBlinkTask(void *pvParameters) {
    pinMode(LED_PIN, OUTPUT);
    while (1) {
        digitalWrite(LED_PIN, !digitalRead(LED_PIN));
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void setup() {
    //usb_serial_begin();  // Initialize USB Serial
    delay(2000);

    xSerialQueue = xQueueCreate(QUEUE_LENGTH, ITEM_SIZE);

    if (xSerialQueue != NULL) {
        xTaskCreate(vSerialReadTask, "SerialRead", 512, NULL, 2, NULL);
        xTaskCreate(vSerialWriteTask, "SerialWrite", 512, NULL, 3, NULL);
        xTaskCreate(vBlinkTask, "Blink", 256, NULL, 4, NULL);

        vTaskStartScheduler();
    } else {

        //usb_serial_write("Queue creation failed!\n");
    }
}

void loop() {
    // Not used in FreeRTOS setup
}
