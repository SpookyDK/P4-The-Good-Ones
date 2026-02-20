#include "NEO_6M_UART.h"
#include "esp_log.h"
#include <driver/uart.h>
#include <stdint.h>
#include <stdio.h>

void app_main(void) {
    gpsInitUart();
    gpsTask();
}
