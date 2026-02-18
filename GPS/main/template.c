#include <stdio.h>
#include <stdint.h> 
#include "GPS_Parser.h"
#include <driver/uart.h>
#include "esp_log.h"
uint8_t ccfg_rate_5hz[] = {
    0xB5, 0x62, // UBX header
    0x06, 0x08, // CFG-RATE
    0x06, 0x00, // payload length = 6
    0x10, 0x27, // measurement rate = 200 ms (5 Hz)
    0x01, 0x00, // navigation rate (cycles per measurement)
    0x01, 0x00, // time reference: GPS time
    0xDE, 0x6A  // checksum
};


void app_main(void)
{
    calcCheckSum(ccfg_rate_5hz, 14);
    init_gps_uart();
    gps_task(NULL);


}

