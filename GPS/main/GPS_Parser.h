#ifndef GPS_PARSER_H
#define GPS_PARSER_H

#include "freertos/idf_additions.h"
#include <stdint.h>

#define PI 3.14
#define GPS_UART_NUM      UART_NUM_1
#define GPS_RX_PIN        1
#define GPS_TX_PIN        2 
#define BUF_SIZE 2048
struct Coordinate {
    float lat;
    float lon;
    float UTC;
    int height;
};

struct GPGGA_Message {
    char ID[6];   
    float UTC;         
    double latitude;
    char direction_Latitude;
    double longitude;
    char direction_Longitude;
    int GPS_Quality_indicator;
    int number_of_GPS;
    float HDOP;
    float height;
    char unit_of_height; 
    float geoid_Separation; 
    char geoid_unit; 
    char checkSum[3];
};

/************** DATA STRUCT ************/
#ifndef WORKOUT_DATA_H
#define WORKOUT_DATA_H

#include <stdint.h>   // needed for uint32_t, uint16_t, etc.

#endif // WORKOUT_DATA_H

// Function prototypes
int isCheckSumTrue(const char *sentence);
int calcCheckSum(uint8_t *sentence, uint8_t messageLengthInc);
int parseGPGGA(const char *sentence, struct GPGGA_Message *msg);
double convertToDecimalDegrees(float raw, char dir);
int GPSParser(char* sentence, struct Coordinate *retCoordinate);
void init_gps_uart();
double distance_meters(double lat1, double lon1, double lat2, double lon2);
static struct Coordinate lastCoordinate;
volatile extern struct Coordinate newCoordinate;
static char file_name_and_location[128];
static uint8_t data[1024]; 
extern SemaphoreHandle_t gps_mutex;
volatile extern bool data_to_write;
void gps_task(void *arg);

#endif // GPS_PARSER_H
