#ifndef AvionicsConsts_h
#define AvionicsConsts_h

#define DEBUG_SERIAL                1     // set 0 for flight;
#define DEBUG_BOARD                 1     // set 0 for flight;
#define DEBUG_BUZZER                0     // set 0 for flight; set 1 for silence;

#define UPDATE_RATE_STATE           20    // hertz;
#define UPDATE_RATE_GPS             1     // hertz;
#define UPDATE_RATE_TELEMETRY       1     // hertz;

#define UPDATE_DELAY_STATE          1000/UPDATE_RATE_STATE      
#define UPDATE_DELAY_GPS            1000/UPDATE_RATE_GPS          
#define UPDATE_DELAY_TELEMETRY      1000/UPDATE_RATE_TELEMETRY    

#define AGL_CALIBRATION_SAMPLES     100   // vector size;
#define AGL_CALIBRATION_TIME        5     // seconds;

#define AGL_VARIANCE_THRESHOLD      10    // meters;

#define IMU_ADDRESS                 0x68  // hexadecimal address
#define BMP280_ADDRESS              0x77  // hexadecimal address

#define RFM95_CS                    15
#define RFM95_RST                   30
#define RFM95_INT                   29
#define RF95_FREQ                   915.0
#define PACKET_SIZE                 8

#define DROGUE_PIN                  36    // teensy pin for drogue servo motor.
#define MAIN_PIN                    35    // teensy pin for main servo motor.

#define DROGUE_TIME_MS              1000  // activation pulse time in milliseconds.
#define MAIN_TIME_MS                1000  // activation pulse time in milliseconds.

#define BUZZER_PIN                  25    // teensy pin for buzzer audio in OBC.
#define LED_R_PIN                   26    // teensy pin for red led in OBC.
#define LED_Y_PIN                   27    // teensy pin for yello led in OBC.
#define LED_G_PIN                   28    // teensy pin for green led in OBC.

#define DEBUG_INIT_TIME             500   // debug buzzer and led testing time.

#define MEMORY_SIZE                 20    // vector size
#define FILTER_SIZE                 5     // vector size

#define THRESHOLD_LIFTOFF           50    // altitude AGL;
#define THRESHOLD_PARACHUTE         0.90  // percentage;
#define THRESHOLD_MAIN              500   // altitude AGL;
#define THRESHOLD_TOUCHDOWN         50    // altitude AGL;

#endif