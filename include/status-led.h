// #define STATUS_LED 22

// enum led_state {
//   led_off,
//   led_on,
//   breathing,
//   fast_blink,
//   slow_blink,
//   very_slow_blink
// };

// void startLEDTask();

// void setStatusLED(led_state newState);
void blinkStatusLED(int millis, int flashesPerSec = 10);