#ifndef CONFIG_H_
#define CONFIG_H_

#define TEMP_READ_INTERVAL_MS 300000
#define LED_RESULT_DISPLAY_LENGTH_MS 10000

#if LED_RESULT_DISPLAY_LENGTH_MS > TEMP_READ_INTERVAL_MS
#error "Invalid LED result display length."
#endif

#define URL_BASE "http://192.168.3.3:5555/?temp="

#define MAC_ADDR { 0x02, 0x12, 0x13, 0x10, 0x15, 0x11 }

#endif
