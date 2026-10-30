#define I2C_SDA 9
#define I2C_SCL 40

#define USE_SX1262

// RAK13302 (1 W SX1262) on the WisBlock IO slot, not the onboard SX1262
#define LORA_SCK   13
#define LORA_MISO  10
#define LORA_MOSI  11
#define LORA_CS    12

#define SX126X_CS      LORA_CS
#define SX126X_DIO1    39   // WB_IO6
#define SX126X_BUSY    38   // WB_IO5
#define SX126X_RESET   42   // WB_IO4
#define SX126X_POWER_EN 41  // WB_IO3 = ANT_SW
#define SX126X_DIO2_AS_RF_SWITCH
#define SX126X_DIO3_TCXO_VOLTAGE 1.8

// 3V3_S rail (WB_IO2), powers the RAK13302
#define PIN_3V3_EN 14
#define PIN_POWER_EN PIN_3V3_EN

#define LED_GREEN 46
#define LED_BLUE 45
#define PIN_LED1 LED_GREEN
#define LED_NOTIFICATION LED_BLUE
#define LED_POWER LED_GREEN
#define ledOff(pin) pinMode(pin, INPUT)
#define LED_STATE_ON 1

#define BATTERY_PIN 1
#define ADC_CHANNEL ADC_CHANNEL_0
#define ADC_MULTIPLIER 2.11
#define BUTTON_NEED_PULLUP

#define I2C_SDA1 PIN_WIRE1_SDA
#define I2C_SCL1 PIN_WIRE1_SCL