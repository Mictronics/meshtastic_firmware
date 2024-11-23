#define LED_PIN LED

#define USE_SSD1306 // Heltec_v3 has a SSD1306 display

#define RESET_OLED RST_OLED
#define I2C_SDA SDA_OLED // I2C pins for this board
#define I2C_SCL SCL_OLED

// Enable secondary bus for external periherals
#define I2C_SDA1 SDA
#define I2C_SCL1 SCL

#define VEXT_ENABLE Vext // active low, powers the oled display and the lora antenna boost
#define BUTTON_PIN 0

#define ADC_CTRL 37
#define ADC_CTRL_ENABLED LOW
#define BATTERY_PIN 1 // A battery voltage measurement pin, voltage divider connected here to measure battery voltage
#define ADC_CHANNEL ADC1_GPIO1_CHANNEL
#define ADC_ATTENUATION ADC_ATTEN_DB_2_5 // lower dB for high resistance voltage divider
#define ADC_MULTIPLIER 4.9 * 1.045

#define USE_SX1262

#define LORA_DIO0 -1 // a No connect on the SX1262 module
#define LORA_RESET 12
#define LORA_DIO1 14 // SX1262 IRQ
#define LORA_DIO2 13 // SX1262 BUSY
#define LORA_DIO3    // Not connected on PCB, but internally on the TTGO SX1262, if DIO3 is high the TXCO is enabled

#undef LORA_SCK
#define LORA_SCK 9
#undef LORA_MISO
#define LORA_MISO 11
#undef LORA_MOSI
#define LORA_MOSI 10
#undef LORA_CS
#define LORA_CS 8

#define SX126X_CS LORA_CS
#define SX126X_DIO1 LORA_DIO1
#define SX126X_BUSY LORA_DIO2
#define SX126X_RESET LORA_RESET

#define SX126X_DIO2_AS_RF_SWITCH
#define SX126X_DIO3_TCXO_VOLTAGE 1.8

#define MESHTASTIC_EXCLUDE_WEBSERVER 1
#define MESHTASTIC_EXCLUDE_MQTT 1
#define MESHTASTIC_EXCLUDE_SCREEN 1
#define MESHTASTIC_EXCLUDE_AUDIO 1
#define MESHTASTIC_EXCLUDE_EXTERNALNOTIFICATION 1
#define MESHTASTIC_EXCLUDE_PAXCOUNTER 1
#define MESHTASTIC_EXCLUDE_RANGETEST 1
#define MESHTASTIC_EXCLUDE_REMOTEHARDWARE 1
#define MESHTASTIC_EXCLUDE_STOREFORWARD 1
#define MESHTASTIC_EXCLUDE_ATAK 1
#define MESHTASTIC_EXCLUDE_CANNEDMESSAGES 1
#define MESHTASTIC_EXCLUDE_WAYPOINT 1
#define MESHTASTIC_EXCLUDE_INPUTBROKER 1
#define MESHTASTIC_EXCLUDE_POWERSTRESS 1