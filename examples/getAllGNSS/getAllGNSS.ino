 /*!
  * @file  getAllGNSS.ino
  * @brief read all gnss data
  * @copyright Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
  * @license The MIT License (MIT)
  * @author ZhixinLiu(zhixin.liu@dfrobot.com)
  * @version V1.0
  * @date 2023-03-07
  * @url https://github.com/dfrobot/DFRobot_RTK201
  */

#include "DFRobot_RTK201.h"

void callback(char *data, uint8_t len)
{
  for(uint8_t i = 0; i < len; i++){
    Serial.print((char)data[i]);
  }
}

#define I2C_COMMUNICATION  //use I2C for communication, but use the serial port for communication if the line of codes were masked

#ifdef  I2C_COMMUNICATION
  DFRobot_RTK201_I2C gnss(&Wire ,DEVICE_ADDR);
#else
/* ---------------------------------------------------------------------------------------------------------------------
 *    board   |             MCU                | Leonardo/Mega2560/M0 |    UNO    | ESP8266 | ESP32 |  microbit  |   m0  |
 *     VCC    |            3.3V/5V             |        VCC           |    VCC    |   VCC   |  VCC  |     X      |  vcc  |
 *     GND    |              GND               |        GND           |    GND    |   GND   |  GND  |     X      |  gnd  |
 *     RX     |              TX                |     Serial1 TX1      |     5     |   5/D6  |  D2   |     X      |  tx1  |
 *     TX     |              RX                |     Serial1 RX1      |     4     |   4/D7  |  D3   |     X      |  rx1  |
 * ----------------------------------------------------------------------------------------------------------------------*/
/* Baud rate cannot be changed  */
  #if defined(ARDUINO_AVR_UNO) || defined(ESP8266)
    SoftwareSerial mySerial(4, 5);
    DFRobot_RTK201_UART gnss(&mySerial, 115200);
  #elif defined(ESP32)
    DFRobot_RTK201_UART gnss(&Serial1, 115200 ,/*rx*/D2 ,/*tx*/D3);
  #else
    DFRobot_RTK201_UART gnss(&Serial1, 115200);
  #endif
#endif

void setup()
{
  Serial.begin(115200);
  while(!gnss.begin()){
    Serial.println("NO Deivces !");
    delay(1000);
  }
  Serial.println("Device connected !");

  /*
    module_4g
    module_lora
  */
  gnss.setModule(module_lora);

  gnss.setCallback(callback);
}

void loop()
{
  gnss.getAllGnss();
}