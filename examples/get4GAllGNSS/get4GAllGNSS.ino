 /*!
  * @file  getAllGNSS.ino
  * @brief read all rtk data at 4G mode
  * @copyright Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
  * @license The MIT License (MIT)
  * @author ZhixinLiu(zhixin.liu@dfrobot.com)
  * @version V1.0
  * @date 2023-03-07
  * @url https://github.com/DFRobot/DFRobot_RTK201
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
  DFRobot_RTK201_I2C rtk(&Wire ,DEVICE_ADDR);
#else
/* -----------------------------------------------------------------------------------------------------
 * |  Sensor  | Connect line | Leonardo/Mega2560/M0 |    UNO    | ESP8266 | ESP32 |  microbit  |   m0  |
 * |   VCC    |=============>|        VCC           |    VCC    |   VCC   |  VCC  |     X      |  vcc  |
 * |   GND    |=============>|        GND           |    GND    |   GND   |  GND  |     X      |  gnd  |
 * |   RX     |=============>|     Serial1 TX1      |     5     |   5/D6  |  D2   |     X      |  tx1  |
 * |   TX     |=============>|     Serial1 RX1      |     4     |   4/D7  |  D3   |     X      |  rx1  |
 * ----------------------------------------------------------------------------------------------------*/
/* Baud rate cannot be changed  */
  #if defined(ARDUINO_AVR_UNO) || defined(ESP8266)
    SoftwareSerial mySerial(4, 5);
    DFRobot_RTK201_UART rtk(&mySerial, 57600);
  #elif defined(ESP32)
    DFRobot_RTK201_UART rtk(&Serial1, 115200 ,/*rx*/D2 ,/*tx*/D3);
  #else
    DFRobot_RTK201_UART rtk(&Serial1, 115200);
  #endif
#endif


#define  USER_NAME      "chwj068746"
#define  USER_PASSWORD  "16409678"
#define  SERVER_ADDR    "119.3.136.126"
#define  MOUNT_POINT    "RTCM33"
uint16_t port = 8002;
String   result = "";

void setup()
{
  Serial.begin(115200);
  while(!rtk.begin()){
    Serial.println("NO Deivces !");
    delay(1000);
  }
  Serial.println("Device connected !");

  rtk.setModule(module_4g);

  rtk.setUserName(USER_NAME, strlen(USER_NAME));

  rtk.setUserPassword(USER_PASSWORD, strlen(USER_PASSWORD));

  rtk.setServerAddr(SERVER_ADDR, strlen(SERVER_ADDR));

  rtk.setMountPoint(MOUNT_POINT, strlen(MOUNT_POINT));

  rtk.setPort(port);
  
  result = rtk.connect();

  if((String)"CONNECT SUCCESSFUL" == result){
    Serial.println("connect success");
  }else{
    Serial.println(result);
  }

  rtk.setCallback(callback);
}

void loop()
{
  // Please note that there is no judgment of timeout reconnection for the 4G module here
  rtk.getAllGnss();
  if(!rtk.getConnectState()){
  result = rtk.connect();
  if((String)CONNECT_SUCCESS == result){
    Serial.println("connect success");
  }else{
    Serial.println(result);
  }
  delay(1000);
  }
}
