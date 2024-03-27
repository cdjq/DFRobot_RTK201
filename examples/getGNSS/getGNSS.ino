 /*!
  * @file  getGNSS.ino
  * @brief Get gnss simple data
  * @copyright Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
  * @license The MIT License (MIT)
  * @author ZhixinLiu(zhixin.liu@dfrobot.com)
  * @version V1.0
  * @date 2023-03-07
  * @url https://github.com/DFRobot/DFRobot_RTK201
  */

#include "DFRobot_RTK201.h"

#define I2C_COMMUNICATION  //use I2C for communication, but use the serial port for communication if the line of codes were masked

#ifdef  I2C_COMMUNICATION
  DFRobot_RTK201_I2C gnss(&Wire ,DEVICE_ADDR);
#else
/* -----------------------------------------------------------------------------------------------------
 * |  Sensor  | Connect line | Leonardo/Mega2560/M0 |    UNO    | ESP8266 | ESP32 |  microbit  |   m0  |
 * |   VCC    |=============>|        VCC           |    VCC    |   VCC   |  VCC  |     X      |  vcc  |
 * |   GND    |=============>|        GND           |    GND    |   GND   |  GND  |     X      |  gnd  |
 * |   RX     |=============>|     Serial1 TX1      |     5     |   5/D6  |  D2   |     X      |  tx1  |
 * |   TX     |=============>|     Serial1 RX1      |     4     |   4/D7  |  D3   |     X      |  rx1  |
 * ----------------------------------------------------------------------------------------------------*/
/* Baud rate cannot be changed */
  #if defined(ARDUINO_AVR_UNO) || defined(ESP8266)
    SoftwareSerial mySerial(4, 5);
    DFRobot_RTK201_UART gnss(&mySerial, 57600);
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
  gnss.setModule(module_lora);
}

void loop()
{
  sTim_t utc = gnss.getUTC();
  sTim_t date = gnss.getDate();
  sLonLat_t lat = gnss.getLat();
  sLonLat_t lon = gnss.getLon();
  double high = gnss.getAlt();
  uint8_t starUserd = gnss.getNumSatUsed();
  double hdop = gnss.getHdop();
  double sep = gnss.getSep();
  uint8_t mode = gnss.getQuality();
  uint16_t siteID = gnss.getSiteID();
  double diftime = gnss.getDifTime();

  Serial.println("");
  Serial.print(date.year);
  Serial.print("/");
  Serial.print(date.month);
  Serial.print("/");
  Serial.print(date.date);
  Serial.print("/");
  Serial.print(utc.hour);
  Serial.print(":");
  Serial.print(utc.minute);
  Serial.print(":");
  Serial.print(utc.second);
  Serial.println();
  Serial.println((char)lat.latDirection);
  Serial.println((char)lon.lonDirection);
  
  // Serial.print("lat DDMM.MMMMM = ");
  // Serial.println(lat.latitude, 5);
  // Serial.print(" lon DDDMM.MMMMM = ");
  // Serial.println(lon.lonitude, 5);
  Serial.print("lat degree = ");
  Serial.println(lat.latitudeDegree,6);
  Serial.print("lon degree = ");
  Serial.println(lon.lonitudeDegree,6);

  Serial.print("star userd = ");
  Serial.println(starUserd);
  Serial.print("alt high = ");
  Serial.println(high);
  Serial.print("sep  = ");
  Serial.println(sep);

  Serial.print("hdop = ");
  Serial.println(hdop);
  Serial.print("message mode  = ");
  Serial.println(mode);
  Serial.print("siteID = ");
  Serial.println(siteID);
  Serial.print("diftime = ");
  Serial.println(diftime);
  
  Serial.println(gnss.getGnssMessage(gnGGA));
  Serial.println(gnss.getGnssMessage(gnRMC));
  Serial.println(gnss.getGnssMessage(gnGLL));
  Serial.println(gnss.getGnssMessage(gnVTG));
  delay(500);
}
