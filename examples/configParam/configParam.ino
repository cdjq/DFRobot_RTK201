 /*!
  * @file  configParam.ino
  * @brief config param
  * @copyright Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
  * @license The MIT License (MIT)
  * @author ZhixinLiu(zhixin.liu@dfrobot.com)
  * @version V1.0
  * @date 2023-03-07
  * @url https://github.com/dfrobot/DFRobot_RTK201
  */

#include "DFRobot_RTK201.h"

// must use iic config parameter
#define I2C_COMMUNICATION  //use I2C for communication, but use the serial port for communication if the line of codes were masked

#ifdef  I2C_COMMUNICATION
  DFRobot_RTK201_I2C rtk(&Wire ,DEVICE_ADDR);
#else
#endif

void setup()
{
  Serial.begin(115200);
  while(!rtk.begin()){
    Serial.println("NO Deivces !");
    delay(1000);
  }
  Serial.println("Device connected !");


  /**
    baud_2400 = 0,
    baud_4800 = 1,
    baud_9600 = 2,
    baud_14400 = 3,
    baud_19200 = 4,
    baud_38400 = 5,
    baud_56000 = 6,
    baud_57600 = 7,
    baud_115200 = 8,
  */
  rtk.setMoudleBaud(baud_115200);

  rtk.set4gBaud(baud_115200);

  rtk.setLoraBaud(baud_19200);

  Serial.print("moudle buad = ");
  Serial.println(rtk.getMoudleBaud());

  Serial.print("lora buad = ");
  Serial.println(rtk.getLoraBaud());

  Serial.print("4g buad = ");
  Serial.println(rtk.get4gBaud());
}

void loop()
{
  Serial.println(rtk.transmitAT("$PQTMVERNO*58\r\n"));
  delay(2000);
}