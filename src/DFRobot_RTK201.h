/*!
 * @file DFRobot_RTK201.h
 * @brief Define the basic structure of the DFRobot_RTK201 class, the implementation of the basic methods
 * @copyright	Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @license The MIT License (MIT)
 * @author [ZhixinLiu](zhixin.liu@dfrobot.com)
 * @version V1.0
 * @date 2023-03-07
 * @url https://github.com/DFRobot/DFRobot_RTK201
 */
#ifndef __DFRobot_GNSS_H__
#define __DFRobot_GNSS_H__

#include <Arduino.h>
#include <Wire.h>

#if defined(ARDUINO_AVR_UNO) || defined(ESP8266)
#include "SoftwareSerial.h"
#else
#include "HardwareSerial.h"
#endif

/**
 * @struct sTim_t
 * @brief Store the time and date information obtained from GPS 
 */
typedef struct {
  uint16_t year;
  uint8_t month;
  uint8_t date;
  uint8_t hour;
  uint8_t minute;
  uint8_t second;
}sTim_t;

/**
 * @struct sLonLat_t
 * @brief Store latitude, longitude and direction information obtained from GPS 
 */
typedef struct {
  uint8_t lonDDD;
  uint8_t lonMM;
  uint32_t lonMMMMM;
  char lonDirection;
  uint8_t latDD;
  uint8_t latMM;
  uint32_t latMMMMM;
  char latDirection;
  double latitude;
  double latitudeDegree;
  double lonitude;
  double lonitudeDegree;
}sLonLat_t;

enum GNSS_Mode {
  gnGGA,
  gnRMC,
  gnGLL,
  gnVTG,
};

enum Module {
  module_4g = 20,
  module_lora = 10,
};

enum Module_Baud {
  baud_2400 = 0,
  baud_4800 = 1,
  baud_9600 = 2,
  baud_14400 = 3,
  baud_19200 = 4,
  baud_38400 = 5,
  baud_56000 = 6,
  baud_57600 = 7,
  baud_115200 = 8,
  baud_230400 = 9,
  baud_512000 = 10,
  baud_921600 = 11,
};


typedef struct {
  char  gga[120];
  char  rmc[120];
  char  gll[100];
  char  vtg[60];
  char  cmd[120];
} sSource_t;

class DFRobot_RTK201{
public:
  #define MAX_LEN       3000
  #define REG_YEAR_H 		0
  #define REG_YEAR_L 		1
  #define REG_MONTH 		2
  #define REG_DATE  		3
  #define REG_HOUR  		4
  #define REG_MINUTE 		5
  #define REG_SECOND 		6
  #define REG_LAT_1 		7
  #define REG_LAT_2 		8
  #define REG_LAT_X_24 	9
  #define REG_LAT_X_16 	10
  #define REG_LAT_X_8  	11
  #define REG_LAT_DIS  	12
  #define REG_LON_1 		13
  #define REG_LON_2 		14
  #define REG_LON_X_24 	15
  #define REG_LON_X_16 	16
  #define REG_LON_X_8  	17
  #define REG_LON_DIS  	18
  #define REG_GPS_STATE	19
  #define REG_USE_STAR 	20
  #define REG_HDOP_Z		21
  #define REG_HDOP_X		22
  #define REG_ALT_H 		23
  #define REG_ALT_L 		24
  #define REG_ALT_X 		25
  #define REG_SEP_H 		26
  #define REG_SEP_L 		27
  #define REG_SEP_X 		28
  #define REG_DIF_Z			29
  #define REG_DIF_X			30
  #define REG_DIFID_H		31
  #define REG_DIFID_L		32

  #define REG_I2C_ID    50     ///< uart device id
  //  32~~~80
  #define REG_GGA_LEN		81
  #define REG_GGA_ALL		82
  #define REG_RMC_LEN		83
  #define REG_RMC_ALL		84
  #define REG_GLL_LEN		85
  #define REG_GLL_ALL		86
  #define REG_VTG_LEN		87
  #define REG_VTG_ALL		88
  #define REG_ALL_LEN_H	89
  #define REG_ALL_LEN_L	90
  #define REG_ALL				91
  #define REG_ALL_MODE	92
  #define REG_OPERATION	93
  #define REG_LORA_BAUD	94
  #define REG_4G_BAUD 	95
  #define REG_UNO_BAUD	96
  #define REG_TRANS_AT	97
  #define REG_T_AT_LEN	98
  #define REG_TRANMIT 	99
  #define REG_RECV_AT	  100
  #define REG_R_AT_LEN	101
  #define REG_USER_NAME	102
  #define REG_USER_NAME_LEN	103
  #define REG_USER_PASSWORD	104
  #define REG_USER_PASSWORD_LEN	105
  #define REG_SERVER_ADDR	106
  #define REG_SERVER_ADDR_LEN	107
  #define REG_MOUNT_POINT	108
  #define REG_MOUNT_POINT_LEN	109
  #define REG_PORT_H	110
  #define REG_PORT_L	111
  #define REG_CONNECT	112
  #define REG_CONNECT_STATE	113


  #define I2C_FLAG  1
  #define UART_FLAG 2
  #define TIME_OUT  100            ///< time out
  #define DEVICE_ADDR 0x20

  DFRobot_RTK201();
  ~DFRobot_RTK201();
  uint8_t  uartI2CFlag = 0;

/**
 * @fn getUTC
 * @brief Get UTC, standard time 
 * @return sTim_t type, represents the returned hour, minute and second 
 * @retval sTim_t.hour hour 
 * @retval sTim_t.minute minute 
 * @retval sTim_t.second second 
 */
  sTim_t getUTC(void);

/**
 * @fn getDate
 * @brief Get date information, year, month, day 
 * @return sTim_t type, represents the returned year, month, day 
 * @retval sTim_t.year year
 * @retval sTim_t.month month 
 * @retval sTim_t.day day 
 */
  sTim_t getDate(void);

/**
 * @fn getLat
 * @brief Get latitude 
 * @return sLonLat_t type, represents the returned latitude  
 * @retval sLonLat_t.latDD   Latitude degree(0-90)
 * @retval sLonLat_t.latMM   The first and second digits behind the decimal point 
 * @retval sLonLat_t.latMMMMM Latitude  The third and seventh digits behind the decimal point 
 * @retval sLonLat_t.latitude Latitude value with 7 decimal digits
 * @retval sLonLat_t.latDirection Direction of latitude
 */
  sLonLat_t getLat(void);

/**
 * @fn getLon
 * @brief Get longitude 
 * @return sLonLat_t Type, represents the returned longitude
 * @retval sLonLat_t.lonDDD  Longitude degree(0-90)
 * @retval sLonLat_t.lonMM   Longitude  The first and second digits behind the decimal point
 * @retval sLonLat_t.lonMMMMM Longitude The third and seventh digits behind the decimal point
 * @retval sLonLat_t.lonitude Longitude value with 7 decimal digits
 * @retval sLonLat_t.lonDirection Direction of longitude 
 */
  sLonLat_t getLon(void);

/**
 * @fn getNumSatUsed
 * @brief Get the number of the used satellite used
 * @return uint8_t type, represents the number of the used satellite
 */
  uint8_t getNumSatUsed(void);

/**
 * @fn getAlt
 * @brief Get altitude
 * @return double type, represents altitude 
 */
  double getAlt(void);

/**
 * @fn getSep
 * @brief Get the Sep object
 * 
 * @return double 
 */
  double getSep(void);

/**
 * @fn getHdop
 * @brief Get the Hdop object
 * 
 * @return double
 */
  double getHdop(void);

/**
 * @fn getQuality
 * @brief Get the Quality object
 * 
 * @return uint8_t 
 */
  uint8_t getQuality(void);

/**
 * @brief Get the Site ID object
 * 
 * @return uint16_t
 */
  uint16_t getSiteID(void);


/**
 * @brief Get the Dif Time object
 * 
 * @return double 
 */
  double getDifTime(void);

  void setModule(Module mode);

  Module getModule(void);


  /**
   * @brief transmitAT
   * 
   * @return char *
   */
  char * transmitAT(const char* cmd);

/**
 * @brief Get the Source object
 * 
 * @param mode 
 * @return char* 
 */
  char * getSource(GNSS_Mode mode);

/**
 * @fn getSog
 * @brief Get speed over ground 
 * @return speed Float data(unit: knot)
 */
  double getSog(void);

/**
 * @fn getCog
 * @brief Get course over ground
 * @return Float data(unit: degree) 
 */
  double getCog(void);

/**
 * @fn getAllGnss
 * @brief Get GNSS data, call back and receive
 * @return null
 */
  void getAllGnss(void);

/**
 * @brief Set the Moudle Baud object
 * 
 * @param baud 
 */
  void setMoudleBaud(Module_Baud baud);

/**
 * @brief Set the 4g  Baud object
 * 
 * @param baud 
 */
  void set4gBaud(Module_Baud baud);

/**
 * @brief Set the Lora Baud object
 * 
 * @param baud 
 */
  void setLoraBaud(Module_Baud baud);

/**
 * @brief Get the Moudle Baud object
 * 
 * @return uint32_t 
 */
  uint32_t getMoudleBaud(void);

/**
 * @brief Get the Lora Baud object
 * 
 * @return uint32_t 
 */
  uint32_t getLoraBaud(void);
/**
 * @brief 
 * 
 */
  uint32_t get4gBaud(void);

/**
 * @brief 
 * 
 * @param baud 
 * @return uint32_t 
 */
  uint32_t baudMatch(Module_Baud baud);

/**
 * @brief Set the User Name object
 * 
 * @param name 
 * @param len 
 */
  void setUserName(const char *name, uint8_t len);

/**
 * @brief Set the User Password object
 * 
 * @param password 
 * @param len 
 */
  void setUserPassword(const char *password, uint8_t len);

/**
 * @brief Set the Server Addr object
 * 
 * @param addr 
 * @param len 
 */
  void setServerAddr(const char *addr, uint8_t len);

/**
 * @brief setMountPoint
 * 
 * @param point 
 * @param len 
 */
  void setMountPoint(const char *point, uint8_t len);

/**
 * @brief Set the Port object
 * 
 * @param port 
 */
  void setPort(uint16_t port);


  String connect(void);

/**
 * @fn setCallback
 * @brief Set callback function type
 * @param  call function name 
 * @return null
 */
  void setCallback(void (*call)(char *, uint8_t));


  void (* callback)(char *data, uint8_t len);
private:
  uint8_t  _addr;
  uint8_t  _M_Flag = 0;
  sSource_t __sourceData;
/**
 * @fn getGnssLen
 * @brief Get length of gnss data 
 * @return Length 
 */
  uint16_t getGnssLen(void);
  virtual void writeReg(uint8_t reg, uint8_t *data, uint8_t len) = 0;
  virtual int16_t readReg(uint8_t reg, uint8_t *data, uint8_t len) = 0;
};

class DFRobot_RTK201_I2C:public DFRobot_RTK201{
public:
  DFRobot_RTK201_I2C(TwoWire *pWire=&Wire, uint8_t addr = 0x75);
  bool begin(void);
protected:
  virtual void writeReg(uint8_t reg, uint8_t *data, uint8_t len);
  virtual int16_t readReg(uint8_t reg, uint8_t *data, uint8_t len);
private:
  TwoWire *_pWire;
  uint8_t _I2C_addr;
};

class DFRobot_RTK201_UART:public DFRobot_RTK201{
public:
#if defined(ARDUINO_AVR_UNO) || defined(ESP8266)
  DFRobot_RTK201_UART(SoftwareSerial *sSerial, uint32_t Baud);
#else
  DFRobot_RTK201_UART(HardwareSerial *hSerial, uint32_t Baud ,uint8_t rxpin = 0, uint8_t txpin = 0);
#endif

  bool begin(void);
protected:
  virtual void writeReg(uint8_t reg, uint8_t *data, uint8_t len);
  virtual int16_t readReg(uint8_t reg, uint8_t *data, uint8_t len);
private:

#if defined(ARDUINO_AVR_UNO) || defined(ESP8266)
  SoftwareSerial *_serial;
#else
  HardwareSerial *_serial;
#endif
  uint32_t _baud;
  uint8_t _rxpin;
  uint8_t _txpin;
};
#endif
