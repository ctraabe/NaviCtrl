#ifndef SENSOR_ENUMERATION_H_
#define SENSOR_ENUMERATION_H_


enum Sensors {
  LSM303DL,
  UBLOX,
  VISION,
};

enum SensorBits {
  SENSOR_BIT_LSM303DL = 1<<0,
  SENSOR_BIT_UBLOX    = 1<<1,
  SENSOR_BIT_VISION   = 1<<2,
};


#endif  // SENSOR_ENUMERATION_H_
