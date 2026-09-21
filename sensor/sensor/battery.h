#ifndef __ROCKETALT_SENSOR_BATTERY_H
#define __ROCKETALT_SENSOR_BATTERY_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <uORB/uORB.h>

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct sensor_battery
{
  uint64_t timestamp; /* Timestamp in microseconds */
  float voltage;      /* Voltage in Volts */
  uint8_t level;      /* Battery level in percentage */
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

ORB_DECLARE(sensor_battery);

#endif /* __ROCKETALT_SENSOR_BATTERY_H */
