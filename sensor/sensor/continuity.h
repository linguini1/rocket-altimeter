#ifndef __ROCKETALT_SENSOR_CONTINUITY_H
#define __ROCKETALT_SENSOR_CONTINUITY_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <uORB/uORB.h>

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct sensor_continuity
{
  uint64_t timestamp; /* Timestamp in microseconds */
  uint8_t id;         /* Channel ID associated with the measurement */
  uint8_t continuous; /* Continuous (0 no, 1 yes) */
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

ORB_DECLARE(sensor_continuity);

#endif /* __ROCKETALT_SENSOR_CONTINUITY_H */
