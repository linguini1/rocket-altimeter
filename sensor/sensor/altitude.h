#ifndef __ROCKETALT_SENSOR_ALTFUSION_H
#define __ROCKETALT_SENSOR_ALTFUSION_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <uORB/uORB.h>

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct fusion_altitude
{
  uint64_t timestamp; /* Timestamp in microseconds */
  float altitude;     /* Altitude in meters */
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

ORB_DECLARE(fusion_altitude);

#endif /* __ROCKETALT_SENSOR_ALTFUSION_H */
