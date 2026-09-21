#ifndef __ROCKETALT_SENSOR_HEIGHT_H
#define __ROCKETALT_SENSOR_HEIGHT_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <uORB/uORB.h>

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct fusion_height
{
  uint64_t timestamp; /* Timestamp in microseconds */
  float height;       /* Height (from launch altitude) in meters */
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

ORB_DECLARE(fusion_height);

#endif /* __ROCKETALT_SENSOR_HEIGHT_H */
