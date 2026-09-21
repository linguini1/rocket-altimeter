#ifndef __ROCKETALT_SENSOR_FLIGHT_EVENT_H
#define __ROCKETALT_SENSOR_FLIGHT_EVENT_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <uORB/uORB.h>

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct flight_event
{
  uint64_t timestamp; /* Timestamp in microseconds */
  uint8_t event;      /* Flight event code */
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

ORB_DECLARE(flight_event);

#endif /* __ROCKETALT_SENSOR_FLIGHT_EVENT_H */
