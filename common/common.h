#ifndef _ROCKETALT_COMMON_H
#define _ROCKETALT_COMMON_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <uORB/uORB.h>

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct fusion_altitude
{
  uint64_t timestamp; /* Timestamp in microseconds */
  float altitude;     /* Altitude in meters */
};

struct fusion_height
{
  uint64_t timestamp; /* Timestamp in microseconds */
  float height;       /* Height (from launch altitude) in meters */
};

struct fusion_velocity
{
  uint64_t timestamp; /* Timestamp in microseconds */
  float velocity;     /* Velocity in m/s */
};

enum event_e
{
  FEVENT_GROUNDED = 0, /* Rocket is waiting for liftoff */
  FEVENT_ASCENT = 1,   /* Rocket is ascending */
  FEVENT_APOGEE = 2,   /* Rocket has reached apogee */
  FEVENT_DESCENT = 3,  /* Rocket is descending */
  FEVENT_LANDED = 4,   /* Rocket has landed */
};

struct flight_event
{
  uint64_t timestamp; /* Timestamp in microseconds */
  enum event_e event; /* Flight event */
};

#endif // _ROCKETALT_COMMON_H
