#ifndef _ROCKETALT_COMMON_H
#define _ROCKETALT_COMMON_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>
#include <stdint.h>

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

struct sensor_voltage
{
  uint64_t timestamp; /* Timestamp in microseconds */
  float voltage;      /* Voltage (Volts) */
};

enum fevent_e
{
  FEVENT_GROUNDED = 0, /* Rocket is waiting for liftoff */
  FEVENT_ASCENT = 1,   /* Rocket is ascending */
  FEVENT_APOGEE = 2,   /* Rocket has reached apogee */
  FEVENT_DESCENT = 3,  /* Rocket is descending */
  FEVENT_LANDED = 4,   /* Rocket has landed */
};

struct flight_event
{
  uint64_t timestamp;  /* Timestamp in microseconds */
  enum fevent_e event; /* Flight event */
};

enum devent_e
{
  DEVENT_MAIN = 0,   /* Rocket is waiting for liftoff */
  DEVENT_DROGUE = 1, /* Rocket is ascending */
};

struct deploy_event
{
  uint64_t timestamp;  /* Timestamp in microseconds */
  enum devent_e event; /* Deployment event */
};

/* Deployment configuration options */

struct depconfig_s
{
  float main_alt;       /* Altitude for main deployment (m) */
  float drogue_alt;     /* Altitude for drogue deployment (m) */
  uint16_t main_time;   /* Time to deploy main (s) */
  uint16_t drogue_time; /* Time to deploy drogue (s) */
  bool drogue_apogee;   /* True: deploy drogue at apogee, false: altitude */
};

/* Processing configuration */

struct processconfig_s
{
  float pred_apogee; /* Predicted apogee (m) */
};

/* Altimeter configuration options */

struct altconfig_s
{
  struct depconfig_s dep;       /* Deployment config */
  struct processconfig_s event; /* Processing config */
};

#endif // _ROCKETALT_COMMON_H
