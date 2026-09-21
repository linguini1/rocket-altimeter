#ifndef __ROCKETALT_SENSOR_DEPLOY_EVENT_H
#define __ROCKETALT_SENSOR_DEPLOY_EVENT_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <uORB/uORB.h>

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct deploy_event
{
  uint64_t timestamp; /* Timestamp in microseconds */
  uint8_t id;         /* Deployment channel ID */
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

ORB_DECLARE(deploy_event);

#endif /* __ROCKETALT_SENSOR_DEPLOY_EVENT_H */
