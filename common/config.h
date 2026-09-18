#ifndef _ROCKETALT_CONFIG_H
#define _ROCKETALT_CONFIG_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

#include "common.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define CONF_MAX_DEPCHANS (CONFIG_ROCKETALT_CONF_MAXDEPCHANS)

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct chan_config_s
{
  int conditions; /* Deployment conditions */
  float altitude; /* Deployment altitude in meters */
  uint16_t time;  /* Deployment time in seconds */
};

struct config_s
{
  struct chan_config_s chans[CONF_MAX_DEPCHANS]; /* Channel configurations */
  float apogee;   /* Predicted apogee in meters */
  uint8_t nchans; /* Number of channels used */
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#endif /* _ROCKETALT_CONFIG_H */
