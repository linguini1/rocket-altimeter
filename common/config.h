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

#define rocketalt_config_apogee(conf) ((conf)->apogee)

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

/****************************************************************************
 * Name: rocketalt_config_from_file
 *
 * Description:
 *   Read in the contents of the file and return a configuration structure.
 *
 * Input Parameters:
 *   path - The file to read the configuration from
 *   config - The config struct to populate with file data
 *
 * Returned Value:
 *   0 on success, an error code on failure.
 *
 ****************************************************************************/

int rocketalt_config_from_file(const char *path, struct config_s *config);

/****************************************************************************
 * Name: rocketalt_config_getchan
 *
 * Description:
 *   Get a channel by its ID.
 *
 * Input Parameters:
 *   config - The configuration to get the channel from
 *   chanid - The channel ID of the channel to get
 *
 * Returned Value:
 *   A pointer to the channel on success, NULL on failure.
 *
 ****************************************************************************/

const struct chan_config_s *
rocketalt_config_getchan(const struct config_s *config, uint8_t chanid);

#endif /* _ROCKETALT_CONFIG_H */
