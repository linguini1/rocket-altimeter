/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <fcntl.h>
#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include <sys/types.h>
#include <syslog.h>

#include <nuttx/sensors/fakesensor.h>
#include <nuttx/sensors/sensor.h>

#include <uORB/uORB.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Program already knows about barometer data */

ORB_DECLARE(sensor_baro);

static_assert(CONFIG_ROCKETALT_FAKE_BARO_USECSV,
              "Only the CSV data method is currently supported.");

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int main(int argc, char **argv)
{
  int err;

  err = fakesensor_init(SENSOR_TYPE_BAROMETER,
                        CONFIG_ROCKETALT_FAKE_BARO_CSVPATH, 0,
                        CONFIG_ROCKETALT_FAKE_BARO_QLEN);
  if (err < 0)
    {
      syslog(LOG_ERR | LOG_USER, "Could not register fake sensor_baro0: %d\n",
             err);
      return EXIT_FAILURE;
    }

  syslog(LOG_INFO | LOG_USER, "Registered fake sensor_baro0 using CSV "
                              "'" CONFIG_ROCKETALT_FAKE_BARO_CSVPATH "'\n");
  return EXIT_SUCCESS;
}
