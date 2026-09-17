/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <fcntl.h>
#include <getopt.h>
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
  int c;
  int devno = 0; /* Default of 0 */
  char *csvpath = NULL;

  /* Parse command line arguments */

  while ((c = getopt(argc, argv, ":n:")) != -1)
    {
      switch (c)
        {
        case 'n':
          devno = atoi(optarg);
          break;

        case ':':
          syslog(LOG_ERR | LOG_USER, "Option -%c requires an argument.\n",
                 optopt);
          return EXIT_FAILURE;

        case '?':
          syslog(LOG_ERR | LOG_USER, "Unknown option '-%c'.\n", optopt);
          break; /* Don't exit, parse other options */

        default:
          syslog(LOG_ERR | LOG_USER, "Usage: fake_baro [-n devno] csvpath\n");
          return EXIT_FAILURE;
        }
    }

  /* Ensure that after parsing options, we are also given a CSV path */

  if (argc < optind)
    {
      syslog(LOG_ERR | LOG_USER, "fake_baro expected CSV path.\n");
      return EXIT_FAILURE;
    }

  csvpath = argv[optind];

  /* Assume the first argument is a path to a CSV file containing barometer
   * data and try to run the fake sensor.
   */

  err = fakesensor_init(SENSOR_TYPE_BAROMETER, csvpath, devno,
                        CONFIG_ROCKETALT_FAKE_BARO_QLEN);
  if (err)
    {
      syslog(LOG_ERR | LOG_USER,
             "Could not register fake sensor_baro%d: %d\n", devno, err);
      return EXIT_FAILURE;
    }

  syslog(LOG_INFO | LOG_USER, "Registered fake sensor_baro%d from %s\n",
         devno, argv[1]);

  return EXIT_SUCCESS;
}
