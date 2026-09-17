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
  int baro_fd;
  int c;
  int devno = 0; /* Default of 0 */
  char *interface = NULL;

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
          syslog(LOG_ERR | LOG_USER, "Usage: fake_baro [-n devno]\n");
          return EXIT_FAILURE;
        }
    }

  /* Ensure that after parsing options, we are also given the mandatory curve
   * parameters.
   *
   * TODO: what parameters do I need?
   */

  /* Set up sensor_baro uORB topic */

  baro_fd = orb_advertise_multi_queue(ORB_ID(sensor_baro), NULL, &devno,
                                      CONFIG_ROCKETALT_FAKE_BARO_QLEN);
  if (baro_fd < 0)
    {
      syslog(LOG_ERR | LOG_USER, "Couldn't advertise sensor_baro%d: %d\n",
             devno, errno);
      return EXIT_FAILURE;
    }

  syslog(LOG_INFO | LOG_USER, "sensor_baro%d advertised\n", devno);

  /* Start publishing to the topic from the curve! */

  /* TODO: implementation */

  syslog(LOG_ERR | LOG_USER,
         "Could not start sensor_baro%d using curve: not implemented\n",
         devno);

cleanup_topic:
  orb_unadvertise(baro_fd);

  return EXIT_FAILURE;
}
