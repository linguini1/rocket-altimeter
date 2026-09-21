/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <errno.h>
#include <fcntl.h>
#include <getopt.h>
#include <math.h>
#include <poll.h>
#include <stdint.h>
#include <stdlib.h>
#include <sys/types.h>
#include <syslog.h>

#include <uORB/uORB.h>

#include "sensor/altitude.h"
#include <sensor/velocity.h>

#include "../../common/common.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Low-pass filter alpha for computing filtered velocity */

#define LP_ALPHA (0.98)

/* Microsecond to second conversion factor */

#define US_TO_S (1e-06)

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
 * Name: vel_from_alt
 *
 * Description:
 *   Calculate velocity discretely using two altitude measurements.
 *
 * Input Parameters:
 *   prev - The altitude measurement from the previous time step
 *   cur - The current altitude measurement
 *   res - Where to store the resulting velocity measurement
 *
 ****************************************************************************/

static void calculate_vel(struct fusion_altitude *prev,
                          struct fusion_altitude *cur,
                          struct sensor_velocity *res)
{
  res->timestamp = cur->timestamp;
  res->velocity = (cur->altitude - prev->altitude) /
                  ((float)(cur->timestamp - prev->timestamp) * US_TO_S);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int main(int argc, char **argv)
{
  int c;
  int err;
  int vel_fd;
  int devno = 0;
  int alt_devno = 0;
  float prev_vel = 0.0f;
  struct sensor_velocity vel_data;
  struct fusion_altitude prev_alt;
  struct fusion_altitude cur_alt;
  struct pollfd pfd;

  while ((c = getopt(argc, argv, ":n:a:")) != -1)
    {
      switch (c)
        {
        case 'n':
          devno = atoi(optarg);
          break;

        case 'a':
          alt_devno = atoi(optarg);
          break;

        case ':':
          syslog(LOG_ERR | LOG_USER, "Option -%c requires an argument.\n",
                 optopt);
          return EXIT_FAILURE;

        case '?':
          syslog(LOG_ERR | LOG_USER, "Unknown option '-%c'.\n", optopt);
          break; /* Don't exit, parse other options */

        default:
          syslog(LOG_ERR | LOG_USER, "Usage: velocity_fusion [-n devno]\n");
          return EXIT_FAILURE;
        }
    }

  /* Set up velocity fusion topic for publishing */

  vel_fd = orb_advertise_multi_queue(ORB_ID(sensor_velocity), NULL, &devno,
                                     CONFIG_ROCKETALT_VELFUSION_QLEN);
  if (vel_fd < 0)
    {
      syslog(LOG_ERR | LOG_USER,
             "Could not advertise sensor_velocity%d topic: %d\n", devno,
             errno);
      return EXIT_FAILURE;
    }

  syslog(LOG_INFO | LOG_USER, "sensor_velocity%d advertised.\n", devno);

  /* Subscribe to altitude topic */

  pfd.fd = orb_subscribe_multi(ORB_ID(fusion_altitude), alt_devno);
  if (pfd.fd < 0)
    {
      syslog(LOG_ERR | LOG_USER,
             "Could not subscribe to fusion_altitude%d: %d\n", alt_devno,
             errno);
      orb_unadvertise(vel_fd);
      return EXIT_FAILURE;
    }

  /* Set up polling */

  pfd.events = POLLIN;
  pfd.revents = 0;

  /* Our previous altitude measurement for the first iteration is marked with
   * a 0 timestamp. This tells us to copy the current measurement and skip an
   * iteration.
   */

  prev_alt.timestamp = 0;

  /* Forever convert altitude data to velocity data */

  for (;;)
    {
      err = poll(&pfd, 1, -1);
      if (err <= 0)
        {
          syslog(LOG_ERR | LOG_USER, "Failed to poll barometer: %d\n", errno);
          continue; /* Try again */
        }

      err = orb_copy(ORB_ID(fusion_altitude), pfd.fd, &cur_alt);
      if (err)
        {
          if (errno != ENODATA)
            {
              syslog(LOG_ERR | LOG_USER, "Couldn't get altitude data: %d\n",
                     errno);
            }
          continue;
        }

      if (prev_alt.timestamp == 0)
        {
          prev_alt = cur_alt;
          continue; /* Skip this iteration until we get 1 more measurement */
        }

      /* Calculate raw velocity */

      calculate_vel(&prev_alt, &cur_alt, &vel_data);
      prev_alt = cur_alt;

      /* Low pass filtering */

      vel_data.velocity = lp_filter(prev_vel, vel_data.velocity, LP_ALPHA);
      prev_vel = vel_data.velocity;

      err = orb_publish(ORB_ID(sensor_velocity), vel_fd, &vel_data);
      if (err)
        {
          syslog(LOG_ERR | LOG_USER, "Couldn't publish velocity data: %d\n",
                 errno);
          continue;
        }
    }

  orb_unadvertise(vel_fd);
  orb_unsubscribe(pfd.fd);
  return EXIT_FAILURE;
}
