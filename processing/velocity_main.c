/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <fcntl.h>
#include <math.h>
#include <poll.h>
#include <stdint.h>
#include <stdlib.h>
#include <sys/types.h>
#include <syslog.h>

#include <uORB/uORB.h>

#include "../common/common.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Low-pass filter alpha for computing filtered velocity */

#define LP_ALPHA (0.98)

/* Microsecond to second conversion factor */

#define US_TO_S (1e-06)

/* Program already knows about altitude data */

ORB_DECLARE(fusion_altitude);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Optional debug output format string */

#ifdef CONFIG_DEBUG_UORB
static const char sensor_velocity_format[] =
    "sensor_velocity - timestamp:%" PRIu64 ",velocity:%hf";
#endif

/* Definition for altitude topic */

ORB_DEFINE(sensor_velocity, struct sensor_velocity, sensor_velocity_format);

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static struct sensor_velocity vel_from_alt(struct fusion_altitude *prev,
                                           struct fusion_altitude *cur)
{
  return (struct sensor_velocity){
      .timestamp = prev->timestamp,
      .velocity = (cur->altitude - prev->altitude) /
                  ((float)(cur->timestamp - prev->timestamp) * US_TO_S),
  };
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int main(int argc, char **argv)
{
  int err;
  int vel_fd;
  int alt_fd;
  float prev_vel = 0.0f;
  struct sensor_velocity vel_data;
  struct fusion_altitude prev_alt;
  struct fusion_altitude cur_alt;
  struct pollfd pfd;

  /* Set up velocity fusion topic for publishing */

  vel_fd =
      orb_advertise_multi_queue(ORB_ID(sensor_velocity), NULL, NULL,
                                CONFIG_ROCKETALT_PROCESSING_VELFUSION_QLEN);
  if (vel_fd < 0)
    {
      syslog(LOG_ERR | LOG_USER,
             "Could not advertise sensor_velocity topic: %d\n", errno);
      return EXIT_FAILURE;
    }

  syslog(LOG_INFO | LOG_USER, "sensor_velocity topic advertised.\n");

  /* Subscribe to first altitude topic */

  alt_fd = orb_subscribe_multi(ORB_ID(fusion_altitude), 0);
  if (alt_fd < 0)
    {
      syslog(LOG_ERR | LOG_USER,
             "Could not subscribe to fusion_altitude0: %d\n", errno);
      orb_unadvertise(vel_fd);
      return EXIT_FAILURE;
    }

  /* Set up polling */

  pfd.fd = alt_fd;
  pfd.events = POLLIN;
  pfd.revents = 0;

  /* Forever convert altitude data to velocity data */

  for (;;)
    {
      err = poll(&pfd, 1, -1);
      if (err <= 0)
        {
          syslog(LOG_ERR | LOG_USER, "Failed to poll barometer: %d\n", errno);
          continue; /* Try again */
        }

      err = orb_copy(ORB_ID(fusion_altitude), alt_fd, &cur_alt);
      if (err)
        {
          if (errno != ENODATA)
            {
              syslog(LOG_ERR | LOG_USER, "Couldn't get altitude data: %d\n",
                     errno);
            }
          continue;
        }

      /* Calculate raw velocity */

      vel_data = vel_from_alt(&prev_alt, &cur_alt);
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
  orb_unsubscribe(alt_fd);
  return EXIT_FAILURE;
}
