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

/* Pressure at sea-level in millibars */

#define SEA_PRESSURE (1013.25f)

/* The universal gas constant */

#define GAS_CONSTANT (8.31432f)

/* Acceleration due to gravity on Earth (m/s^2) */

#define GRAVITY (9.80665f)

/* Constant for the mean molar mass of atmospheric gases */

#define MOLAR_MASS (0.0289644f)

/* Celsius to Kelvin conversion factor */

#define CELSIUS_TO_KELVIN (273.0f)

/* Program already knows about barometer data */

ORB_DECLARE(sensor_baro);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Optional debug output format string */

#ifdef CONFIG_DEBUG_UORB
static const char fusion_altitude_format[] =
    "fusion_altitude - timestamp:%" PRIu64 ",altitude:%hf";

static const char fusion_height_format[] =
    "fusion_height - timestamp:%" PRIu64 ",height:%hf";
#endif

/* Definition for altitude topic */

ORB_DEFINE(fusion_altitude, struct fusion_altitude, fusion_altitude_format);
ORB_DEFINE(fusion_height, struct fusion_height, fusion_height_format);

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: alt_from_baro
 *
 * Input Parameters:
 *   baro - Barometer data to convert into altitude
 *
 * Description:
 *   Converts barometric readings into altitude data.
 *
 * Returned Value:
 *   Calculated altitude measurement.
 ****************************************************************************/

static struct fusion_altitude alt_from_baro(struct sensor_baro *baro)
{
  return (struct fusion_altitude){
      .timestamp = baro->timestamp,
      .altitude = -(GAS_CONSTANT * (CELSIUS_TO_KELVIN + baro->temperature)) /
                  (MOLAR_MASS * GRAVITY) * log(baro->pressure / SEA_PRESSURE),
  };
}

/****************************************************************************
 * Name: height_from_alt
 *
 * Input Parameters:
 *   launch - Altitude measurement taken at the launch altitude
 *   cur - Current altitude measurement
 *
 * Description:
 *   Converts altitude measurements into a height offset.
 *
 * Returned Value:
 *   Current height (relative altitude) measurement.
 ****************************************************************************/

static struct fusion_height height_from_alt(struct fusion_altitude *launch,
                                            struct fusion_altitude *cur)
{
  return (struct fusion_height){
      .timestamp = cur->timestamp,
      .height = (cur->altitude - launch->altitude),
  };
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int main(int argc, char **argv)
{
  int ret;
  int err;
  int alt_fd;
  int height_fd;
  int baro_fd;
  unsigned baro_instance = 0;
  struct sensor_baro baro_data;
  struct fusion_altitude cur_alt;
  struct fusion_altitude launch_alt;
  struct fusion_height height;
  struct pollfd pfd;

  /* Get which barometer instance should be used if one is provided.
   * NOTE: assumes that first argument is always the baro instance to use.
   * TODO: allow other parameters to be chosen?
   */

  if (argc > 1)
    {
      errno = 0;
      baro_instance = strtoul(argv[1], NULL, 10);

      /* There was a conversion error */

      if (errno)
        {
          syslog(LOG_ERR | LOG_USER, "'%s' is not a valid number.\n",
                 argv[1]);
          return EXIT_FAILURE;
        }
    }

  /* Set up altitude fusion topic for publishing */

  alt_fd =
      orb_advertise_multi_queue(ORB_ID(fusion_altitude), NULL, NULL,
                                CONFIG_ROCKETALT_PROCESSING_ALTFUSION_QLEN);
  if (alt_fd < 0)
    {
      syslog(LOG_ERR | LOG_USER,
             "Could not advertise fusion_altitude topic: %d\n", errno);
      return EXIT_FAILURE;
    }

  syslog(LOG_INFO | LOG_USER, "fusion_altitude topic advertised.\n");

  /* Set up height fusion topic for publishing */

  height_fd = orb_advertise_multi_queue(
      ORB_ID(fusion_height), NULL, NULL,
      CONFIG_ROCKETALT_PROCESSING_HEIGHTFUSION_QLEN);

  if (height_fd < 0)
    {
      syslog(LOG_ERR | LOG_USER,
             "Could not advertise fusion_height topic: %d\n", errno);
      ret = EXIT_FAILURE;
      goto cleanup_alt;
    }

  syslog(LOG_INFO | LOG_USER, "fusion_height topic advertised.\n");

  /* Subscribe to barometer topic of correct instance */

  baro_fd = orb_subscribe_multi(ORB_ID(sensor_baro), baro_instance);
  if (baro_fd < 0)
    {
      syslog(LOG_ERR | LOG_USER, "Could not subscribe to sensor_baro%d: %d\n",
             baro_instance, errno);
      ret = EXIT_FAILURE;
      goto cleanup_height;
    }

  /* Set our launch height to zero to indicate that we haven't recorded it
   * yet.
   */

  memset(&launch_alt, 0, sizeof(launch_alt));

  /* Set up polling */

  pfd.fd = baro_fd;
  pfd.events = POLLIN;
  pfd.revents = 0;

  /* Forever convert barometer data to altitude data */

  for (;;)
    {
      err = poll(&pfd, 1, -1);
      if (err <= 0)
        {
          syslog(LOG_ERR | LOG_USER, "Failed to poll barometer: %d\n", errno);
          continue; /* Try again */
        }

      err = orb_copy(ORB_ID(sensor_baro), baro_fd, &baro_data);
      if (err)
        {
          if (errno != ENODATA)
            {
              syslog(LOG_ERR | LOG_USER, "Couldn't get barometer data: %d\n",
                     errno);
            }
          continue;
        }

      cur_alt = alt_from_baro(&baro_data);

      /* If we haven't recorded launch height yet, record it now */

      if (launch_alt.timestamp == 0)
        {
          launch_alt = cur_alt;
        }

      /* Publish raw altitude data */

      err = orb_publish(ORB_ID(fusion_altitude), alt_fd, &cur_alt);
      if (err)
        {
          syslog(LOG_ERR | LOG_USER, "Couldn't publish altitude data: %d\n",
                 errno);
          continue;
        }

      /* Publish our computed height */

      height = height_from_alt(&launch_alt, &cur_alt);

      orb_publish(ORB_ID(fusion_height), height_fd, &height);
      if (err)
        {
          syslog(LOG_ERR | LOG_USER, "Couldn't publish height data: %d\n",
                 errno);
          continue;
        }
    }

  orb_unsubscribe(baro_fd);
cleanup_height:
  orb_unadvertise(height_fd);
cleanup_alt:
  orb_unadvertise(alt_fd);
  return ret;
}
