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

/* Topic indexes */

#define ALT_IDX (0)
#define HEIGHT_IDX (1)

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct topic_s
{
  int fd;                          /* File descriptor to publish to */
  int qlen;                        /* Length of topic queue */
  int devno;                       /* Device number of topic instance */
  const struct orb_metadata *meta; /* Topic metadata */
};

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

/* uORB topics we publish */

static struct topic_s g_topics[] = {
    [ALT_IDX] =
        {
            .fd = -1,
            .meta = ORB_ID(fusion_altitude),
            .qlen = CONFIG_ROCKETALT_ALTFUSION_QLEN,
        },
    [HEIGHT_IDX] =
        {
            .fd = -1,
            .meta = ORB_ID(fusion_height),
            .qlen = CONFIG_ROCKETALT_HEIGHTFUSION_QLEN,
        },
};

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
      .height = cur->altitude - launch->altitude,
  };
}

static int publish_to_topic(const struct topic_s *topic, void *data)
{
  int err;

  err = orb_publish(topic->meta, topic->fd, data);
  if (err)
    {
      syslog(LOG_ERR | LOG_USER, "Couldn't publish to %s%d: %d\n",
             topic->meta->o_name, topic->devno, errno);
    }

  return err;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int main(int argc, char **argv)
{
  int ret;
  int err;
  int c;
  int devno = 0;
  int baro_instance = 0;
  struct sensor_baro baro_data;
  struct fusion_altitude cur_alt;
  struct fusion_altitude launch_alt;
  struct fusion_height height;
  struct pollfd pfd;

  while ((c = getopt(argc, argv, ":n:b:")) != -1)
    {
      switch (c)
        {
        case 'n':
          devno = atoi(optarg);
          break;

        case 'b':
          baro_instance = atoi(optarg);
          break;

        case ':':
          syslog(LOG_ERR | LOG_USER, "Option -%c requires an argument.\n",
                 optopt);
          return EXIT_FAILURE;

        case '?':
          syslog(LOG_ERR | LOG_USER, "Unknown option '-%c'.\n", optopt);
          break; /* Don't exit, parse other options */

        default:
          syslog(LOG_ERR | LOG_USER, "Usage: altitude_fusion [-n devno]\n");
          return EXIT_FAILURE;
        }
    }

  /* Set up topics for publishing */

  for (int i = 0; i < array_len(g_topics); i++)
    {
      g_topics[i].devno = devno;
      g_topics[i].fd = orb_advertise_multi_queue(g_topics[i].meta, NULL,
                                                 &devno, g_topics[i].qlen);
      if (g_topics[i].fd < 0)
        {
          syslog(LOG_ERR | LOG_USER, "Could not advertise %s%d: %d\n",
                 g_topics[i].meta->o_name, g_topics[i].devno, errno);
          ret = EXIT_FAILURE;
          goto cleanup_topics;
        }

      syslog(LOG_INFO | LOG_USER, "%s%d advertised\n",
             g_topics[i].meta->o_name, g_topics[i].devno);
    }

  /* Subscribe to barometer topic of correct instance */

  pfd.fd = orb_subscribe_multi(ORB_ID(sensor_baro), baro_instance);
  if (pfd.fd < 0)
    {
      syslog(LOG_ERR | LOG_USER, "Could not subscribe to sensor_baro%d: %d\n",
             baro_instance, errno);
      ret = EXIT_FAILURE;
      goto cleanup_topics;
    }

  /* Set our launch height to zero to indicate that we haven't recorded it
   * yet.
   */

  memset(&launch_alt, 0, sizeof(launch_alt));

  /* Set up polling */

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

      err = orb_copy(ORB_ID(sensor_baro), pfd.fd, &baro_data);
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

      publish_to_topic(&g_topics[ALT_IDX], &cur_alt);

      /* Publish our computed height */

      height = height_from_alt(&launch_alt, &cur_alt);
      publish_to_topic(&g_topics[HEIGHT_IDX], &height);
    }

cleanup_topics:
  for (int i = 0; i < array_len(g_topics); i++)
    {
      if (g_topics[i].fd < 0)
        {
          orb_unadvertise(g_topics[i].fd);
        }
    }

  orb_unsubscribe(pfd.fd);
  return ret;
}
