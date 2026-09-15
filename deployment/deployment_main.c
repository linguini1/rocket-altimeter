/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <fcntl.h>
#include <math.h>
#include <poll.h>
#include <stdint.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <syslog.h>

#include <nuttx/ioexpander/gpio.h>
#include <nuttx/sensors/fakesensor.h>
#include <nuttx/sensors/sensor.h>

#include <uORB/uORB.h>

#include "../common/common.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Array length helper */

#define array_len(arr) (sizeof(arr) / sizeof(arr[0]))

/* Indices into arrays needed for height and velocity data */

#define HEIGHT_IDX (0)
#define EVENT_IDX (1)

/* Program already knows flight events and height */

ORB_DECLARE(fusion_height);
ORB_DECLARE(flight_event);

/****************************************************************************
 * Private Types
 ****************************************************************************/

union sensor_data
{
  struct fusion_height height;
  struct flight_event event;
};

/* Conditions for deployment */

enum depcond_e
{
  COND_APOGEE = 0x1, /* Deploy at apogee */
  COND_ALT = 0x2,    /* Deploy at configured altitude, after apogee */
  COND_TIME = 0x4,   /* Deploy using timer, after ascent */
};

/* Represents a pyro channel */

struct pyrochan_s
{
  char *path;     /* Channel GPIO path */
  int fd;         /* File descriptor to the channel GPIO */
  int cond;       /* Deployment condition bitmask */
  float altitude; /* Altitude in meters */
  uint16_t time;  /* Deployment time in seconds (for timer) */
  uint8_t id;     /* Channel ID */
  bool fired;     /* Whether or not this channel has been deployed */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Deployment channels */

static struct pyrochan_s g_channels[CONFIG_ROCKETALT_DEPLOYMENT_NUMCHANS];

/* Optional debug output format string */

#ifdef CONFIG_DEBUG_UORB
static const char deploy_event_format[] =
    "deploy_event - timestamp:%" PRIu64 ",event:%u";
#endif

/* Definition for deployment event topic */

ORB_DEFINE(deploy_event, struct deploy_event, deploy_event_format);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: channel_deinit
 *
 * Description:
 *   De-initializes a pyro channel.
 *
 * Input Parameters:
 *   chan - The channel to de-initialize.
 *
 * Returned Value:
 *   0 on success, negated errno on failure.
 *
 ****************************************************************************/

static int channel_deinit(struct pyrochan_s *chan)
{
  int err = 0;

  if (chan->fd > 0)
    {
      err = close(chan->fd);
      chan->fd = -1;
    }

  return err;
}

/****************************************************************************
 * Name: fire_channel
 *
 * Description:
 *   Fires a deployment channel.
 *
 * Input Parameters:
 *   chan - Deployment channel to fire.
 *
 * Returned Value:
 *   0 on success, error code on failure.
 *
 ****************************************************************************/

static int channel_fire(struct pyrochan_s *chan)
{
#ifndef CONFIG_ROCKETALT_DEPLOYMENT_MOCK
  int err;
  bool fire = true;

  err = ioctl(chan->fd, GPIOC_WRITE, &fire);
  if (err < 0)
    {
      syslog(LOG_ERR | LOG_USER, "Couldn't fire channel %d: %d\n", chan->id,
             errno);
      return errno;
    }

    /* Channel fired successfully. TODO: turn it off again! */
#endif

  syslog(LOG_INFO | LOG_USER, "Fired channel %d!\n", chan->id);
  chan->fired = true;
  return 0;
}

/****************************************************************************
 * Name: publish_deployment
 *
 * Description:
 *   Publishes a deployment event.
 *
 * Input Parameters:
 *   fd - The file descriptor of the topic to publish the event to.
 *   id - The ID of the channel that was deployed.
 *
 * Returned Value:
 *   0 on success, error code on failure.
 *
 ****************************************************************************/

static int publish_deployment(int fd, uint8_t id)
{
  int err;
  struct deploy_event event = {.timestamp = orb_absolute_time(), .id = id};

  err = orb_publish(ORB_ID(deploy_event), fd, &event);
  if (err)
    {
      syslog(LOG_ERR | LOG_USER, "Couldn't publish deployment: %d\n", errno);
    }

  return err;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int main(int argc, char **argv)
{
  int err;
  int ret;
  int dep_fd;
  struct pollfd fds[2];
  union sensor_data data[2];
  bool drogue_deployed = false;
  bool main_deployed = false;

  /* Initialize deployment channels */

  for (int i = 0; i < array_len(g_channels); i++)
    {
      g_channels[i].fd = -1;
      g_channels[i].id = i + 1;
      g_channels[i].path = NULL;
      g_channels[i].deployed = false;
    }

  /* Configure deployment channels TODO: do this dynamically */

  g_channels[0].cond = COND_APOGEE;
  g_channels[1].cond = COND_ALT;
  g_channels[1].arg.altitude = 305.0f; /* 1000 ft */

  /* Set up deployment channel file descriptors */

  for (int i = 0; i < array_len(g_channels); i++)
    {
      /* Not opening a deployment channel should be considered fatal */

      err = open(g_channels[i].path, O_RDWR);
      if (err < 0)
        {
          syslog(LOG_ERR | LOG_USER, "Couldn't open %s: %d\n",
                 g_channels[i].path, errno);
          ret = EXIT_FAILURE;
          goto clean_channels;
        }

      g_channels[i].fd = err; /* Store the opened fd */
    }

  /* Set up flight event topic for publishing */

  dep_fd = orb_advertise_multi_queue(ORB_ID(deploy_event), NULL, NULL,
                                     CONFIG_ROCKETALT_DEPLOYMENT_QLEN);
  if (dep_fd < 0)
    {
      syslog(LOG_ERR | LOG_USER,
             "Could not advertise deploy_event topic: %d\n", errno);
      ret = EXIT_FAILURE;
      goto clean_channels;
    }

  syslog(LOG_INFO | LOG_USER, "deploy_event topic advertised.\n");

  /* Subscribe to height topic */

  fds[HEIGHT_IDX].fd = orb_subscribe_multi(ORB_ID(fusion_height), 0);
  if (fds[HEIGHT_IDX].fd < 0)
    {
      syslog(LOG_ERR | LOG_USER,
             "Could not subscribe to fusion_height0: %d\n", errno);
      ret = EXIT_FAILURE;
      goto clean_dep;
    }

  /* Subscribe to flight event topic */

  fds[EVENT_IDX].fd = orb_subscribe_multi(ORB_ID(flight_event), 0);
  if (fds[EVENT_IDX].fd < 0)
    {
      syslog(LOG_ERR | LOG_USER, "Could not subscribe to flight_event0: %d\n",
             errno);
      ret = EXIT_FAILURE;
      goto clean_height;
    }

  /* Set up for polling */

  fds[EVENT_IDX].events = POLLIN;
  fds[EVENT_IDX].revents = 0;
  fds[HEIGHT_IDX].events = POLLIN;
  fds[HEIGHT_IDX].revents = 0;

  /* Assume start on the ground to avoid bad deployments */

  data[EVENT_IDX].event.event = FEVENT_GROUNDED;

  /* Handle events and make deployment decisions */

  for (;;)
    {
      /* Poll to read some height and event data */

      err = poll(fds, array_len(fds), -1);
      if (err <= 0)
        {
          syslog(LOG_ERR | LOG_USER, "Failed to poll: %d\n", errno);
          continue; /* Try again */
        }

      /* We have some data, read it in! */

      for (unsigned i = 0; i < array_len(fds); i++)
        {
          if (fds[i].revents & POLLIN)
            {
              orb_copy_multi(fds[i].fd, &data[i], sizeof(union sensor_data));
              fds[i].revents = 0; /* Clear events */
            }
        }

      /* If we are on the ground or landed, we do not deploy under any
       * circumstance.
       */

      if (data[EVENT_IDX].event.event == FEVENT_GROUNDED ||
          data[EVENT_IDX].event.event == FEVENT_LANDED)
        {
          continue;
        }

      /* If we have detected ascent, we have just lifted off. Start the
       * backup timers if they're configured.
       */

      if (data[EVENT_IDX].event.event == FEVENT_ASCENT)
        {
          for (int i = 0; i < array_len(g_channels); i++)
            {
              if (g_channels[i].cond & COND_TIME)
                {
                  /* TODO: set up a timer for this channel's deployment */
                }
            }
        }

      /* If we have detected apogee, fire any channel which is meant to fire
       * at apogee.
       */

      if (data[EVENT_IDX].event.event == FEVENT_APOGEE)
        {
          for (int i = 0; i < array_len(g_channels); i++)
            {
              if (g_channels[i].cond & COND_APOGEE)
                {
                  err = channel_fire(&g_channels[i]);
                  if (err == 0)
                    {
                      publish_deployment(dep_fd, g_channels[i].id);
                    }
                }
            }
        }

      /* If we are past apogee, we can deploy any channels that have an
       * altitude limit above our current altitude.
       *
       * Only do this if we have detected apogee or descent. We don't want to
       * deploy if a pressure spike (like Mach dip) causes height to increase
       * beyond our configured deployment altitude.
       *
       * NOTE: It is permissible to check >= FEVENT_APOGEE because we would
       * have already skipped past FEVENT_LANDED earlier in the loop, so this
       * effectively checks for apogee or ascent.
       */

      if (data[EVENT_IDX].event.event >= FEVENT_APOGEE)
        {
          for (int i = 0; i < array_len(g_channels); i++)
            {
              if (g_channels[i].cond & COND_ALT &&
                  data[HEIGHT_IDX].height.height <= g_channels[i].altitude)
                {
                  err = channel_fire(&g_channels[i]);
                  if (err == 0)
                    {
                      publish_deployment(dep_fd, g_channels[i].id);
                    }
                }
            }
        }
    }

  orb_unsubscribe(fds[EVENT_IDX].fd);

clean_height:
  orb_unsubscribe(fds[HEIGHT_IDX].fd);

clean_dep:
  orb_unadvertise(dep_fd);

clean_channels:
  for (int i = 0; i < array_len(g_channels); i++)
    {
      channel_deinit(&g_channels[i]);
    }

  return ret;
}
