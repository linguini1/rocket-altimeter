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

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct depconfig_s dummy_conf = {
    .main_alt = 4000.0f,
    .drogue_alt = 8000.0f,
    .main_time = 18,
    .drogue_time = 26,
    .drogue_apogee = true,
};

/* Optional debug output format string */

#ifdef CONFIG_DEBUG_UORB
static const char deploy_event_format[] =
    "deploy_event - timestamp:%" PRIu64 ",event:%u";
#endif

/* Definition for deployment event topic */

ORB_DEFINE(deploy_event, struct deploy_event, deploy_event_format);

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int deploy_main(void)
{
#ifdef CONFIG_ROCKETALT_DEPLOYMENT_MOCK
  syslog(LOG_INFO | LOG_USER, "Main deployed!\n");
  return 0;
#else
  return 0; // TODO: gpio
#endif /* CONFIG_ROCKETALT_DEPLOYMENT_MOCK */
}

static int deploy_drogue(void)
{
#ifdef CONFIG_ROCKETALT_DEPLOYMENT_MOCK
  syslog(LOG_INFO | LOG_USER, "Drogue deployed!\n");
  return 0;
#else
  return 0; // TODO: gpio
#endif /* CONFIG_ROCKETALT_DEPLOYMENT_MOCK */
}

static int publish_deployment(int fd, enum devent_e etype)
{
  int err;
  struct deploy_event event = {
      .event = etype,
      .timestamp = orb_absolute_time(),
  };

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

  /* Set up flight event topic for publishing */

  dep_fd = orb_advertise_multi_queue(ORB_ID(deploy_event), NULL, NULL,
                                     CONFIG_ROCKETALT_DEPLOYMENT_QLEN);
  if (dep_fd < 0)
    {
      syslog(LOG_ERR | LOG_USER,
             "Could not advertise deploy_event topic: %d\n", errno);
      ret = EXIT_FAILURE;
      return ret;
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
          if (dummy_conf.drogue_time != 0)
            {
              /* TODO: set up deployment timer(s) */
            }

          if (dummy_conf.main_time != 0)
            {
              /* TODO: set up deployment timer(s) */
            }
        }

      /* If we have passed the altitude at which we should deploy main/drogue
       * and the chute is configured for an altitude based deployment, deploy
       * it.
       *
       * Only do this if we have detected apogee or descent. We don't want to
       * deploy if a pressure spike (like Mach dip) causes height to increase
       * beyond our configured deployment altitude.
       *
       * NOTE: It is permissible to check >= FEVENT_APOGEE because we would
       * have already skipped past FEVENT_LANDED earlier in the loop, so this
       * effectively checks for apogee or ascent.
       */

      if (!main_deployed &&
          data[HEIGHT_IDX].height.height <= dummy_conf.main_alt &&
          data[EVENT_IDX].event.event >= FEVENT_APOGEE)
        {
          err = deploy_main();
          if (err)
            {
              syslog(LOG_ERR | LOG_USER, "Couldn't deploy main: %d\n", err);
            }
          else
            {
              main_deployed = true;
              publish_deployment(dep_fd, DEVENT_MAIN);
#ifdef CONFIG_ROCKETALT_DEPLOYMENT_MOCK
              syslog(LOG_INFO | LOG_USER, "Deployed main at %.2f m\n",
                     data[HEIGHT_IDX].height.height);
#endif
            }
        }

      if (dummy_conf.drogue_apogee)
        {
          /* If we have detected apogee, and the drogue is configured to
           * deploy at apogee, deploy it.
           */

          if (!drogue_deployed &&
              data[EVENT_IDX].event.event == FEVENT_APOGEE)
            {
              err = deploy_drogue();
              if (err)
                {
                  syslog(LOG_ERR | LOG_USER, "Couldn't deploy drogue: %d\n",
                         err);
                }
              else
                {
                  drogue_deployed = true;
                  publish_deployment(dep_fd, DEVENT_DROGUE);
#ifdef CONFIG_ROCKETALT_DEPLOYMENT_MOCK
                  syslog(LOG_INFO | LOG_USER, "Deployed drogue at %.2f m\n",
                         data[HEIGHT_IDX].height.height);
#endif
                }
            }
        }
      else
        {
          /* Drogue gets deployed at specific altitude */

          if (!drogue_deployed &&
              data[HEIGHT_IDX].height.height <= dummy_conf.drogue_alt &&
              data[EVENT_IDX].event.event >= FEVENT_APOGEE)
            {
              err = deploy_drogue();
              if (err)
                {
                  syslog(LOG_ERR | LOG_USER, "Couldn't deploy drogue: %d\n",
                         err);
                }
              else
                {
                  drogue_deployed = true;
                  publish_deployment(dep_fd, DEVENT_DROGUE);
#ifdef CONFIG_ROCKETALT_DEPLOYMENT_MOCK
                  syslog(LOG_INFO | LOG_USER, "Deployed drogue at %.2f m\n",
                         data[HEIGHT_IDX].height.height);
#endif
                }
            }
        }
    }

  orb_unsubscribe(fds[EVENT_IDX].fd);
clean_height:
  orb_unsubscribe(fds[HEIGHT_IDX].fd);
clean_dep:
  orb_unadvertise(dep_fd);
  return ret;
}
