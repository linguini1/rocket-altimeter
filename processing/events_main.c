/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <errno.h>
#include <fcntl.h>
#include <math.h>
#include <poll.h>
#include <stdint.h>
#include <stdlib.h>
#include <sys/types.h>
#include <syslog.h>

#include <nuttx/circbuf.h>

#include <uORB/uORB.h>

#include "../common/common.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Zero velocity detection in m/s */

#define ZERO_VEL_TOL (0.8f)

#define is_zero(vel, tol) ((vel) <= (tol) && (vel) >= -(tol))

/* Descent velocity bound.
 * Typical descent velocity is around 25ft/s, so anything faster
 * than 12ft/s should be good. That's ~3.6m/s.
 */

#define MIN_DESCENT_VEL (-3.6f)

/* Minimum take-off velocity
 * 12m/s -> ~40kmph which should be hard to achieve accidentally
 */

#define MIN_TAKEOFF_VEL (12.0f)

/* Minimum take-off altitude
 * Allow 1s latency in detecting take-off by taking `MIN_TAKEOFF_VEL * 1` to
 * be our minimum altitude increase at take-off. This is acceptable since no
 * deployment logic (should) happen during the first second of ascent.
 */

#define MIN_TAKEOFF_ALT MIN_TAKEOFF_VEL

/* Number of velocity measurements to average */

#define NUMVEL_SAMPLES (10)

/* Percentage of apogee reached to detect apogee */

#define APOGEE_PERCENTAGE (0.8f)

/* Indices into arrays needed for height and velocity data */

#define HEIGHT_IDX (0)
#define VEL_IDX (1)

/* Program already knows about some topics */

ORB_DECLARE(fusion_height);
ORB_DECLARE(sensor_velocity);

/****************************************************************************
 * Private Types
 ****************************************************************************/

union sensor_data
{
  struct fusion_height height;
  struct sensor_velocity vel;
};

struct topic_s
{
  int devno;
  struct orb_metadata *meta;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct processconfig_s dummy_config = {
    .pred_apogee = 10000.0f, /* Matches default of fake barometer curve */
};

/* Optional debug output format string */

#ifdef CONFIG_DEBUG_UORB
static const char flight_event_format[] =
    "flight_event - timestamp:%" PRIu64 ",event:%u";
#endif

/* Definition for height topic */

ORB_DEFINE(flight_event, struct flight_event, flight_event_format);

/* Syslog printing flight events */

#ifdef CONFIG_ROCKETALT_EVENT_SYSLOG
static const char *FEVENT_STR[] = {
    [FEVENT_GROUNDED] = "Grounded", [FEVENT_ASCENT] = "Ascent",
    [FEVENT_APOGEE] = "Apogee",     [FEVENT_DESCENT] = "Descent",
    [FEVENT_LANDED] = "Landed",
};
#endif

/* Buffer for averaging velocity */

static float g_velbuf[NUMVEL_SAMPLES];
static struct circbuf_s g_velocities =
    CIRCBUF_INITIALIZER(g_velbuf, sizeof(g_velbuf));
static float g_avg_vel;

/* Data buffer for reading measurements */

static union sensor_data g_data[2];

/* Polling structure for polling files */

static struct pollfd g_fds[2];

/* Array of topics we're subscribed to */

static struct topic_s g_topics[2] = {
    [HEIGHT_IDX] = {.devno = 0, .meta = ORB_ID(fusion_height)},
    [VEL_IDX] = {.devno = 0, .meta = ORB_ID(sensor_velocity)},
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void update_avg_vel(float new)
{
  float old;
  if (circbuf_is_full(&g_velocities))
    {
      /* Remove old value from calculation and add new value */

      circbuf_read(&g_velocities, &old, sizeof(old));
      circbuf_write(&g_velocities, &new, sizeof(new));
      g_avg_vel -= (old / (float)NUMVEL_SAMPLES);
      g_avg_vel += (new / (float)NUMVEL_SAMPLES);
    }
  else
    {
      /* Store the new velocity */

      circbuf_write(&g_velocities, &new, sizeof(new));

      /* Compute average velocity based off current samples only. This path
       * only happens for the first `NUMVEL_SAMPLES`.
       */

      g_avg_vel = 0.0f;
      unsigned num_measures = circbuf_used(&g_velocities) / sizeof(float);
      for (unsigned i = 0; i < num_measures; i++)
        {
          g_avg_vel += (g_velbuf[i] / (float)num_measures);
        }
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int main(int argc, char **argv)
{
  int err;
  int ret;
  int event_fd;
  struct flight_event event;
  enum fevent_e current = FEVENT_GROUNDED;

  /* Set up flight event topic for publishing */

  event_fd = orb_advertise_multi_queue(ORB_ID(flight_event), NULL, NULL,
                                       CONFIG_ROCKETALT_EVENT_QLEN);
  if (event_fd < 0)
    {
      syslog(LOG_ERR | LOG_USER,
             "Could not advertise flight_event topic: %d\n", errno);
      ret = EXIT_FAILURE;
      return ret;
    }

  syslog(LOG_INFO | LOG_USER, "flight_event topic advertised.\n");

  /* Subscribe to uORB topics */

  for (int i = 0; i < array_len(g_topics); i++)
    {
      g_fds[i].fd = orb_subscribe_multi(g_topics[i].meta, g_topics[i].devno);
      if (g_fds[i].fd < 0)
        {
          syslog(LOG_ERR | LOG_USER, "Could not subscribe to %s%d: %d\n",
                 g_topics[i].meta->o_name, g_topics[i].devno, errno);
          ret = EXIT_FAILURE;
          goto clean_fds;
        }

      /* Initialize polling fields */

      g_fds[i].events = POLLIN;
      g_fds[i].revents = 0;
    }

  syslog(LOG_INFO | LOG_USER, "Event topic subscribed to inputs.\n");

  /* Give an initial value to our data that is consistent with
   * FEVENT_GROUNDED so we don't compute a crazy event. Zero velocity
   * and zero height is reasonable for this.
   */

  g_data[HEIGHT_IDX].height.height = 0.0f;
  g_data[VEL_IDX].vel.velocity = 0.0f;

  /* Forever try to detect events */

  for (;;)
    {
      /* Poll to read some height and velocity data */

      err = poll(g_fds, array_len(g_fds), -1);
      if (err <= 0)
        {
          syslog(LOG_ERR | LOG_USER, "Failed to poll: %d\n", errno);
          continue; /* Try again */
        }

      /* We have some data, read it in! */

      for (unsigned i = 0; i < array_len(g_fds); i++)
        {
          if (g_fds[i].revents & POLLIN)
            {
              g_fds[i].revents = 0; /* Clear events */

              /* Update our average velocity */

              switch (i)
                {
                case HEIGHT_IDX:
                  err = orb_copy_multi(g_fds[i].fd, &g_data[i].height,
                                       g_topics[i].meta->o_size);
                  break;
                case VEL_IDX:
                  err = orb_copy_multi(g_fds[i].fd, &g_data[i].vel,
                                       g_topics[i].meta->o_size);
                  break;
                }

              if (err < 0)
                {
                  syslog(LOG_ERR | LOG_USER, "Couldn't read %s%d: %d\n",
                         g_topics[i].meta->o_name, g_topics[i].devno, errno);
                  continue; /* Try the next topic */
                }

              if (i == VEL_IDX)
                {
                  update_avg_vel(g_data[i].vel.velocity);
                }
            }
        }

      /* Based on height, velocity and current flight state, determine
       * next event.
       */

      switch (current)
        {
        case FEVENT_GROUNDED:
          /* If height has increased by some amount and velocity is
           * high, we are now going up.
           */

          if (g_data[HEIGHT_IDX].height.height >= MIN_TAKEOFF_ALT &&
              g_data[VEL_IDX].vel.velocity >= MIN_TAKEOFF_VEL)
            {
              event.event = FEVENT_ASCENT;
              event.timestamp = orb_absolute_time();
            }

          /* Otherwise, nothing has changed */
          break;

        case FEVENT_ASCENT:
          /* If our velocity reaches 0 and we are reasonably close to
           * the predicted apogee, then we have reached apogee!
           */

          if (g_data[HEIGHT_IDX].height.height >=
                  APOGEE_PERCENTAGE * dummy_config.pred_apogee &&
              is_zero(g_avg_vel, ZERO_VEL_TOL) &&
              is_zero(g_data[VEL_IDX].vel.velocity, ZERO_VEL_TOL))
            {
              event.event = FEVENT_APOGEE;
              event.timestamp = orb_absolute_time();
            }

          /* Otherwise, nothing has changed */

          break;

        case FEVENT_APOGEE:

          /* If our velocity is now negative, we're descending. */

          if (g_avg_vel <= MIN_DESCENT_VEL)
            {
              event.event = FEVENT_DESCENT;
              event.timestamp = orb_absolute_time();
            }

          /* Otherwise, nothing has changed */

          break;

        case FEVENT_DESCENT:

          /* If our velocity is 0, we have a constant height after
           * descending and therefore must have landed.
           */

          if (is_zero(g_avg_vel, ZERO_VEL_TOL) &&
              is_zero(g_data[VEL_IDX].vel.velocity, ZERO_VEL_TOL))
            {
              event.event = FEVENT_LANDED;
              event.timestamp = orb_absolute_time();
            }

          /* Otherwise, nothing has changed */

          break;

        case FEVENT_LANDED:

          /* If we are in a landed state, move to grounded now that the
           * landing event has been reported so that we're prepared for
           * another launch.
           */

          event.event = FEVENT_GROUNDED;
          event.timestamp = orb_absolute_time();
          break;
        }

      /* If our event has been updated, publish it */

      if (event.event != current)
        {
          err = orb_publish(ORB_ID(flight_event), event_fd, &event);
          if (err)
            {
              syslog(LOG_ERR | LOG_USER, "Couldn't publish event: %d\n",
                     errno);
            }

          /* We update our current state regardless of whether or not
           * the event was published.
           */

          current = event.event;
#ifdef CONFIG_ROCKETALT_EVENT_SYSLOG
          syslog(LOG_INFO | LOG_USER,
                 "Flight event: %s @ height=%.2f m, vel=%.2f m/s\n",
                 FEVENT_STR[event.event], g_data[HEIGHT_IDX].height.height,
                 g_data[VEL_IDX].vel.velocity);
#endif
        }
    }

clean_fds:

  for (int i = 0; i < array_len(g_fds); i++)
    {
      if (g_fds[i].fd > 0)
        {
          orb_unsubscribe(g_fds[i].fd);
        }
    }

  orb_unadvertise(event_fd);
  return ret;
}
