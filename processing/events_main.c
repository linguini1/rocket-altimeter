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

/* Array length helper */

#define array_len(arr) (sizeof(arr) / sizeof(arr[0]))

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

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct processconfig_s dummy_config = {
    .pred_apogee = 8000.0f,
};

/* Optional debug output format string */

#ifdef CONFIG_DEBUG_UORB
static const char flight_event_format[] =
    "flight_event - timestamp:%" PRIu64 ",event:%u";
#endif

/* Definition for height topic */

ORB_DEFINE(flight_event, struct flight_event, flight_event_format);

/* Buffer for averaging velocity */

static float velbuf[NUMVEL_SAMPLES];
static struct circbuf_s velocities =
    CIRCBUF_INITIALIZER(velbuf, sizeof(velbuf));

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static float avg_vel(void)
{
  float vel = 0.0f;
  for (unsigned i = 0; i < NUMVEL_SAMPLES; i++)
    {
      vel += velbuf[i] / (float)NUMVEL_SAMPLES;
    }
  return vel;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int main(int argc, char **argv)
{
  int err;
  int ret;
  int vel_fd;
  int height_fd;
  int event_fd;
  union sensor_data data[2];
  struct pollfd fds[2];
  struct flight_event event;
  enum fevent_e current = FEVENT_GROUNDED;

  /* Set up flight event topic for publishing */

  event_fd =
      orb_advertise_multi_queue(ORB_ID(flight_event), NULL, NULL,
                                CONFIG_ROCKETALT_PROCESSING_EVENT_QLEN);
  if (event_fd < 0)
    {
      syslog(LOG_ERR | LOG_USER,
             "Could not advertise flight_event topic: %d\n", errno);
      ret = EXIT_FAILURE;
      return ret;
    }

  syslog(LOG_INFO | LOG_USER, "flight_event topic advertised.\n");

  /* Subscribe to height topic */

  height_fd = orb_subscribe_multi(ORB_ID(fusion_height), 0);
  if (height_fd < 0)
    {
      syslog(LOG_ERR | LOG_USER,
             "Could not subscribe to fusion_height0: %d\n", errno);
      ret = EXIT_FAILURE;
      goto clean_eventonly;
    }

  /* Subscribe to velocity topic */

  vel_fd = orb_subscribe_multi(ORB_ID(sensor_velocity), 0);
  if (vel_fd < 0)
    {
      syslog(LOG_ERR | LOG_USER,
             "Could not subscribe to sensor_velocity0: %d\n", errno);
      ret = EXIT_FAILURE;
      goto clean_alt;
    }

  /* Create our polling structure */

  fds[HEIGHT_IDX].fd = height_fd;
  fds[HEIGHT_IDX].events = POLLIN;
  fds[HEIGHT_IDX].revents = 0;

  fds[VEL_IDX].fd = vel_fd;
  fds[VEL_IDX].events = POLLIN;
  fds[VEL_IDX].revents = 0;

  /* Give an initial value to our data that is consistent with FEVENT_GROUNDED
   * so we don't compute a crazy event. Zero velocity and zero height is
   * reasonable for this.
   */

  memset(data, 0, sizeof(data));

  /* Forever try to detect events */

  for (;;)
    {
      /* Poll to read some height and velocity data */

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

              /* Store velocities in our averaging list */

              if (i == VEL_IDX)
                {
                  circbuf_overwrite(&velocities, &data[i].vel.velocity,
                                    sizeof(float));
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

          if (data[HEIGHT_IDX].height.height >= MIN_TAKEOFF_ALT &&
              data[VEL_IDX].vel.velocity >= MIN_TAKEOFF_VEL)
            {
              event.event = FEVENT_ASCENT;
              event.timestamp = orb_absolute_time();
            }

          /* Otherwise, nothing has changed */
          break;

        case FEVENT_ASCENT:
          /* If our velocity reaches 0 and we are reasonably close to the
           * predicted apogee, then we have reached apogee!
           */

          if (data[HEIGHT_IDX].height.height >=
                  APOGEE_PERCENTAGE * dummy_config.pred_apogee &&
              is_zero(avg_vel(), ZERO_VEL_TOL))
            {
              event.event = FEVENT_APOGEE;
              event.timestamp = orb_absolute_time();
            }

          /* Otherwise, nothing has changed */

          break;

        case FEVENT_APOGEE:

          /* If our velocity is now negative, we're descending. */

          if (avg_vel() <= MIN_DESCENT_VEL)
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

          if (is_zero(avg_vel(), ZERO_VEL_TOL))
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

          /* We update our current state regardless of whether or not the
           * event was published.
           */

          current = event.event;
        }
    }

  orb_unsubscribe(vel_fd);
clean_alt:
  orb_unsubscribe(height_fd);
clean_eventonly:
  orb_unadvertise(event_fd);
  return ret;
}
