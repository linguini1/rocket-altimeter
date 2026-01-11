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

/* Zero velocity detection in m/s */

#define ZERO_VEL_TOL (0.8f)
#define is_zero(vel, tol) ((vel) <= (tol) && (vel) >= -(tol))

/* Array length helper */

#define array_len(arr) (sizeof(arr) / sizeof(arr[0]))

/* Indices into arrays needed for height and velocity data */

#define HEIGHT_IDX (0)
#define VEL_IDX (1)

/* Program already knows about some topics */

ORB_DECLARE(fusion_height);
ORB_DECLARE(fusion_velocity);

/****************************************************************************
 * Private Types
 ****************************************************************************/

union sensor_data
{
  struct fusion_height height;
  struct fusion_velocity vel;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Optional debug output format string */

#ifdef CONFIG_DEBUG_UORB
static const char flight_event_format[] =
    "flight_event - timestamp:%" PRIu64 ",event:%u";
#endif

/* Definition for height topic */

ORB_DEFINE(flight_event, struct flight_event, flight_event_format);

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

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

  vel_fd = orb_subscribe_multi(ORB_ID(fusion_velocity), 0);
  if (vel_fd < 0)
    {
      syslog(LOG_ERR | LOG_USER,
             "Could not subscribe to fusion_velocity0: %d\n", errno);
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
           *
           * TODO: add thresholds
           */

          if (data[HEIGHT_IDX].height.height >= 10.0f &&
              data[VEL_IDX].vel.velocity > 10.0f)
            {
              event.event = FEVENT_ASCENT;
              event.timestamp = orb_absolute_time();
            }

          /* Otherwise, nothing has changed */
          break;

        case FEVENT_ASCENT:
          /* If our velocity reaches 0 and we are reasonably high up in the
           * air, then we have reached apogee!
           *
           * TODO: have an actual apogee estimation threshold, not just
           * exactly 0.
           */

          if (data[HEIGHT_IDX].height.height >= 300.0f &&
              is_zero(data[VEL_IDX].vel.velocity, ZERO_VEL_TOL))
            {
              event.event = FEVENT_APOGEE;
              event.timestamp = orb_absolute_time();
            }

          /* Otherwise, nothing has changed */

          break;

        case FEVENT_APOGEE:

          /* If our velocity is now negative, we're descending.
           *
           * Typical descent velocity is around 25ft/s, so anything faster
           * than 12ft/s should be good. That's ~3.6m/s.
           *
           * TODO: should base this off of some amount of averaging time so
           * we don't just move to descent from one anomalous measurement.
           */

          if (data[VEL_IDX].vel.velocity <= -3.6f)
            {
              event.event = FEVENT_DESCENT;
              event.timestamp = orb_absolute_time();
            }

          /* Otherwise, nothing has changed */

          break;

        case FEVENT_DESCENT:

          /* If our velocity is 0, we have a constant height after
           * descending and therefore must have landed.
           *
           * TODO: should base this off of some amount of averaging time so
           * we don't just move to descent from one anomalous measurement.
           */

          if (is_zero(data[VEL_IDX].vel.velocity, ZERO_VEL_TOL))
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
