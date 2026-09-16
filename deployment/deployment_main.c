/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <fcntl.h>
#include <math.h>
#include <nuttx/sched.h>
#include <poll.h>
#include <signal.h>
#include <stdint.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <syslog.h>
#include <time.h>

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

/* Signal used for deployment timers */

#define TIMER_SIG SIGALRM

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
  char *path;      /* Channel GPIO path */
  timer_t timerid; /* ID of this channel's timer (if used) */
  int fd;          /* File descriptor to the channel GPIO */
  int cond;        /* Deployment condition bitmask */
  float altitude;  /* Altitude in meters */
  uint16_t time;   /* Deployment time in seconds (for timer) */
  uint8_t id;      /* Channel ID */
  bool fired;      /* Whether or not this channel has been deployed */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Deployment channels */

static struct pyrochan_s g_channels[CONFIG_ROCKETALT_DEPLOYMENT_NUMCHANS];

/* Handle to the thread which handles the timer notifications */

pthread_t g_thread;
bool g_thread_started; /* Record if we started the thread */

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
 * Name: channel_start_timer
 *
 * Description:
 *   Starts the channel's timer for timer-based deployment.
 *
 *   WARNING: This function should only be called on channels which have
 *   a valid `time` member value, and are configured with COND_TIME as a
 *   deployment condition.
 *
 *   WARNING: NuttX only guarantees support for CLOCK_REALTIME clock on
 *   POSIX timers. It may be that the wall-clock time of CLOCK_REALTIME
 *   resets to the time since boot on _most_ devices. Timer-based
 *   deployment is thus not robust to spurious reboots during flight; if
 *   timers get re-configured on reboot and the rocket is flying, there is
 *   no way to know how much time has already passed since initial ascent
 *   and thus the timer-based deployment will be offset. We still
 *   re-configure timers anyways since it's likely a better idea to blow
 *   deployment charges at some after landing than to keep live energetics
 *   in the (presumably damaged) rocket.
 *
 * Input Parameters:
 *   chan - The channel to start the timer for
 *
 * Returned Value:
 *   0 on success, error code on failure.
 *
 ****************************************************************************/

static int channel_start_timer(struct pyrochan_s *chan)
{
  int err;
  sigevent_t evp;
  struct itimerspec config;

  DEBUGASSERT(chan->cond & COND_TIME);

  /* Create the timer for this channel.
   *
   * The timer will trigger a signal, and the signal value will carry a
   * pointer to the channel the timer corresponds to.
   */

  evp.sigev_notify = SIGEV_SIGNAL;
  evp.sigev_value.sival_ptr = chan;
  evp.sigev_signo = TIMER_SIG;

  err = timer_create(CLOCK_REALTIME, &evp, &chan->timerid);
  if (err < 0)
    {
      err = errno;
      syslog(LOG_ERR | LOG_USER, "Couldn't create timer for channel %d: %d\n",
             chan->id, err);
      return err;
    }

  /* Configure the timer.
   *
   * `it_value` is configured such that our timer will expire
   * `chan->time` seconds from this call.
   *
   * `it_interval` is configured such that the timer expires just once.
   */

  config.it_value.tv_sec = chan->time;
  config.it_value.tv_nsec = 0;

  config.it_interval.tv_sec = 0;
  config.it_interval.tv_nsec = 0;

  err = timer_settime(chan->timerid, 0, &config, NULL);
  if (err < 0)
    {
      err = errno;
      syslog(LOG_ERR | LOG_USER,
             "Couldn't configure timer for channel %d: %d\n", chan->id, err);
      timer_delete(chan->timerid);
      return err;
    }

  syslog(LOG_INFO | LOG_USER, "Started %us timer for channel %d\n",
         chan->time, chan->id);
  return err;
}

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

  err = ioctl(chan->fd, GPIOC_WRITE, 1);
  if (err < 0)
    {
      syslog(LOG_ERR | LOG_USER, "Couldn't fire channel %d: %d\n", chan->id,
             errno);
      return errno;
    }

    /* Channel fired successfully.
     * TODO: turn it off again! But how much time do we wait to turn it off?
     *
     * Shouldn't we also do this asynchronously so that we don't delay the
     * deployment event being published/other simultaneous channels that need
     * turning off?
     */

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
 * Name: timer_thread
 *
 * Description:
 *   Thread which handles timer events for deploying timer-based channels.
 *
 * Returned Value:
 *   Error code as exit status. NOTE: this thread should not return, it should
 *   last the lifetime of the parent process.
 *
 ****************************************************************************/

static void *timer_thread(void *arg)
{
  int err;
  int dep_fd = (int)arg;
  sigset_t set;
  siginfo_t info;
  struct pyrochan_s *chan;

  syslog(LOG_INFO | LOG_USER, "Timer thread started.\n");

  /* Configure the set of signals we're waiting for (just timer signals) */

  err = sigemptyset(&set);
  if (err < 0)
    {
      syslog(LOG_ERR | LOG_USER, "Couldn't configure signal set: %d\n",
             errno);
      return (void *)(uintptr_t)errno;
    }

  err = sigaddset(&set, TIMER_SIG); /* We wait for timer signal */
  if (err < 0)
    {
      syslog(LOG_ERR | LOG_USER, "Couldn't configure signal set: %d\n",
             errno);
      return (void *)(uintptr_t)errno;
    }

  err = sigaddset(&set, SIGABRT); /* We also allow a cancellation signal */
  if (err < 0)
    {
      syslog(LOG_ERR | LOG_USER, "Couldn't configure signal set: %d\n",
             errno);
      return (void *)(uintptr_t)errno;
    }

  /* We specifically unblock the timer signal from this thread's set of
   * blocked signals.
   */

  err = pthread_sigmask(SIG_UNBLOCK, &set, NULL);
  if (err)
    {
      syslog(LOG_ERR | LOG_USER, "Couldn't unblock timer signal: %d\n", err);
      return (void *)(uintptr_t)err;
    }

  /* We are waiting for the timer signal. */

  for (;;)
    {
      /* Block until we receive a signal, with continue to re-block on
       * spurious wake-ups.
       */

      err = sigwaitinfo(&set, &info);
      if (err < 0)
        {
          syslog(LOG_ERR | LOG_USER,
                 "Error while waiting for timer signal: %d", errno);
          continue;
        }

      /* If this was a cancellation signal, stop execution and return */

      if (info.si_signo == SIGABRT)
        {
          syslog(LOG_INFO | LOG_USER, "Timer thread cancelled.\n");
          return 0;
        }

      /* Handle the timer expiration by deploying the channel and indicating
       * the deployment event.
       */

      chan = (struct pyrochan_s *)info.si_value.sival_ptr;

      err = channel_fire(chan);
      if (err == 0)
        {
          err = publish_deployment(dep_fd, chan->id);

          /* Not really much to do if this fails; we continue so we can fire
           * any other timer-based channels.
           */
        }

      /* Clean up the expired timer */

      err = timer_delete(chan->timerid);
      if (err < 0)
        {
          syslog(LOG_ERR | LOG_USER, "Couldn't clean up expired timer: %d\n",
                 errno);
        }
    }

  return 0;
}

/****************************************************************************
 * Name: start_timer_thread
 *
 * Description:
 *   Sets up the timer thread for handling timer-based deployment channels.
 *
 * Input Parameters:
 *   dep_fd - The file descriptor to the deployment topic, so the thread can
 *            publish deployment events.
 *
 * Returned Value:
 *   0 on success, negated errno on failure.
 *
 ****************************************************************************/

static int start_timer_thread(int dep_fd)
{
  int err;
  pthread_attr_t attr;
  struct sched_param sched_param;

  err = pthread_attr_init(&attr);
  if (err)
    {
      return err;
    }

  /* Configure the stack size to what the user decided. */

  err = pthread_attr_setstacksize(
      &attr, CONFIG_ROCKETALT_DEPLOYMENT_TMRTHREAD_STACKSIZE);
  if (err)
    {
      syslog(LOG_ERR | LOG_USER, "Couldn't set timer thread stack size: %d",
             err);
      /* Not a fatal error, try the default if this was bad. */
    }

  /* Configure the thread priority to be the same as this process.
   *
   * We first get the existing schedule parameters so we don't overwrite
   * something else.
   */

  err = pthread_attr_getschedparam(&attr, &sched_param);
  if (err)
    {
      /* This is not fatal, we'll try to just set the priority anyways. */

      syslog(LOG_ERR | LOG_USER, "Couldn't get scheduler parameters: %d\n",
             err);
    }

  sched_param.sched_priority = CONFIG_ROCKETALT_DEPLOYMENT_PRIORITY;
  err = pthread_attr_setschedparam(&attr, &sched_param);
  if (err)
    {
      syslog(LOG_ERR | LOG_USER, "Couldn't set timer thread priority: %d",
             err);
      return err; /* This is a problem */
    }

  err = pthread_create(&g_thread, &attr, timer_thread, (void *)dep_fd);
  if (err)
    {
      syslog(LOG_ERR | LOG_USER, "Could not create timer thread: %d\n", err);
      return err;
    }

  g_thread_started = true;
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
  void *threadret;
  union sigval cancelval;
  struct pollfd fds[2];
  union sensor_data data[2];
  g_thread_started = false; /* Initially not started */

  /* Ensure that the number of arguments aligns with the number of GPIO device
   * paths we're expecting.
   */

  if (argc < CONFIG_ROCKETALT_DEPLOYMENT_NUMCHANS + 1)
    {
      syslog(LOG_ERR | LOG_USER,
             "Received %d possible paths, should have %d\n", argc,
             CONFIG_ROCKETALT_DEPLOYMENT_NUMCHANS + 1);
      return EXIT_FAILURE;
    }

  /* Initialize deployment channels */

  for (int i = 0; i < array_len(g_channels); i++)
    {
      g_channels[i].fd = -1;
      g_channels[i].id = i + 1;
      g_channels[i].path = argv[i + 1];
      g_channels[i].fired = false;

      syslog(LOG_USER | LOG_INFO, "Channel %d using %s\n", g_channels[i].id,
             g_channels[i].path);
    }

  /* Configure deployment channels
   * TODO: do this dynamically from the configuration file.
   */

  g_channels[0].cond = COND_APOGEE | COND_TIME;
  g_channels[0].time = 10;

  g_channels[1].cond = COND_ALT | COND_TIME;
  g_channels[1].altitude = 305.0f; /* 1000 ft */
  g_channels[1].time = 20;

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

  /* Set up deployment event topic for publishing */

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

  /* If there are any channels configured to use timer-based deployment, set
   * up a timer thread.
   */

  for (int i = 0; i < array_len(g_channels); i++)
    {
      if (g_channels[i].cond & COND_TIME)
        {
          err = start_timer_thread(dep_fd);

          /* We'll continue on errors for now on the hope that other channels
           * use more robust conditions, like altitude-based deployment.
           */

          break; /* Only set up the timer thread once. */
        }
    }

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

      /* If we have detected ascent, we have just lifted off. Start
       * the backup timers if they're configured.
       */

      if (data[EVENT_IDX].event.event == FEVENT_ASCENT)
        {
          for (int i = 0; i < array_len(g_channels); i++)
            {
              if (g_channels[i].cond & COND_TIME)
                {
                  err = channel_start_timer(&g_channels[i]);

                  /* We will continue on errors for now on the hopes that
                   * channels also have more robust, altitude-based deployment
                   * conditions.
                   */
                }
            }
        }

      /* If we have detected apogee, fire any channel which is meant
       * to fire at apogee.
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

      /* If we are past apogee, we can deploy any channels that have
       * an altitude limit above our current altitude.
       *
       * Only do this if we have detected apogee or descent. We don't
       * want to deploy if a pressure spike (like Mach dip) causes
       * height to increase beyond our configured deployment altitude.
       *
       * NOTE: It is permissible to check >= FEVENT_APOGEE because we
       * would have already skipped past FEVENT_LANDED earlier in the
       * loop, so this effectively checks for apogee or ascent.
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

clean_thread:
  if (g_thread_started)
    {
      cancelval.sival_ptr = NULL;

      /* NOTE: NuttX has no `pthread_sigqueue` implementation. However, the
       * `sigqueue` implementation is documented as taking a 'task ID' as the
       * first argument. As far as I can tell, NuttX gives a unique task ID to
       * each process AND thread. So it should be safe to use the pthread
       * handle as a task ID here.
       */

#if 0
      err = sigqueue(g_thread, SIGABRT, cancelval);
#else
      err = sigqueue(g_thread, SIGABRT, cancelval);
#endif

      if (err)
        {
          syslog(LOG_ERR | LOG_USER,
                 "Couldn't send timer thread cancel signal: %d\n", err);

          /* No point joining if we couldn't cancel. */

          goto clean_channels;
        }

      err = pthread_join(g_thread, &threadret);
      if (err)
        {
          syslog(LOG_ERR | LOG_USER, "Couldn't join to timer thread: %d\n",
                 err);
        }

      syslog(LOG_INFO | LOG_USER, "Timer thread exited with status %d\n",
             (int)threadret);
    }

clean_channels:
  for (int i = 0; i < array_len(g_channels); i++)
    {
      channel_deinit(&g_channels[i]);
    }

  return ret;
}
