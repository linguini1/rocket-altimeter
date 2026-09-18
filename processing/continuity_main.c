/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <errno.h>
#include <fcntl.h>
#include <getopt.h>
#include <stdint.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <sys/poll.h>
#include <sys/types.h>
#include <syslog.h>
#include <unistd.h>

#include <nuttx/analog/adc.h>
#include <nuttx/analog/ioctl.h>

#include <uORB/uORB.h>

#include "../common/common.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* ADC measurement conversion */

#define measure_to_volts(r)                                                  \
  ((r) * CONFIG_ROCKETALT_CONTMON_VMAX /                                     \
   CONFIG_ROCKETALT_CONTMON_ADCRESOLUTION)

/* Program already knows about continuity data */

ORB_DECLARE(sensor_continuity);

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct chan_s
{
  char *path;     /* Path to ADC device */
  int fd;         /* File descriptor to the ADC device */
  uint8_t id;     /* ID of this channel */
  uint8_t channo; /* Channel number of the ADC device for this channel */
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Channels to measure the continuity of */

static struct chan_s g_channels[CONFIG_ROCKETALT_CONTMON_NUMCHANS];

/* List of file descriptor structures for us to use when polling */

static struct pollfd g_fds[CONFIG_ROCKETALT_CONTMON_NUMCHANS];

/* Optional debug output format string */

#ifdef CONFIG_DEBUG_UORB
static const char sensor_continuity_format[] =
    "sensor_continuity - timestamp:%" PRIu64 ",continuous:" PRIu8;
#endif

/* Definition for continuity topic */

ORB_DEFINE(sensor_continuity, struct sensor_continuity,
           sensor_continuity_format);

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: channel_deinit
 *
 * Description:
 *   De-initialize a channel being monitored for continuity.
 *
 * Input Parameters:
 *   chan - The channel being monitored for continuity
 *
 * Returned Value:
 *   0 on success, error code on failure.
 *
 ****************************************************************************/

static int channel_deinit(struct chan_s *chan)
{
  int err = 0;

  if (chan->fd < 0) return err; /* Can't close an invalid fd */

  err = close(chan->fd);
  if (err < 0)
    {
      syslog(LOG_ERR | LOG_USER, "Couldn't close channel %d: %d\n", chan->id,
             errno);
    }

  return err;
}

#ifdef CONFIG_ROCKETALT_CONTMON_SWTRIG
/****************************************************************************
 * Name: channel_trigger
 *
 * Description:
 *   Executes a software trigger for the ADC associated with the channel.
 *
 * Input Parameters:
 *   chan - The channel to trigger a measurement for
 *
 * Returned Value:
 *   0 on success, error code on failure.
 *
 ****************************************************************************/

static int channel_trigger(struct chan_s *chan)
{
  int err;

  err = ioctl(chan->fd, ANIOC_TRIGGER, 0);
  if (err < 0)
    {
      syslog(LOG_ERR | LOG_USER,
             "ADC trigger ioctl failed for channel %d: %d\n", chan->id,
             errno);
    }

  return err;
}
#endif /* CONFIG_ROCKETALT_CONTMON_SWTRIG */

/****************************************************************************
 * Name: channel_read
 *
 * Description:
 *   Reads the ADC value for this channel.
 *
 * Input Parameters:
 *   chan - The channel to read the value of
 *   data - A pointer to where to store the data that was read
 *
 * Returned Value:
 *   0 on success, error code on failure.
 *
 ****************************************************************************/

static int channel_read(struct chan_s *chan, struct adc_msg_s *data)
{
  int err = 0;
  ssize_t bread;

  /* TODO: I only want data from the associated `channo` */

  bread = read(chan->fd, data, sizeof(*data));
  if (bread < 0)
    {
      syslog(LOG_ERR | LOG_USER, "Failed to read channel %d: %d\n", chan->id,
             errno);
      return errno;
    }
  else if (bread == 0)
    {
      syslog(LOG_ERR | LOG_USER, "Failed to read channel %d: no data\n",
             chan->id);
      return EAGAIN;
    }

  return err;
}

/****************************************************************************
 * Name: publish_continuity
 *
 * Description:
 *   Publishes sensor_continuity data for the passed channel and ADC
 *   measurement.
 *
 * Input Parameters:
 *   fd - The file descriptor for the continuity topic
 *   chan - The channel associated with this measurement
 *   data - The voltage measurement from the ADC
 *
 * Returned Value:
 *   0 on success, error code on failure.
 *
 ****************************************************************************/

static int publish_continuity(int fd, struct chan_s *chan,
                              struct adc_msg_s *data)
{
  int err;
  struct sensor_continuity cont;
  int32_t voltage;

  cont.timestamp = orb_absolute_time();
  voltage = measure_to_volts(data->am_data);
  cont.continuous = voltage >= CONFIG_ROCKETALT_CONTMON_VTHRESH ? 1 : 0;
  cont.id = chan->id;

  err = orb_publish(ORB_ID(sensor_continuity), fd, &cont);
  if (err)
    {
      syslog(LOG_ERR | LOG_USER,
             "Couldn't publish continuity for channel %d: %d\n", chan->id,
             errno);
    }

  return err;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int main(int argc, char **argv)
{
  int c;
  int err;
  int ret = EXIT_FAILURE;
  int cont_fd;
  int devno = 0;
  struct adc_msg_s adc_data;

  /* Parse command line arguments */

  while ((c = getopt(argc, argv, ":n:")) != -1)
    {
      switch (c)
        {
        case 'n':
          devno = atoi(optarg);
          break;

        case ':':
          syslog(LOG_ERR | LOG_USER, "Option -%c requires an argument.\n",
                 optopt);
          return EXIT_FAILURE;

        case '?':
          syslog(LOG_ERR | LOG_USER, "Unknown option '-%c'.\n", optopt);
          break; /* Don't exit, parse other options */

        default:
          syslog(LOG_ERR | LOG_USER,
                 "Usage: continuity [-n devno] devpath channo ...\n");
          return EXIT_FAILURE;
        }
    }

  /* Ensure enough arguments were passed to the program for all of the ADC
   * device paths and channel numbers.
   */

  if (argc - optind < 2 * CONFIG_ROCKETALT_CONTMON_NUMCHANS)
    {
      syslog(LOG_ERR | LOG_USER,
             "Not enough positional arguments for all channels. Got %d, need "
             "%d\n",
             argc - optind, 2 * CONFIG_ROCKETALT_CONTMON_NUMCHANS);
      return EXIT_FAILURE;
    }

  /* Initialize channels and pollfds */

  for (int i = 0; i < array_len(g_channels); i++)
    {
      g_channels[i].id = i + 1;
      g_channels[i].fd = -1;
      g_channels[i].path = argv[optind];
      g_channels[i].channo = atoi(argv[optind + 1]);
      optind += 2;

      syslog(LOG_INFO | LOG_USER, "Channel %d using %s, ADC channo %d\n",
             g_channels[i].id, g_channels[i].path, g_channels[i].channo);

      g_fds[i].fd = -1;
      g_fds[i].events = POLLIN;
      g_fds[i].revents = 0;
    }

  /* Set up continuity topic for publishing */

  cont_fd = orb_advertise_multi_queue(ORB_ID(sensor_continuity), NULL, &devno,
                                      CONFIG_ROCKETALT_CONTMON_VOLTAGE_QLEN);
  if (cont_fd < 0)
    {
      syslog(LOG_ERR | LOG_USER,
             "Could not advertise sensor_continuity%d: %d\n", devno, errno);
      return EXIT_FAILURE;
    }

  syslog(LOG_INFO | LOG_USER, "sensor_continuity%d advertised.\n", devno);

  /* Open ADC devices */

  for (int i = 0; i < array_len(g_channels); i++)
    {
      err = open(g_channels[i].path, O_RDONLY);
      if (err < 0)
        {
          syslog(LOG_ERR | LOG_USER, "Could not open %s: %d\n",
                 g_channels[i].path, errno);
          ret = EXIT_FAILURE;
          goto cleanup_channels;
        }

      g_channels[i].fd = err; /* Store resulting fd */
      g_fds[i].fd = err;
    }

  /* Forever convert ADC measurements to voltage uORB output */

  for (;;)
    {
      /* For each channel, trigger an ADC measurement if that's necessary
       * (i.e. software trigger required). Nothing we can do if this fails.
       */
#ifdef CONFIG_ROCKETALT_CONTMON_SWTRIG
      for (int i = 0; i < array_len(g_channels); i++)
        {
          channel_trigger(&g_channels[i]);
        }
#endif /* CONFIG_ROCKETALT_CONTMON_SWTRIG */

      /* Now we poll on the file descriptors for each channel until something
       * is ready to handle.
       */

      err = poll(g_fds, array_len(g_fds), 0);
      if (err <= 0)
        {
          syslog(LOG_ERR | LOG_USER, "Failed to poll: %d\n", errno);
          continue; /* Try again */
        }

      for (int i = 0; i < array_len(g_fds); i++)
        {
          if (g_fds[i].revents & POLLIN)
            {
              /* We have data. Read the channel and publish the result if the
               * read succeeds.
               */

              err = channel_read(&g_channels[i], &adc_data);
              if (err == 0)
                {
                  publish_continuity(cont_fd, &g_channels[i], &adc_data);
                }

              g_fds[i].revents = 0; /* Clear the events for next time */
            }
        }

#ifdef CONFIG_ROCKETALT_CONTMON_SWTRIG
      /* If we're manually triggering the ADC, make sure we only read the ADCs
       * often as the user configured.
       */

      sleep(CONFIG_ROCKETALT_CONTMON_PERIOD);
#endif /* CONFIG_ROCKETALT_CONTMON_SWTRIG */
    }

cleanup_channels:
  for (int i = 0; i < array_len(g_channels); i++)
    {
      channel_deinit(&g_channels[i]);
    }

  orb_unadvertise(cont_fd);

  return ret;
}
