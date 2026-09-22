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

#include "sensor/continuity.h"
#include <sensor/voltage.h>

#include "../../common/common.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct chan_s
{
  int fd;     /* File descriptor to the voltage topic */
  int devno;  /* Voltage topic device number */
  uint8_t id; /* Channel ID (for continuity channel) */
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: channel_read
 *
 * Description:
 *   Read voltage data tied to a specific continuity channel.
 *
 * Input Parameters:
 *   chan - The channel to read voltage data for
 *   data - Where to store the voltage data
 *
 * Returned Value:
 *   0 on success, error code on failure.
 *
 ****************************************************************************/

static int channel_read(const struct chan_s *chan,
                        struct sensor_voltage *data)
{
  int err = 0;

  err = orb_copy(ORB_ID(sensor_voltage), chan->fd, data);
  if (err)
    {
      syslog(LOG_ERR | LOG_USER, "Couldn't get sensor_voltage%d data: %d\n",
             chan->devno, errno);
      err = errno;
    }

  return err;
}

/****************************************************************************
 * Name: channel_publish
 *
 * Description:
 *   Publish continuity data.
 *
 * Input Parameters:
 *   chan - The file descriptor of the continuity topic
 *   data - The continuity data to publish
 *
 * Returned Value:
 *   0 on success, error code on failure.
 *
 ****************************************************************************/

static int continuity_publish(int fd, const struct sensor_continuity *data)
{
  int err;

  err = orb_publish(ORB_ID(sensor_continuity), fd, data);
  if (err)
    {
      syslog(LOG_ERR | LOG_USER, "Couldn't publish continuity: %d\n", errno);
      err = errno;
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
  float v_thresh;
  int num_chans;
  struct pollfd *pfds = NULL;
  struct chan_s *channels = NULL;
  struct sensor_voltage voltage;
  struct sensor_continuity continuity;

  int devno = 0;           /* Device number for continuity topic */
  uint8_t channo_base = 0; /* Channel ID base */

  /* Parse command line arguments */

  while ((c = getopt(argc, argv, ":n:c:")) != -1)
    {
      switch (c)
        {
        case 'n':
          devno = atoi(optarg); /* Device number for continuity topic */
          break;

        case 'c':
          /* Base number at which to start enumerating continuity channel IDs
           */

          channo_base = strtoul(optarg, NULL, 0);
          break;

        case ':':
          syslog(LOG_ERR | LOG_USER, "Option -%c requires an argument.\n",
                 optopt);
          return EXIT_FAILURE;

        case '?':
          syslog(LOG_ERR | LOG_USER, "Unknown option '-%c'.\n", optopt);
          break; /* Don't exit, parse other options */

        default:
          syslog(LOG_ERR | LOG_USER, "Usage: continuity [-n devno] [-c "
                                     "channo] v_thresh topicno [...]\n");
          return EXIT_FAILURE;
        }
    }

  /* Check for threshold voltage */

  if (argc <= optind)
    {
      syslog(LOG_ERR | LOG_USER, "Expected v_thresh argument.\n");
      return EXIT_FAILURE;
    }

  v_thresh = strtof(argv[optind], NULL);
  optind++;

  /* Verify we have some actual inputs to work with */

  num_chans = argc - optind;
  if (num_chans < 1)
    {
      syslog(LOG_ERR | LOG_USER,
             "Expected at least one sensor_voltage topic as input.\n");
      return EXIT_FAILURE;
    }

  /* Allocate structures for each input voltage */

  pfds = malloc(num_chans * sizeof(struct pollfd));
  if (pfds == NULL)
    {
      syslog(LOG_ERR | LOG_USER, "Couldn't allocate pollfd structures: %d\n",
             errno);
      return EXIT_FAILURE;
    }

  channels = malloc(num_chans * sizeof(struct chan_s));
  if (channels == NULL)
    {
      syslog(LOG_ERR | LOG_USER, "Couldn't allocate channel structures: %d\n",
             errno);
      ret = EXIT_FAILURE;
      goto free_pfds;
    }

  /* Initialize structures */

  for (int i = 0; i < num_chans; i++, optind++)
    {
      pfds[i].revents = 0;
      pfds[i].events = POLLIN;
      channels[i].id = channo_base + i;
      channels[i].devno = strtoul(argv[optind], NULL, 10);

      pfds[i].fd =
          orb_subscribe_multi(ORB_ID(sensor_voltage), channels[i].devno);
      if (pfds[i].fd < 0)
        {
          syslog(LOG_ERR | LOG_USER,
                 "Couldn't subscribe to sensor_voltage%d\n",
                 channels[i].devno);
          ret = EXIT_FAILURE;
          goto clean_channels;
        }

      channels[i].fd = pfds[i].fd;
    }

  /* Set up continuity topic for publishing */

  cont_fd = orb_advertise_multi_queue(ORB_ID(sensor_continuity), NULL, &devno,
                                      CONFIG_ROCKETALT_CONTMON_QLEN);
  if (cont_fd < 0)
    {
      syslog(LOG_ERR | LOG_USER,
             "Could not advertise sensor_continuity%d: %d\n", devno, errno);
      ret = EXIT_FAILURE;
      goto clean_channels;
    }

  syslog(LOG_INFO | LOG_USER, "sensor_continuity%d advertised.\n", devno);

  /* Forever convert voltages to continuity uORB output */

  for (;;)
    {
      err = poll(pfds, num_chans, -1);
      if (err <= 0)
        {
          syslog(LOG_ERR | LOG_USER, "Failed to poll: %d\n", errno);
          continue; /* Try again */
        }

      for (int i = 0; i < num_chans; i++)
        {
          if (pfds[i].revents & POLLIN)
            {
              /* We have data. Read the topic and publish the result if the
               * read succeeds.
               */

              err = channel_read(&channels[i], &voltage);
              if (err == 0)
                {
                  continuity.timestamp = voltage.timestamp;
                  continuity.id = channels[i].id;
                  continuity.continuous = voltage.voltage > v_thresh ? 1 : 0;
                  continuity_publish(cont_fd, &continuity);
                }

              pfds[i].revents = 0; /* Clear the events for next time */
            }
        }
    }

  orb_unadvertise(cont_fd);

clean_channels:
  for (int i = 0; i < num_chans; i++)
    {
      if (pfds[i].fd > 0)
        {
          orb_unsubscribe(pfds[i].fd);
        }
    }

  free(channels);

free_pfds:
  free(pfds);

  return ret;
}
