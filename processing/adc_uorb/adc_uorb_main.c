/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <ctype.h>
#include <errno.h>
#include <fcntl.h>
#include <getopt.h>
#include <stdint.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <syslog.h>

#include <nuttx/analog/adc.h>
#include <nuttx/analog/ioctl.h>

#include <uORB/uORB.h>

#include <sensor/voltage.h>

#include "../../common/common.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct topic_s
{
  int fd;     /* File descriptor */
  int devno;  /* Device number */
  int channo; /* Associated channel number */
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: measure_to_volts
 *
 * Description:
 *   Turn an ADC measurement into voltage.
 *
 * Input Parameters:
 *   measure - The ADC measurement
 *   max_voltage - The maximum voltage in volts
 *   max_adc - The maximum value the ADC could put in `measure`
 *
 * Returned Value:
 *   Voltage
 *
 ****************************************************************************/

static float measure_to_volts(int32_t measure, float max_voltage,
                              int32_t adc_max)
{
  return (float)(measure)*max_voltage / (float)adc_max;
}

/****************************************************************************
 * Name: adc_read_sample
 *
 * Description:
 *   Read a sample from the ADC device.
 *
 * Input Parameters:
 *   fd - The ADC device file descriptor
 *   data - Where to store the sample
 *
 * Returned Value:
 *   0 on success, error code on failure. ENODATA if nothing was read.
 *
 ****************************************************************************/

static int adc_read_sample(int fd, struct adc_msg_s *data)
{
  ssize_t bread;

  bread = read(fd, data, sizeof(*data));
  if (bread < 0)
    {
      return errno;
    }
  else if (bread == 0)
    {
      return ENODATA;
    }

  return 0;
}

/****************************************************************************
 * Name: get_topic_by_chan
 *
 * Description:
 *   Finds the topic corresponding to `channo` in the topics array.
 *   WARN: check the value of the `devno` member in the returned reference.
 *
 * Input Parameters:
 *   topics - descr
 *   num_chans - descr
 *   channo - descr
 *
 * Returned Value:
 *   If the matching topic is found, its pointer is returned and its `channo`
 *   member will equal the input `channo` argument.
 *
 *   If no match is found, but there is space to allocate the topic, a pointer
 *   to where the topic can be initialized is returned and its `devno` will be
 *   -1.
 *
 *   If no match is found and there is no space left in the array, NULL is
 *   returned.
 *
 ****************************************************************************/

struct topic_s *get_topic_by_chan(struct topic_s *topics, int num_chans,
                                  uint8_t channo)
{
  for (int i = 0; i < num_chans; i++)
    {
      if (topics[i].devno < 0)
        {
          /* We hit unallocated channel, which means this channo won't have a
           * match and instead the topic needs to be populated.
           */

          return &topics[i];
        }
      else if (topics[i].channo == channo)
        {
          /* We hit a match, return it! */

          return &topics[i];
        }
    }

  return NULL; /* No match and out of space! */
}

/****************************************************************************
 * Name: topic_create
 *
 * Description:
 *   Creates the uORB sensor_voltage topic for a channel, and initializes the
 *   topic struct for it.
 *
 * Input Parameters:
 *   topic - Where to initialize the topic struct
 *   devno_base - The base device number to start from
 *   channo - The ADC channel number which feeds this topic
 *
 * Returned Value:
 *   0 on success, error code on failure.
 *
 ****************************************************************************/

static int topic_create(struct topic_s *topic, int devno_base, uint8_t channo)
{
  topic->channo = channo;
  topic->devno = devno_base + channo;
  topic->fd =
      orb_advertise_multi_queue(ORB_ID(sensor_voltage), NULL, &topic->devno,
                                CONFIG_ROCKETALT_PROCESSING_ADCUORB_QLEN);

  if (topic->fd < 0)
    {
      syslog(LOG_ERR | LOG_USER, "Could not advertise sensor_voltage%d: %d\n",
             topic->devno, errno);
      return errno;
    }

  syslog(LOG_INFO | LOG_USER, "sensor_voltage%d advertised for channel %u\n",
         topic->devno, channo);
  return 0;
}

/****************************************************************************
 * Name: publish_sample
 *
 * Description:
 *   Publishes an ADC measurement as a voltage to the output topic.
 *
 * Input Parameters:
 *   topic - The topic to publish to
 *   sample - The ADC sample data
 *   max_voltage - The maximum voltage the ADC can measure at an input
 *   adc_max - The maximum value the ADC can return in `sample->am_data`
 *
 * Returned Value:
 *   0 on success, error code on failure.
 *
 ****************************************************************************/

static int publish_sample(const struct topic_s *topic,
                          struct adc_msg_s *sample, float max_voltage,
                          int32_t adc_max)
{
  struct sensor_voltage voltage;

  DEBUGASSERT(sample->am_channel == topic->channo); /* Should match */

  /* Convert measurement */

  voltage.timestamp = orb_absolute_time();
  voltage.voltage = measure_to_volts(sample->am_data, max_voltage, adc_max);

  /* Publish the sample */

  if (orb_publish(ORB_ID(sensor_voltage), topic->fd, &voltage) < 0)
    {
      syslog(LOG_ERR | LOG_USER, "Couldn't publish to sensor_voltage%d: %d\n",
             topic->devno, errno);
      return errno;
    }

  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int main(int argc, char **argv)
{
  int c;
  int err;
  int ret = EXIT_FAILURE;
  int adcfd;
  int num_chans;
  int samples_to_read = 0;
  uint8_t adc_bits;
  float max_voltage;
  int32_t adc_max;
  struct topic_s *topics = NULL;
  struct topic_s *sel_topic;
  struct adc_msg_s sample;

  /* Command line arguments and their default values */

  char *adcpath = NULL;       /* Path to ADC device */
  int devno_base = 0;         /* Number to start voltage devnos at */
  unsigned period = 1;        /* Measurement period in seconds */
  bool software_trig = false; /* ADC assumed to not need software trigger */

  while ((c = getopt(argc, argv, ":n:sp:")) != -1)
    {
      switch (c)
        {
        case 'n':
          devno_base = atoi(optarg);
          break;

        case 's':
          software_trig = true;
          break;

        case 'p':
          period = strtoul(optarg, NULL, 10);
          break;

        case ':':
          syslog(LOG_ERR | LOG_USER, "Option -%c requires an argument.\n",
                 optopt);
          return EXIT_FAILURE;

        case '?':
          syslog(LOG_ERR | LOG_USER, "Unknown option '-%c'.\n", optopt);
          break; /* Don't exit, parse other options */

        default:
          syslog(LOG_ERR | LOG_USER, "Usage: adc_uorb [-n devnostart] [-p "
                                     "period] adcpath maxvoltage adc_bits\n");
          return EXIT_FAILURE;
        }
    }

  /* ADC device path */

  if (argc <= optind)
    {
      syslog(LOG_ERR | LOG_USER, "Expected ADC device path.\n");
      return EXIT_FAILURE;
    }

  adcpath = argv[optind];
  optind++;

  /* Maximum voltage that the ADC will experience on an input */

  if (argc <= optind)
    {
      syslog(LOG_ERR | LOG_USER, "Expected maximum voltage input.\n");
      return EXIT_FAILURE;
    }

  max_voltage = strtof(argv[optind], NULL);
  optind++;

  /* ADC resolution in bits */

  if (argc <= optind)
    {
      syslog(LOG_ERR | LOG_USER, "Expected maximum voltage input.\n");
      return EXIT_FAILURE;
    }

  adc_bits = strtoul(argv[optind], NULL, 10);
  adc_max = (2 << (adc_bits - 1)) - 1;
  optind++;

  /* First, we must access the ADC device */

  adcfd = open(adcpath, O_RDONLY);
  if (adcfd < 0)
    {
      syslog(LOG_ERR | LOG_USER, "Couldn't open '%s': %d\n", adcpath, errno);
      return EXIT_FAILURE;
    }

  /* Now we determine how many channels the ADC device has */

  num_chans = ioctl(adcfd, ANIOC_GET_NCHANNELS, 0);
  if (num_chans < 0)
    {
      syslog(LOG_ERR | LOG_USER,
             "Failed to get number of channels for '%s': %d\n ", adcpath,
             errno);
      ret = EXIT_FAILURE;
      goto clean_fd;
    }

  /* Using the number of channels available in the ADC device, we dynamically
   * allocate the correct number of structures for publishing data.
   */

  topics = malloc(sizeof(struct topic_s) * num_chans);
  if (topics == NULL)
    {
      syslog(LOG_ERR | LOG_USER, "Couldn't allocate channel topics: %d\n",
             errno);
      ret = EXIT_FAILURE;
      goto clean_fd;
    }

  /* NOTE: The value of `am_channel` represents the channel number for a
   * measurement from an ADC device. This value may correspond to an actual,
   * physical channel number (i.e. the ADC has 3 channels, with IDs 1-3).
   * However, the user may have selected to only use channels 1 and 3,
   * excluding 2. In this case, we can't assume a nice sequence of channel
   * IDs. It is not possible to tell what the channel IDs will be in advance
   * without looking at device-specific compile time macros, which is not
   * portable.
   *
   * We map channel IDs to uORB topic devnos by adding the channel number to
   * the `base_devno` number being used. This allows us to deterministically
   * refer to the channel's uORB topic from other programs.
   *
   * This unfortunately means that we need to find out all the channel IDs as
   * we read them. Initially, we'll mark the devno field as `-1` to indicate
   * that a device number and uORB topic hasn't been assigned yet. As we
   * encounter new channel IDs, we'll advertise the corresponding topic
   * 'on-the-fly' and then mark its devno as `base_devno + am_channel`.
   */

  for (int i = 0; i < num_chans; i++)
    {
      topics[i].fd = -1;
      topics[i].devno = -1;
      topics[i].channo = -1;
    }

  /* Turn ADC measurements into voltage measurements over uORB forever */

  for (;;)
    {
      /* If the ADC needs to be triggered in software, do that */

      if (software_trig)
        {
          err = ioctl(adcfd, ANIOC_TRIGGER, 0);
          if (err < 0)
            {
              syslog(LOG_ERR | LOG_USER, "Failed to trigger '%s': %d\n",
                     adcpath, errno);
              continue; /* Try again */
            }
        }

      /* Read as much data as is available and publish each measurement to the
       * correct uORB topic!
       */

      samples_to_read = 0;

      do
        {
          /* Figure out if there are samples left to read */

          samples_to_read = ioctl(adcfd, ANIOC_SAMPLES_ON_READ, 0);
          if (samples_to_read < 0)
            {
              syslog(LOG_ERR | LOG_USER,
                     "Couldn't get '%s' number of samples to read: %d\n",
                     adcpath, errno);
              break;
            }
          else if (samples_to_read == 0)
            {
              break; /* Nothing to read */
            }

          /* Read the sample */

          err = adc_read_sample(adcfd, &sample);
          if (err)
            {
              syslog(LOG_ERR | LOG_USER, "Couldn't read '%s': %d\n", adcpath,
                     errno);
              continue;
            }

          /* Check if the right topic exists for that channel ID. If not,
           * create the topic.
           */

          sel_topic = get_topic_by_chan(topics, num_chans, sample.am_channel);
          if (sel_topic == NULL)
            {
              /* Something is very wrong, we have to die! */

              syslog(LOG_ERR | LOG_USER,
                     "Got unexpected channel that exceeds ADC reported "
                     "channel count!\n");
              ret = EXIT_FAILURE;
              goto close_topics;
            }
          else if (sel_topic->devno < 0)
            {
              /* We need to spin up a topic for this */

              err = topic_create(sel_topic, devno_base, sample.am_channel);
              if (err)
                {
                  /* If we can't advertise data then this is also unexpected
                   * behaviour, let's quit.
                   */

                  syslog(LOG_ERR | LOG_USER,
                         "Couldn't advertise sensor_voltage for channel %d\n",
                         sample.am_channel);
                  ret = EXIT_FAILURE;
                  goto close_topics;
                }
              else if (sel_topic->channo != sample.am_channel)
                {
                  /* This is an unexpected case. We expected to have to
                   * allocate or to get the right channel.
                   */

                  syslog(
                      LOG_ERR | LOG_USER,
                      "Couldn't find sensor_voltage topic for channel %d\n",
                      sample.am_channel);
                  ret = EXIT_FAILURE;
                  goto close_topics;
                }
            }

          /* We have the right topic selected! Send the sample there. */

          err = publish_sample(sel_topic, &sample, max_voltage, adc_max);
          if (err)
            {
              continue; /* Error logged within function */
            }
        }
      while (samples_to_read > 0);

      /* We had an error checking the samples left to read */

      if (samples_to_read < 0) continue;

      /* Wait the measurement period */

      sleep(period);
    }

close_topics:
  for (int i = 0; i < num_chans; i++)
    {
      if (topics[i].fd > 0)
        {
          close(topics[i].fd);
        }
    }

  if (topics != NULL) free(topics);

clean_fd:
  if (adcfd > 0) close(adcfd);

  return ret;
}
