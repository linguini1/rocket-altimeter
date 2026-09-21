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
#include <sys/types.h>
#include <syslog.h>

#include <nuttx/analog/adc.h>
#include <nuttx/analog/ioctl.h>

#include <uORB/uORB.h>

#include "sensor/battery.h"

#include "../../common/common.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* ADC measurement conversion */

#define measure_to_volts(r)                                                  \
  ((r) * CONFIG_ROCKETALT_BATMON_BATMAX /                                    \
   CONFIG_ROCKETALT_BATMON_ADCRESOLUTION)

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int main(int argc, char **argv)
{
  int c;
  int err;
  int devno = 0;
  int ret = EXIT_FAILURE;
  char *adcpath = NULL;
  int bat_fd;
  int adc_fd;
  uint8_t channo;
  ssize_t bread;
  struct adc_msg_s adc_data;
  struct sensor_battery batdata;

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
                 "Usage: adcbat [-n devno] devpath channo\n");
          return EXIT_FAILURE;
        }
    }

  if (argc <= optind)
    {
      syslog(LOG_ERR | LOG_USER, "Expected ADC device path.\n");
      return EXIT_FAILURE;
    }

  adcpath = argv[optind++];

  if (argc <= optind)
    {
      syslog(LOG_ERR | LOG_USER, "Expected ADC channel number.\n");
      return EXIT_FAILURE;
    }

  channo = atoi(argv[optind]); /* Parse channel number */

  /* Set up battery topic for publishing */

  bat_fd = orb_advertise_multi_queue(ORB_ID(sensor_battery), NULL, &devno,
                                     CONFIG_ROCKETALT_BATMON_QLEN);
  if (bat_fd < 0)
    {
      syslog(LOG_ERR | LOG_USER, "Could not advertise sensor_battery%d: %d\n",
             devno, errno);
      return EXIT_FAILURE;
    }

  syslog(LOG_INFO | LOG_USER, "sensor_battery%d advertised.\n", devno);

  /* Open ADC device */

  adc_fd = open(adcpath, O_RDONLY);
  if (adc_fd < 0)
    {
      syslog(LOG_ERR | LOG_USER, "Could not open ADC device %s: %d\n",
             adcpath, errno);
      ret = EXIT_FAILURE;
      goto clean_bat;
    }

  /* Forever convert ADC measurements to uORB output */

  for (;;)
    {
#ifdef CONFIG_ROCKETALT_BATMON_SWTRIG
      /* Some ADCs need a conversion to be triggered manually. */

      err = ioctl(adc_fd, ANIOC_TRIGGER, 0);
      if (err < 0)
        {
          syslog(LOG_ERR | LOG_USER, "ADC trigger ioctl failed: %d\n", errno);
          continue;
        }
#endif

      /* TODO: Need read to respect `channo` */

      bread = read(adc_fd, &adc_data, sizeof(adc_data));
      if (bread <= 0)
        {
          syslog(LOG_ERR | LOG_USER, "Failed to read ADC: %d\n", errno);
          continue; /* Try again */
        }

      batdata.timestamp = orb_absolute_time();
      batdata.voltage = (float)measure_to_volts(adc_data.am_data) / 1000.0f;
      batdata.level = 0; /* TODO */

      err = orb_publish(ORB_ID(sensor_battery), bat_fd, &batdata);
      if (err)
        {
          syslog(LOG_ERR | LOG_USER,
                 "Couldn't publish to sensor_battery%d: %d\n", devno, errno);
          continue;
        }

#ifdef CONFIG_ROCKETALT_BATMON_SWTRIG
      /* If we're manually triggering the ADC, make sure we only do this as
       * often as the user configured.
       */

      sleep(CONFIG_ROCKETALT_BATMON_PERIOD);
#endif
    }

  close(adc_fd);
clean_bat:
  orb_unadvertise(bat_fd);
  return ret;
}
