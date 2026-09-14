/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <errno.h>
#include <fcntl.h>
#include <stdint.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <syslog.h>

#include <nuttx/analog/adc.h>
#include <nuttx/analog/ioctl.h>

#include <uORB/uORB.h>

#include "../common/common.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Program already knows about voltage data */

ORB_DECLARE(sensor_voltage);

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
  int err;
  int bat_fd;
  int adc_fd;
  ssize_t bread;
  struct adc_msg_s adc_data;
  struct sensor_voltage volt;

  /* Set up battery topic for publishing */

  bat_fd = orb_advertise_multi_queue(ORB_ID(sensor_voltage), NULL, NULL,
                                     CONFIG_ROCKETALT_BATMON_VOLTAGE_QLEN);
  if (bat_fd < 0)
    {
      syslog(LOG_ERR | LOG_USER,
             "Could not advertise sensor_voltage topic: %d\n", errno);
      return EXIT_FAILURE;
    }

  syslog(LOG_INFO | LOG_USER, "sensor_voltage topic advertised.\n");

  /* Open ADC device */

  adc_fd = open(CONFIG_ROCKETALT_BATMON_ADCPATH, O_RDONLY);
  if (adc_fd < 0)
    {
      syslog(LOG_ERR | LOG_USER,
             "Could not open ADC device '" CONFIG_ROCKETALT_BATMON_ADCPATH
             "' %d\n",
             errno);
      orb_unadvertise(bat_fd);
      return EXIT_FAILURE;
    }

  /* Forever convert ADC measurements to voltage uORB output */

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

      bread = read(adc_fd, &adc_data, sizeof(adc_data));
      if (bread <= 0)
        {
          syslog(LOG_ERR | LOG_USER, "Failed to read ADC: %d\n", errno);
          continue; /* Try again */
        }

      volt.timestamp = orb_absolute_time();
      volt.voltage = (float)measure_to_volts(adc_data.am_data) / 1000.0f;

      err = orb_publish(ORB_ID(sensor_voltage), bat_fd, &volt);
      if (err)
        {
          syslog(LOG_ERR | LOG_USER, "Couldn't publish voltage data: %d\n",
                 errno);
          continue;
        }

#ifdef CONFIG_ROCKETALT_BATMON_SWTRIG
      /* If we're manually triggering the ADC, make sure we only do this as
       * often as the user configured.
       */

      sleep(CONFIG_ROCKETALT_BATMON_PERIOD);
#endif
    }

  orb_unadvertise(bat_fd);
  close(adc_fd);
  return EXIT_FAILURE;
}
