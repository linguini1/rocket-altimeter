/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <errno.h>
#include <fcntl.h>
#include <getopt.h>
#include <math.h>
#include <poll.h>
#include <stdint.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <syslog.h>

#include <nuttx/analog/adc.h>
#include <nuttx/analog/ioctl.h>

#include <uORB/uORB.h>

#include "sensor/battery.h"
#include <sensor/voltage.h>

#include "../../common/common.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Battery max voltages based on chemistry */

#ifdef CONFIG_ROCKETALT_BATMON_CHEM_UNK
#define MAX_VOLTAGE (CONFIG_ROCKETALT_BATMON_BATMAX)
#endif /* CONFIG_ROCKETALT_BATMON_CHEM_UNK */

#ifdef CONFIG_ROCKETALT_BATMON_CHEM_LIPO
#define MAX_VOLTAGE (4200)
#endif /* CONFIG_ROCKETALT_BATMON_CHEM_LIPO */

#ifdef CONFIG_ROCKETALT_BATMON_CHEM_LIION
#define MAX_VOLTAGE (4200)
#endif /* CONFIG_ROCKETALT_BATMON_CHEM_LIION */

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#ifdef CONFIG_ROCKETALT_BATMON_CHEM_UNK

/****************************************************************************
 * Name: level_from_charge_curve
 *
 * Description:
 *   Returns the battery level as a percentage given the battery voltage.
 *   For unknown chemistry.
 *
 * Input Parameters:
 *   voltage - The battery voltage in Volts
 *
 * Returned Value:
 *   The battery level as a percentage.
 *
 ****************************************************************************/

static uint8_t level_from_charge_curve(float voltage)
{
  uint8_t level;

  /* We'll just perform a linear mapping */

  level = 100 * ((voltage * 1000.0f) / (float)MAX_VOLTAGE);
  return level;
}

#endif /* CONFIG_ROCKETALT_BATMON_CHEM_UNK */

#ifdef CONFIG_ROCKETALT_BATMON_CHEM_LIPO

/****************************************************************************
 * Name: level_from_charge_curve
 *
 * Description:
 *   Returns the battery level as a percentage given the battery voltage.
 *   For LiPo batteries.
 *   Taken from: https://github.com/G6EJD/LiPo_Battery_Capacity_Estimator
 *
 * Input Parameters:
 *   voltage - The battery voltage in Volts
 *
 * Returned Value:
 *   The battery level as a percentage.
 *
 ****************************************************************************/

static uint8_t level_from_charge_curve(float voltage)
{
  float level;

  if (voltage >= 4.2f)
    {
      level = 100.0f;
    }
  else if (voltage < 3.5f)
    {
      level = 0;
    }
  else
    {
      level = 2808.3808f * powf(voltage, 4) - 43560.9157f * powf(voltage, 3) +
              252848.5888f * powf(voltage, 2) - 650767.4615f * voltage +
              626532.5703f;
    }

  return level;
}

#endif /* CONFIG_ROCKETALT_BATMON_CHEM_LIPO */

#ifdef CONFIG_ROCKETALT_BATMON_CHEM_LIION

/****************************************************************************
 * Name: level_from_charge_curve
 *
 * Description:
 *   Returns the battery level as a percentage given the battery voltage.
 *   For Li-ion battery.
 *
 * Input Parameters:
 *   voltage - The battery voltage in Volts
 *
 * Returned Value:
 *   The battery level as a percentage.
 *
 ****************************************************************************/

static uint8_t level_from_charge_curve(float voltage)
{
  /* TODO */

#error "Unimplemented."
  return 0;
}

#endif /* CONFIG_ROCKETALT_BATMON_CHEM_LIION */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int main(int argc, char **argv)
{
  int c;
  int err;
  int devno = 0;
  int ret = EXIT_FAILURE;
  int bat_fd;
  struct pollfd pfd;
  int topicno;
  struct sensor_voltage voltage;
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
          syslog(LOG_ERR | LOG_USER, "Usage: adcbat [-n devno] topicno\n");
          return EXIT_FAILURE;
        }
    }

  if (argc <= optind)
    {
      syslog(LOG_ERR | LOG_USER, "Expected voltage topic device number.\n");
      return EXIT_FAILURE;
    }

  topicno = atoi(argv[optind]);

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

  /* Subscribe to voltage topic */

  pfd.fd = orb_subscribe_multi(ORB_ID(sensor_voltage), topicno);
  if (pfd.fd < 0)
    {
      syslog(LOG_ERR | LOG_USER,
             "Could not subscribe to sensor_voltage%d: %d\n", topicno, errno);
      ret = EXIT_FAILURE;
      goto clean_bat;
    }

  pfd.revents = 0;
  pfd.events = POLLIN;

  /* Forever convert voltage measurements to uORB output */

  for (;;)
    {
      err = poll(&pfd, 1, -1);
      if (err <= 0)
        {
          syslog(LOG_ERR | LOG_USER, "Failed to poll sensor_voltage%d: %d\n",
                 topicno, errno);
          continue; /* Try again */
        }

      /* Get data */

      err = orb_copy(ORB_ID(sensor_voltage), pfd.fd, &voltage);
      if (err)
        {
          if (errno != ENODATA)
            {
              syslog(LOG_ERR | LOG_USER, "Couldn't get barometer data: %d\n",
                     errno);
            }
          continue;
        }

      batdata.timestamp = voltage.timestamp;
      batdata.voltage = voltage.voltage;
      batdata.level = level_from_charge_curve(voltage.voltage);

      err = orb_publish(ORB_ID(sensor_battery), bat_fd, &batdata);
      if (err)
        {
          syslog(LOG_ERR | LOG_USER,
                 "Couldn't publish to sensor_battery%d: %d\n", devno, errno);
          continue;
        }
    }

  orb_unsubscribe(pfd.fd);
clean_bat:
  orb_unadvertise(bat_fd);
  return ret;
}
