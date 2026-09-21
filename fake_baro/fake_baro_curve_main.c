/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <fcntl.h>
#include <getopt.h>
#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include <sys/types.h>
#include <syslog.h>
#include <unistd.h>

#include <nuttx/sensors/sensor.h>

#include <uORB/uORB.h>

#include <sensor/baro.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define DAMPING_RATIO (0.5f)

#define LANDED_DURATION (10.0f) /* Amount of time to simulate landing */

/* Default flight profile */

#define DEFAULT_APOGEE (10000.0f)
#define DEFAULT_APOGEE_TIME (25.0f)
#define DEFAULT_DESCENT_VEL (100.0f)
#define DEFAULT_DT (0.0625f) /* 16Hz measurement rate */

/* Pressure at sea-level in millibars */

#define SEA_PRESSURE (1013.25f)

/* Temperature at sea-level in Kelvin */

#define SEA_TEMPERATURE (288.15f)

/* Temperature lapse rate in Kelvins per meter */

#define LAPSE_RATE (0.0065f)

/* The universal gas constant */

#define GAS_CONSTANT (8.31432f)

/* Acceleration due to gravity on Earth (m/s^2) */

#define GRAVITY (9.80665f)

/* Constant for the mean molar mass of atmospheric gases */

#define MOLAR_MASS (0.0289644f)

/* Celsius to Kelvin conversion factor */

#define CELSIUS_TO_KELVIN (273.0f)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* Curve model Desmos experiment: https://www.desmos.com/calculator/csjn2zfltw
 */

/****************************************************************************
 * Name: curve_ascent
 *
 * Description:
 *   Describes the ascent curve of a rocket.
 *
 * Input Parameters:
 *   t - The current time in seconds (0 <= t <= t_ap)
 *   t_ap - The time that apogee occurs in seconds
 *   a_ap - The apogee altitude in meters
 *
 * Returned Value:
 *   The altitude that the rocket would achieve at time `t` in ascent.
 *
 ****************************************************************************/

static float curve_ascent(float t, float t_ap, float a_ap)
{
  float r = t / t_ap;
  return a_ap * (10.0f * pow(r, 3) - 15.0f * pow(r, 4) + 6.0f * pow(r, 5));
}

/****************************************************************************
 * Name: curve_descent
 *
 * Description:
 *   Describes the descent curve of a rocket.
 *
 * Input Parameters:
 *   t - The current time in seconds (t_ap <= t)
 *   t_ap - The time that apogee occurs in seconds
 *   a_ap - The apogee altitude in meters
 *   v_d - The steady state terminal velocity under parachute in m/s
 *
 * Returned Value:
 *   The altitude the rocket would achieve at time `t` in descent.
 *
 ****************************************************************************/

static float curve_descent(float t, float t_ap, float a_ap, float v_d)
{
  float diff = t - t_ap;
  return a_ap - v_d * diff * (1.0f - exp(-diff / DAMPING_RATIO));
}

/****************************************************************************
 * Name: publish_measurement
 *
 * Description:
 *   Publishes a barometric pressure measurement from the provided altitude.
 *
 * Input Parameters:
 *   fd - The file descriptor of the barometer data topic
 *   altitude - The altitude to publish the pressure measurement for
 *
 * Returned Value:
 *   0 on success, error code on failure.
 *
 ****************************************************************************/

static int publish_measurement(int fd, float altitude)
{
  int err;
  struct sensor_baro data;

  /* NOTE: h_0 (reference altitude) is 0.
   *
   * NOTE: Temperature is computed first based on altitude, which informs the
   * pressure.
   */

  data.timestamp = orb_absolute_time();

  if (altitude <= 11000.0f)
    {
      /* Use the troposphere model */

      data.temperature = SEA_TEMPERATURE - LAPSE_RATE * altitude; /* Kelvin */
    }
  else
    {
      /* Constant stratosphere temperature */

      data.temperature = 216.65; /* Kelvin */
    }

  data.pressure = SEA_PRESSURE * exp(-GRAVITY * MOLAR_MASS * altitude /
                                     (GAS_CONSTANT * data.temperature));
  data.temperature -= CELSIUS_TO_KELVIN; /* Convert back to Celsius */

  /* Compute pressure backwards from the altitude */

  err = orb_publish(ORB_ID(sensor_baro), fd, &data);
  if (err < 0)
    {
      syslog(LOG_ERR | LOG_USER, "Couldn't publish barometer data: %d\n",
             errno);
      return errno;
    }

  return err;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int main(int argc, char **argv)
{
  int baro_fd;
  int c;
  int devno = 0; /* Default of 0 */
  float apogee = DEFAULT_APOGEE;
  float t_ap = DEFAULT_APOGEE_TIME;
  float v_d = DEFAULT_DESCENT_VEL;
  float dt = DEFAULT_DT;
  float t = 0.0f;               /* Start at t = 0 */
  float altitude = 0.0f;        /* Start on the ground */
  float landed_duration = 0.0f; /* Duration in landing */

  /* Parse command line arguments */

  while ((c = getopt(argc, argv, ":n:a:t:v:r:")) != -1)
    {
      switch (c)
        {
        case 'n':
          devno = atoi(optarg);
          break;

        case 'a':
          apogee = strtof(optarg, NULL);
          break;

        case 't':
          t_ap = strtof(optarg, NULL);
          break;

        case 'v':
          v_d = strtof(optarg, NULL);
          break;

        case 'r':
          /* Convert measurement rate to period */

          dt = 1.0f / strtof(optarg, NULL);
          break;

        case ':':
          syslog(LOG_ERR | LOG_USER, "Option -%c requires an argument.\n",
                 optopt);
          return EXIT_FAILURE;

        case '?':
          syslog(LOG_ERR | LOG_USER, "Unknown option '-%c'.\n", optopt);
          break; /* Don't exit, parse other options */

        default:
          syslog(LOG_ERR | LOG_USER, "Usage: fake_baro [-n devno]\n");
          return EXIT_FAILURE;
        }
    }

  /* Set up sensor_baro uORB topic */

  baro_fd = orb_advertise_multi_queue(ORB_ID(sensor_baro), NULL, &devno,
                                      CONFIG_ROCKETALT_FAKE_BARO_QLEN);
  if (baro_fd < 0)
    {
      syslog(LOG_ERR | LOG_USER, "Couldn't advertise sensor_baro%d: %d\n",
             devno, errno);
      return EXIT_FAILURE;
    }

  syslog(LOG_INFO | LOG_USER, "sensor_baro%d advertised\n", devno);

  /* Start publishing to the topic from the curve! */

  syslog(LOG_INFO | LOG_USER,
         "Using flight profile: apogee=%.2f m, t_apogee=%.2f s, v_desc=%.2f "
         "m/s\n",
         apogee, t_ap, v_d);

  for (;;)
    {
      /* Calculate our altitude using the curve */

      if (t <= t_ap)
        {
          altitude = curve_ascent(t, t_ap, apogee);
        }
      else if (t_ap <= t)
        {
          altitude = curve_descent(t, t_ap, apogee, v_d);
        }

      /* If we have landed (altitude is back at 0), maintain landing for
       * at least 10 seconds before re-starting the curve. This allows some
       * time for the landing detection logic to be robustly sure of landing,
       * and also lets the user actually process what's happening.
       */

      if (altitude <= 0.0f && landed_duration < LANDED_DURATION)
        {
          landed_duration += dt;
          altitude = 0.0f;
        }

      /* Publish the barometric pressure measurement */

      publish_measurement(baro_fd, altitude);

      /* Update our current time to the next time step */

      t += dt;

      /* If we've already done the full landing duration, we can reset the
       * curve.
       */

      if (landed_duration >= LANDED_DURATION)
        {
          landed_duration = 0.0f;
          altitude = 0.0f;
          t = 0.0f;
        }

      /* Sleep an appropriate amount of simulated time */

      usleep(dt * 1000000);
    }

  syslog(LOG_ERR | LOG_USER,
         "Could not start sensor_baro%d using curve: not implemented\n",
         devno);

  orb_unadvertise(baro_fd);
  return EXIT_FAILURE;
}
