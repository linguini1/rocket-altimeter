/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <stdlib.h>
#include <sys/types.h>
#include <syslog.h>

#include <uORB/uORB.h>

#include "../common/common.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Indexes of topic file descriptors */

#define BARO_IDX (0)
#define DEP_IDX (1)

/* Array length helper */

#define array_len(arr) (sizeof(arr) / sizeof(arr[0]))

/* Program already knows about barometer data */

ORB_DECLARE(sensor_baro);

/* Program already knows about deployment events */

ORB_DECLARE(deploy_event);

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
 * Public Functions
 ****************************************************************************/

int main(int argc, char **argv)
{
  int err;
  int ret;
  int barolog_fd;
  int deplog_fd;
  struct pollfd fds[2];
  struct sensor_baro baro_data[10];
  struct deploy_event dep_event;

  if (argc < 3)
    {
      syslog(LOG_ERR | LOG_USER,
             "Usage: logger <path/to/baro.log> <path/to/deploy.log>");
      return EXIT_FAILURE;
    }

  /* Subscribe to barometer topic.
   * TODO: we should know how to differentiate between the fake barometer and
   * the real one.
   */

  fds[BARO_IDX].fd = orb_subscribe_multi(ORB_ID(sensor_baro), 0);
  if (fds[BARO_IDX].fd < 0)
    {
      syslog(LOG_ERR | LOG_USER,
             "Could not subscribe to barometer topic: %d\n", errno);
      ret = EXIT_FAILURE;
      goto ret_err;
    }

  /* Subscribe to deployment topic. */

  fds[DEP_IDX].fd = orb_subscribe_multi(ORB_ID(deploy_event), 0);
  if (fds[DEP_IDX].fd < 0)
    {
      syslog(LOG_ERR | LOG_USER,
             "Could not subscribe to deployment event topic: %d\n", errno);
      ret = EXIT_FAILURE;
      goto cleanup_baro;
    }

  /* Open logging files TODO use append to ensure no overwrite */

  barolog_fd = open(argv[1], O_WRONLY);
  if (barolog_fd < 0)
    {
      syslog(LOG_ERR | LOG_USER, "Couldn't open barometer log file %s: %d\n.",
             argv[1], errno);
      goto cleanup_dep;
    }

  deplog_fd = open(argv[2], O_WRONLY);
  if (barolog_fd < 0)
    {
      syslog(LOG_ERR | LOG_USER,
             "Couldn't open deployment log file %s: %d\n.", argv[1], errno);
      goto cleanup_barolog;
    }

  /* Set up polling */

  for (unsigned i = 0; i < array_len(fds); i++)
    {
      fds[i].events = POLLIN;
      fds[i].revents = 0;
    }

  /* Forever take in data and log it */

  for (;;)
    {
      err = poll(fds, 2, -1);
      if (err <= 0)
        {
          syslog(LOG_ERR | LOG_USER, "Failed to poll: %d\n", errno);
          continue; /* Try again */
        }

      for (unsigned i = 0; i < array_len(fds); i++)
        {
          if (fds[i].revents & POLLIN)
            {
              switch (i)
                {
                case BARO_IDX:
                  err = orb_copy_multi(fds[i].fd, &baro_data,
                                       sizeof(baro_data));
                  break;
                case DEP_IDX:
                  err = orb_copy_multi(fds[i].fd, &dep_event,
                                       sizeof(dep_event));
                  break;
                }

              fds[i].revents = 0; /* Event handled */

              if (err < 0)
                {
                  syslog(LOG_ERR | LOG_USER, "Couldn't read data: %d\n",
                         errno);
                  continue;
                }

              /* Write data out to the file since no error copying occurred.
               */

              switch (i)
                {
                case BARO_IDX:
                  /* `err` contains size of data copied to baro data buffer */

                  err = write(barolog_fd, &baro_data, err);
                  if (err < 0)
                    {
                      syslog(LOG_ERR | LOG_USER,
                             "Couldn't log barometer data.\n");
                    }
                  break;
                case DEP_IDX:
                  err = write(deplog_fd, &dep_event, sizeof(dep_event));
                  if (err < 0)
                    {
                      syslog(LOG_ERR | LOG_USER,
                             "Couldn't log deployment event.\n");
                    }
                  break;
                }
            }
        }
    }

  close(deplog_fd);
cleanup_barolog:
  close(barolog_fd);
cleanup_dep:
  orb_unsubscribe(fds[DEP_IDX].fd);
cleanup_baro:
  orb_unsubscribe(fds[BARO_IDX].fd);
ret_err:
  return ret;
}
