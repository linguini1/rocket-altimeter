/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
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
#define FEVENT_IDX (2)

/* Array length helper */

#define array_len(arr) (sizeof(arr) / sizeof(arr[0]))

/* Program already knows about necessary uORB data types */

ORB_DECLARE(sensor_baro);
ORB_DECLARE(deploy_event);
ORB_DECLARE(flight_event);

/* Log file permissions */

#define LOG_PERMS 0666

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct logger_t
{
  int filefd;                          /* Logging file fd */
  int xtraflags;                       /* Extra open flags */
  const char fnamefmt[32];             /* File name format string */
  FAR const struct orb_metadata *meta; /* uORB metadata */
  void *orbbuf;                        /* Buffer for reading in uORB data */
  size_t buflen;                       /* In bytes */
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static char fname[64]; /* File name buffer */

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: get_flightno
 *
 * Description:
 *   Get the current flight number so that new flight logs aren't overwritten.
 *
 * Input Parameters:
 *
 * Returned Value:
 *   Flight number >= 0 on success, negated errno on error.
 *
 ****************************************************************************/

static int get_flightno(void)
{
  /* TODO */
  return 0;
}

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
  int flightno;
  unsigned fdirlen;
  struct sensor_baro baro_data[10];
  struct deploy_event dep_event;
  struct flight_event f_event;
  struct pollfd pollfds[3];

  /* Initialize required data up front */

  struct logger_t loggers[3] = {
      [BARO_IDX] =
          {
              .fnamefmt = "baro_%d.log",
              .xtraflags = 0,
              .orbbuf = baro_data,
              .buflen = sizeof(baro_data),
              .meta = ORB_ID(sensor_baro),
          },
      [DEP_IDX] =
          {
              .fnamefmt = "deployment_%d.log",
              .xtraflags = O_SYNC, /* Sync always due to infrequent events */
              .orbbuf = &dep_event,
              .buflen = sizeof(dep_event),
              .meta = ORB_ID(deploy_event),
          },
      [FEVENT_IDX] =
          {
              .fnamefmt = "flight_event_%d.log",
              .xtraflags = O_SYNC, /* Sync always due to infrequent events */
              .orbbuf = &f_event,
              .buflen = sizeof(f_event),
              .meta = ORB_ID(flight_event),
          },
  };

  DEBUGASSERT(array_len(pollfds) == array_len(loggers));

  /* Initialize polling settings */

  for (unsigned i = 0; i < array_len(loggers); i++)
    {
      pollfds[i].events = POLLIN;
      pollfds[i].revents = 0;
      pollfds[i].fd = -1;
      loggers[i].filefd = -1;
    }

  /* Subscribe to uORB topics.
   * TODO: we should know how to differentiate between the fake barometer and
   * the real one.
   */

  for (int i = 0; i < array_len(loggers); i++)
    {
      pollfds[i].fd = orb_subscribe_multi(loggers[i].meta, 0);
      if (pollfds[i].fd < 0)
        {
          syslog(LOG_ERR | LOG_USER, "Could not subscribe to topic: %s\n",
                 loggers[i].meta->o_name);
          ret = EXIT_FAILURE;
          goto cleanup_orb;
        }
    }

  /* Logging directory in the file path will be either the default directory
   * or overridden by the user provided directory.
   */

  strncpy(fname, argc > 1 ? argv[1] : CONFIG_ROCKETALT_LOGGER_LOGDIR,
          sizeof(fname));
  fdirlen = strlen(fname); /* Length of the directory name */
  fname[fdirlen++] = '/';  /* Trailing slash */
  flightno = get_flightno();

  /* Open logging files */

  for (int i = 0; i < array_len(loggers); i++)
    {
      /* Concatenate logdir path with file name for each logger type */

      snprintf(&fname[fdirlen], sizeof(fname) - fdirlen, loggers[i].fnamefmt,
               flightno);

      /* Open the file for appending with extra flags */

      loggers[i].filefd =
          open(fname, O_APPEND | O_WRONLY | O_CREAT | loggers[i].xtraflags,
               LOG_PERMS);
      if (loggers[i].filefd < 0)
        {
          syslog(LOG_ERR | LOG_USER, "Couldn't open log file %s: %d\n.",
                 fname, errno);
          goto cleanup_files;
        }
      syslog(LOG_INFO | LOG_USER, "Logging %s data to '%s'\n",
             loggers[i].meta->o_name, fname);
    }

  /* Forever take in data and log it */

  for (;;)
    {
      err = poll(pollfds, array_len(pollfds), -1);
      if (err <= 0)
        {
          syslog(LOG_ERR | LOG_USER, "Failed to poll: %d\n", errno);
          continue; /* Try again */
        }

      for (unsigned i = 0; i < array_len(loggers); i++)
        {
          if (pollfds[i].revents & POLLIN)
            {
              err = orb_copy_multi(pollfds[i].fd, loggers[i].orbbuf,
                                   loggers[i].buflen);
              pollfds[i].revents = 0; /* Event handled */

              if (err < 0)
                {
                  syslog(LOG_ERR | LOG_USER,
                         "Couldn't read data from %s: %d\n",
                         loggers[i].meta->o_name, errno);
                  continue;
                }

              /* Write data out to the file since no error copying occurred.
               */

              err = write(loggers[i].filefd, loggers[i].orbbuf, err);
              if (err < 0)
                {
                  syslog(LOG_ERR | LOG_USER,
                         "Couldn't log data from %s: %d\n",
                         loggers[i].meta->o_name, errno);
                  continue;
                }

              /* Barometer data should be sync'd according to a user-defined
               * byte count. TODO
               */
            }
        }
    }

cleanup_files:
  for (int i = 0; i < array_len(loggers); i++)
    {
      close(loggers[i].filefd);
    }
cleanup_orb:
  for (int i = 0; i < array_len(pollfds); i++)
    {
      orb_unsubscribe(pollfds[i].fd);
    }

  return ret;
}
