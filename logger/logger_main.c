/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <errno.h>
#include <fcntl.h>
#include <getopt.h>
#include <poll.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <syslog.h>

#include <uORB/uORB.h>

#include <sensor/baro.h>
#include "sensor/flight_event.h"
#include "sensor/deploy_event.h"

#include "../common/common.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Indexes of topic file descriptors */

#define BARO_IDX (0)
#define DEP_IDX (1)
#define FEVENT_IDX (2)

/* Log file permissions */

#define LOG_PERMS (0666)

/* File name maximum length */

#define MAX_FNAME (64)

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct logger_t
{
  FAR const char *fnamefmt;            /* File name format string */
  FAR const struct orb_metadata *meta; /* uORB metadata */
  void *orbbuf;                        /* Buffer for reading in uORB data */
  size_t buflen;                       /* `orbbuf` length in bytes */
  int devno;                           /* uORB device instance number */
  int filefd;                          /* Logging file fd */
  int xtraflags;                       /* Extra open flags */
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static char g_fname[MAX_FNAME]; /* File name buffer */

/* Describes the data being logged */

static struct logger_t g_loggers[3] = {
    [BARO_IDX] =
        {
            .meta = ORB_ID(sensor_baro),
            .devno = 0,
            .fnamefmt = "baro_%d.log",
            .xtraflags = 0,
        },
    [DEP_IDX] =
        {
            .meta = ORB_ID(deploy_event),
            .devno = 0,
            .fnamefmt = "dep_%d.log",
            .xtraflags = O_SYNC, /* Sync always due to infrequent events */
        },
    [FEVENT_IDX] =
        {
            .meta = ORB_ID(flight_event),
            .devno = 0,
            .fnamefmt = "f_event_%d.log",
            .xtraflags = O_SYNC, /* Sync always due to infrequent events */
        },
};

/* File descriptors for the topics being polled */

static struct pollfd g_pollfds[3];

static_assert(array_len(g_pollfds) == array_len(g_loggers),
              "Mismatched array lengths.");

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
  int c;
  int err;
  int ret;
  int flightno;
  unsigned fdirlen;
  const char *logdir = NULL;
  struct sensor_baro baro_data[10];
  struct deploy_event dep_event;
  struct flight_event f_event;

  /* Parse command line arguments */

  while ((c = getopt(argc, argv, ":b:d:f:")) != -1)
    {
      switch (c)
        {
        case 'b':
          g_loggers[BARO_IDX].devno = atoi(optarg);
          break;

        case 'd':
          g_loggers[DEP_IDX].devno = atoi(optarg);
          break;

        case 'f':
          g_loggers[FEVENT_IDX].devno = atoi(optarg);
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
                 "Usage: logger [-b devno] [-d devno] [-f devno] [logdir]\n");
          return EXIT_FAILURE;
        }
    }

  /* Logging directory in the file path will be either the default directory
   * or overridden by the user provided directory.
   */

  if (argc >= optind)
    {
      logdir = argv[optind]; /* User passed logging directory */
    }

  /* Initialize loggers */

  g_loggers[BARO_IDX].orbbuf = baro_data;
  g_loggers[BARO_IDX].buflen = sizeof(baro_data);

  g_loggers[DEP_IDX].orbbuf = &dep_event;
  g_loggers[DEP_IDX].buflen = sizeof(dep_event);

  g_loggers[FEVENT_IDX].orbbuf = &f_event;
  g_loggers[FEVENT_IDX].buflen = sizeof(f_event);

  /* Initialize polling settings */

  for (unsigned i = 0; i < array_len(g_loggers); i++)
    {
      g_pollfds[i].events = POLLIN;
      g_pollfds[i].revents = 0;
      g_pollfds[i].fd = -1;
      g_loggers[i].filefd = -1;
    }

  /* Subscribe to uORB topics.
   * TODO: we should know how to differentiate between the fake barometer and
   * the real one.
   */

  for (int i = 0; i < array_len(g_loggers); i++)
    {
      g_pollfds[i].fd =
          orb_subscribe_multi(g_loggers[i].meta, g_loggers[i].devno);
      if (g_pollfds[i].fd < 0)
        {
          syslog(LOG_ERR | LOG_USER, "Could not subscribe to %s%d: %d\n",
                 g_loggers[i].meta->o_name, g_loggers[i].devno, errno);
          ret = EXIT_FAILURE;
          goto cleanup_orb;
        }

      syslog(LOG_INFO | LOG_USER, "Logger subscribed to %s%d\n",
             g_loggers[i].meta->o_name, g_loggers[i].devno);
    }

  /* Move logging directory path into file path name buffer */

  strncpy(g_fname, logdir == NULL ? CONFIG_ROCKETALT_LOGGER_LOGDIR : logdir,
          sizeof(g_fname));
  fdirlen = strlen(g_fname); /* Length of the directory name */
  g_fname[fdirlen++] = '/';  /* Trailing slash */
  flightno = get_flightno();

  /* Open logging files */

  for (int i = 0; i < array_len(g_loggers); i++)
    {
      /* Concatenate logdir path with file name for each logger type */

      snprintf(&g_fname[fdirlen], sizeof(g_fname) - fdirlen,
               g_loggers[i].fnamefmt, flightno);

      /* Open the file for appending with extra flags */

      g_loggers[i].filefd = open(
          g_fname, O_APPEND | O_WRONLY | O_CREAT | g_loggers[i].xtraflags,
          LOG_PERMS);
      if (g_loggers[i].filefd < 0)
        {
          syslog(LOG_ERR | LOG_USER, "Couldn't open log file %s: %d\n.",
                 g_fname, errno);
          goto cleanup_files;
        }

      syslog(LOG_INFO | LOG_USER, "Logging %s%d data to '%s'\n",
             g_loggers[i].meta->o_name, g_loggers[i].devno, g_fname);
    }

  /* Forever take in data and log it */

  for (;;)
    {
      err = poll(g_pollfds, array_len(g_pollfds), -1);
      if (err <= 0)
        {
          syslog(LOG_ERR | LOG_USER, "Failed to poll: %d\n", errno);
          continue; /* Try again */
        }

      for (unsigned i = 0; i < array_len(g_loggers); i++)
        {
          if (g_pollfds[i].revents & POLLIN)
            {
              err = orb_copy_multi(g_pollfds[i].fd, g_loggers[i].orbbuf,
                                   g_loggers[i].buflen);
              g_pollfds[i].revents = 0; /* Event handled */

              if (err < 0)
                {
                  syslog(LOG_ERR | LOG_USER,
                         "Couldn't read data from %s%d: %d\n",
                         g_loggers[i].meta->o_name, g_loggers[i].devno,
                         errno);
                  continue;
                }

              /* Write data out to the file since no error copying occurred.
               */

              err = write(g_loggers[i].filefd, g_loggers[i].orbbuf, err);
              if (err < 0)
                {
                  syslog(
                      LOG_ERR | LOG_USER, "Couldn't log data from %s%d: %d\n",
                      g_loggers[i].meta->o_name, g_loggers[i].devno, errno);
                  continue;
                }

              /* Barometer data should be sync'd according to a user-defined
               * byte count. TODO
               */
            }
        }
    }

cleanup_files:
  for (int i = 0; i < array_len(g_loggers); i++)
    {
      if (g_loggers[i].filefd > 0)
        {
          close(g_loggers[i].filefd);
        }
    }

cleanup_orb:
  for (int i = 0; i < array_len(g_pollfds); i++)
    {
      if (g_pollfds[i].fd > 0)
        {
          orb_unsubscribe(g_pollfds[i].fd);
        }
    }

  return ret;
}
