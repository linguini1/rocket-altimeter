/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <fcntl.h>
#include <math.h>
#include <poll.h>
#include <stdint.h>
#include <stdlib.h>
#include <sys/types.h>
#include <syslog.h>

#include "../common/common.h"

/****************************************************************************
 * Pre-processor Definitions
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

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int main(int argc, char **argv)
{
  /* TODO: proper argument parsing using getopts */

  /* TODO:
   * This program will run in the shell to modify the locally stored copy of
   * the configuration file.
   *
   * This program should also be capable of operating as a BLE daemon that
   * accepts configuration changes over BLE.
   *
   * Basic usage options:
   *
   * Read out the whole contents in pretty-print:
   * config -r --file <path/to/conf>
   *
   * Read out deployment altitude:
   * config -r --file <path/to/conf> deploy.alt
   *
   * Write to the config file:
   * config -w --file <path/to/conf> (<part of config file> <value>)+
   *
   * Set deployment altitude to 3000m
   * config -w --file <path/to/conf> deploy.alt 3000
   *
   * Set deployment type to dual deploy
   * config -w --file <path/to/conf> deploy.type DUAL
   *
   * Set deployment type to single deploy
   * config -w --file <path/to/conf> deploy.type SINGLE
   *
   * Set multiple values
   * config -w --file <path/to/conf> deploy.type SINGLE deploy.alt 3000
   *
   * Run in daemon mode where changes are made over BLE:
   * config -d &
   */

  return EXIT_SUCCESS;
}
