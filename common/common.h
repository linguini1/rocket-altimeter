#ifndef _ROCKETALT_COMMON_H
#define _ROCKETALT_COMMON_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>
#include <stdint.h>

#include <uORB/uORB.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define lp_filter(old, new, a) ((a) * (old) + (1.0 - (a)) * (new));

/* Array length helper */

#define array_len(arr) (sizeof(arr) / sizeof(arr[0]))

/****************************************************************************
 * Public Types
 ****************************************************************************/

enum fevent_e
{
  FEVENT_GROUNDED = 0, /* Rocket is waiting for liftoff */
  FEVENT_ASCENT = 1,   /* Rocket is ascending */
  FEVENT_APOGEE = 2,   /* Rocket has reached apogee */
  FEVENT_DESCENT = 3,  /* Rocket is descending */
  FEVENT_LANDED = 4,   /* Rocket has landed */
};

/* Conditions for deployment */

enum depcond_e
{
  COND_APOGEE = 0x1, /* Deploy at apogee */
  COND_ALT = 0x2,    /* Deploy at configured altitude, after apogee */
  COND_TIME = 0x4,   /* Deploy using timer, after ascent */
};

#endif /* _ROCKETALT_COMMON_H */
