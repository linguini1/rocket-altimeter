/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "sensor/continuity.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_UORB_FORMAT
static const char sensor_continuity_format[] =
    "sensor_continuity - timestamp:%" PRIu64 ",continuous:" PRIu8;
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

ORB_DEFINE(sensor_continuity, struct sensor_continuity,
           sensor_continuity_format);
