/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "sensor/battery.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_UORB_FORMAT
static const char sensor_battery_format[] =
    "sensor_battery - timestamp:%" PRIu64 ",voltage:%hf,level:%" PRIu8 "";
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

ORB_DEFINE(sensor_battery, struct sensor_battery, sensor_battery_format);
