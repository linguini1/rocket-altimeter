/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <poll.h>
#include <stdio.h>
#include <stdlib.h>
#include <syslog.h>

#include <uORB/uORB.h>

#include "nimble/nimble_npl.h"
#include "nimble/nimble_port.h"

#include "host/ble_hs.h"
#include "host/util/util.h"

#include "services/ans/ble_svc_ans.h"
#include "services/bas/ble_svc_bas.h"
#include "services/dis/ble_svc_dis.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include "services/ias/ble_svc_ias.h"
#include "services/lls/ble_svc_lls.h"
#include "services/tps/ble_svc_tps.h"

#include "../common/common.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Program already knows about uORB topics */

ORB_DECLARE(sensor_voltage);
ORB_DECLARE(sensor_continuity);

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Name of the BLE device which appears on the connecting device */

static const char *g_gap_name = "Peanut"; /* TODO: this gets truncated */
static const char *g_dev_name = "Peanut Altimeter"; /* Device name */

static uint8_t g_own_addr_type; /* TODO: what is this for? */

/* Poll of topics we want to publish over Bluetooth */

static struct pollfd g_fds[2];

/****************************************************************************
 * External Public Function Prototypes
 ****************************************************************************/

void ble_hci_sock_ack_handler(FAR void *param);
void ble_hci_sock_set_device(int dev);

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int gap_event_cb(FAR struct ble_gap_event *event, FAR void *arg);

/****************************************************************************
 * Name: put_ad
 ****************************************************************************/

static void put_ad(uint8_t ad_type, uint8_t ad_len, FAR const void *ad,
                   FAR uint8_t *buf, FAR uint8_t *len)
{
  /* TODO: what does this do? */
  buf[(*len)++] = ad_len + 1;
  buf[(*len)++] = ad_type;

  memcpy(&buf[*len], ad, ad_len);

  *len += ad_len;
}

/****************************************************************************
 * Name: update_ad
 ****************************************************************************/

static void update_ad(void)
{
  /* TODO: what does this do? */
  uint8_t ad_flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
  uint8_t ad_len = 0;
  uint8_t ad[BLE_HS_ADV_MAX_SZ];

  put_ad(BLE_HS_ADV_TYPE_FLAGS, 1, &ad_flags, ad, &ad_len);
  put_ad(BLE_HS_ADV_TYPE_COMP_NAME, sizeof(g_gap_name), g_gap_name, ad,
         &ad_len);

  ble_gap_adv_set_data(ad, ad_len);
}

/****************************************************************************
 * Name: start_advertise
 ****************************************************************************/

static void start_advertise(void)
{
  struct ble_gap_adv_params advp;
  int rc;

  syslog(LOG_INFO | LOG_USER, "Started advertising.");

  /* TODO: what does this do? */

  update_ad();

  memset(&advp, 0, sizeof advp);
  advp.conn_mode = BLE_GAP_CONN_MODE_UND;
  advp.disc_mode = BLE_GAP_DISC_MODE_GEN;
  rc = ble_gap_adv_start(g_own_addr_type, NULL, BLE_HS_FOREVER, &advp,
                         gap_event_cb, NULL);
  assert(rc == 0);
}

/****************************************************************************
 * Name: gap_event_cb
 ****************************************************************************/

static int gap_event_cb(FAR struct ble_gap_event *event, FAR void *arg)
{
  /* TODO: what does this do? */

  switch (event->type)
    {
    case BLE_GAP_EVENT_CONNECT:
      {
        if (event->connect.status)
          {
            start_advertise();
          }
        break;
      }

    case BLE_GAP_EVENT_DISCONNECT:
      {
        start_advertise();
        break;
      }
    }

  return 0;
}

/****************************************************************************
 * Name: app_ble_sync_cb
 ****************************************************************************/

static void app_ble_sync_cb(void)
{
  ble_addr_t addr;
  int rc;

  /* TODO: what does this do? */

  /* Generate new non-resolvable private address */

  rc = ble_hs_id_gen_rnd(1, &addr);
  assert(rc == 0);

  /* Set generated address */

  rc = ble_hs_id_set_rnd(addr.val);
  assert(rc == 0);

  rc = ble_hs_util_ensure_addr(0);
  assert(rc == 0);

  rc = ble_hs_id_infer_auto(0, &g_own_addr_type);
  assert(rc == 0);

  start_advertise();
}

/****************************************************************************
 * Name: ble_hci_sock_task
 ****************************************************************************/

static FAR void *ble_hci_sock_task(FAR void *param)
{
  syslog(LOG_INFO | LOG_USER, "Started hci_sock task.");
  /* TODO: what does this do? */
  ble_hci_sock_ack_handler(param);
  return NULL;
}

/****************************************************************************
 * Name: ble_host_task
 ****************************************************************************/

static FAR void *ble_host_task(FAR void *param)
{
  syslog(LOG_INFO | LOG_USER, "Started ble_host task.");

  /* TODO: what does this do? */
  ble_hs_cfg.sync_cb = app_ble_sync_cb;

  /* Set some characteristics */

  ble_svc_gap_device_name_set(g_gap_name);
  ble_svc_dis_model_number_set(g_dev_name);

  /* Run the NimBLE stack */

  nimble_port_run();
  return NULL;
}

/****************************************************************************
 * Name: battery_publish
 *
 * Description:
 *   Publish battery data over BLE.
 *
 * Input Parameters:
 *   fd - The file descriptor of the uORB topic where there is battery data
 *        ready to read.
 *
 * Returned Value:
 *   0 on success, error code on failure.
 *
 ****************************************************************************/

static int battery_publish(int fd)
{
  int err;
  struct sensor_voltage data;

  /* Read in the data */

  err = orb_copy(ORB_ID(sensor_voltage), fd, &data);
  if (err)
    {
      syslog(LOG_ERR | LOG_USER, "Couldn't get battery data: %d\n", errno);
      return errno;
    }

  /* Publish over BLE
   * TODO: we shouldn't be converting from voltage to percentage here.
   *
   * Probably the solution is to use a custom uORB type for battery percentage
   * and we can also tell `batmon` what battery curve to use.
   *
   * OR we need to use a different BLE service that can report voltage. This
   * way the user can plug in batteries with whatever charge curve they like,
   * and its up to them to determine if the voltage is good or not.
   */

  err = ble_svc_bas_battery_level_set(100 * data.voltage / 4.2f);
  if (err)
    {
      syslog(LOG_ERR | LOG_USER, "Can't publish battery data: %d\n", err);
    }

  return err;
}

/****************************************************************************
 * Name: continuity_publish
 *
 * Description:
 *   Publish continuity data over BLE.
 *
 * Input Parameters:
 *   fd - The file descriptor of the uORB topic where continuity data is ready
 *        to read.
 *
 * Returned Value:
 *   0 on success, error code on failure.
 *
 ****************************************************************************/

static int continuity_publish(int fd)
{
  int err;
  struct sensor_continuity data;

  /* Read in the data */

  err = orb_copy(ORB_ID(sensor_continuity), fd, &data);
  if (err)
    {
      syslog(LOG_ERR | LOG_USER, "Couldn't get continuity data: %d\n", errno);
      return errno;
    }

  /* TODO: publish over BLE */

  return err;
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
  struct ble_npl_task s_task_host;
  struct ble_npl_task s_task_hci;

  if (argc < 2)
    {
      syslog(LOG_ERR | LOG_USER,
             "Expected Bluetooth interface as argument.\n");
      return EXIT_FAILURE;
    }

  syslog(LOG_INFO | LOG_USER, "btdaemon on %s!\n", argv[1]);

  /* TODO: Not sure what this does, I think it initializes the nimble stack? I
   * ripped this from the example.
   */

  nimble_port_init();

  /* Initialize services TODO: which ones do I need? */

  ble_svc_gap_init();  /* Generic access (shows device name) */
  ble_svc_gatt_init(); /* Generic attribute */
  ble_svc_ans_init();  /* Alert notification service */
  ble_svc_ias_init();  /* Immediate alert */
  ble_svc_lls_init();  /* Link loss */
  ble_svc_tps_init();  /* Transmit power */
  ble_svc_bas_init();  /* Battery */
  ble_svc_dis_init();  /* Device information */

  /* TODO: It would be good to implement the 'binary sensor service' for
   * continuity channels.
   * REF:
   * https://www.bluetooth.com/specifications/specs/binary-sensor-service-1-0/
   */

  /* TODO: We could use object transfer service for transferring log files (?)
   * It might also be beneficial to do this for the fake barometer using
   * Bluetooth; not sure.
   * REF: https://www.bluetooth.com/specifications/specs/object-transfer-service-1-0/
   */

  /* Create task which handles HCI socket */

  err = ble_npl_task_init(&s_task_hci, "hci_sock", ble_hci_sock_task, NULL,
                          CONFIG_ROCKETALT_BTDAEMON_PRIORITY,
                          BLE_NPL_TIME_FOREVER, NULL, 0);
  if (err)
    {
      syslog(LOG_ERR | LOG_USER, "Couldn't start hci task: %d\n", err);
    }

  /* Create task which handles default event queue for host stack. */

  err = ble_npl_task_init(&s_task_host, "ble_host", ble_host_task, NULL,
                          CONFIG_ROCKETALT_BTDAEMON_PRIORITY,
                          BLE_NPL_TIME_FOREVER, NULL, 0);
  if (err)
    {
      syslog(LOG_ERR | LOG_USER, "Couldn't start ble task: %d\n", err);
    }

  /* Subscribe to uORB topics
   * TODO: make this depend on what's configured
   * i.e. we only advertise continuity if continuity topics exist
   * Also should be dynamic (we subscribe to topic with devno 'n' if that was
   * passed as an argument).
   */

  g_fds[0].fd = orb_subscribe_multi(ORB_ID(sensor_voltage), 0);
  if (g_fds[0].fd < 0)
    {
      syslog(LOG_ERR | LOG_USER,
             "Couldn't subscribe to sensor_voltage0: %d\n", errno);
      ret = EXIT_FAILURE;
      goto cleanup_fds;
    }

  g_fds[1].fd = orb_subscribe_multi(ORB_ID(sensor_continuity), 0);
  if (g_fds[1].fd < 0)
    {
      syslog(LOG_ERR | LOG_USER,
             "Couldn't subscribe to sensor_continuity0: %d\n", errno);
      ret = EXIT_FAILURE;
      goto cleanup_fds;
    }

  /* Set up events for all fds */

  for (int i = 0; i < array_len(g_fds); i++)
    {
      g_fds[i].events = POLLIN;
      g_fds[i].revents = 0;
    }

  /* Now we can advertise our data */

  for (;;)
    {
      err = poll(g_fds, array_len(g_fds), -1);
      if (err <= 0)
        {
          /* Failed to poll, try again */
          continue;
        }

      /* Check the file descriptors that have events */

      for (int i = 0; i < array_len(g_fds); i++)
        {
          if (g_fds[i].revents & POLLIN)
            {
              /* Handle event based on the topic */

              switch (i)
                {
                case 0:
                  err = battery_publish(g_fds[i].fd);
                  break;

                case 1:
                  err = continuity_publish(g_fds[i].fd);
                  break;
                }

              g_fds[i].revents = 0; /* Clear event */
            }
        }
    }

cleanup_fds:

  for (int i = 0; i < array_len(g_fds); i++)
    {
      if (g_fds[i].fd > 0)
        {
          close(g_fds[i].fd);
        }
    }

  return ret;
}
