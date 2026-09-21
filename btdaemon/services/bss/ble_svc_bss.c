/* This implementation is based on the Bluetooth specification for the Binary
 * Sensor Service:
 * https://www.bluetooth.com/specifications/specs/binary-sensor-service-1-0/
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <assert.h>
#include <string.h>

#include "host/ble_gap.h"
#include "host/ble_hs.h"
#include "syscfg/syscfg.h"
#include "sysinit/sysinit.h"

#include "ble_svc_bss.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* The header of a message */

struct msg_hdr
{
  uint8_t _rfu1;   /* Reserved */
  uint8_t id;      /* Message ID */
  uint8_t _rfu2;   /* Reserved */
  uint8_t nparams; /* Number of parameters */
};

/* Header of a parameter */

struct param_hdr
{
  uint8_t id;      /* Parameter ID */
  uint8_t len;     /* Length */
  uint8_t _rfu[2]; /* Reserved */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int access_bss_ctrl_pt(uint16_t conn_handle, uint16_t attr_handle,
                              struct ble_gatt_access_ctxt *ctxt, void *arg);
static int access_bss_response(uint16_t conn_handle, uint16_t attr_handle,
                               struct ble_gatt_access_ctxt *ctxt, void *arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct ble_gatt_svc_def ble_svc_bss_defs[] = {
    {
        /* Binary sensor service */

        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = BLE_UUID16_DECLARE(BLE_SVC_BSS_UUID16),
        .characteristics =
            (struct ble_gatt_chr_def[]){
                /* BSS control point characteristic */

                {
                    .uuid = BLE_UUID16_DECLARE(
                        BLE_SVC_BSS_CHR_UUID16_BSS_CONTROL_POINT),
                    .access_cb = access_bss_ctrl_pt,
                    .flags = BLE_GATT_CHR_F_WRITE,
                },

                /* BSS response characteristic */

                {
                    .uuid = BLE_UUID16_DECLARE(
                        BLE_SVC_BSS_CHR_UUID16_BSS_RESPONSE),
                    .access_cb = access_bss_response,
                    .flags = BLE_GATT_CHR_F_INDICATE,
                },
                {
                    0, /* No more characteristics */
                },
            },
    },
    {
        0, /* No more services. */
    },
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: parse_msg_header
 *
 * Description:
 *   Parse incoming data into a message header from the BSS protocol.
 *
 * Input Parameters:
 *   om - The message buffer.
 *   hdr - Where to store the message header
 *
 * Returned Value:
 *   0 on success, NimBLE error code on failure.
 *
 ****************************************************************************/

static int parse_msg_header(const struct os_mbuf *om, struct msg_hdr *hdr)
{
  int rc = 0;

  /* First, make sure we have enough data to at least parse the message
   * header.
   */

  if (OS_MBUF_PKTLEN(om) < sizeof(struct msg_hdr))
    {
      return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
    }

  rc = ble_hs_mbuf_to_flat(om, hdr, sizeof(*hdr), NULL);
  if (rc != 0) return BLE_ATT_ERR_UNLIKELY;

  return rc;
}

static int send_get_sensor_status_response(uint8_t res, uint8_t state)
{
  struct
  {
    struct msg_hdr hdr;
    struct param_hdr result;
    uint8_t result_val;
    struct param_hdr status;
    uint16_t status_val;
  } response;

  memset(&response, 0, sizeof(response));
  response.hdr.id = BLE_SVC_BSS_RESP_GET_SEN_STATUS;
  response.hdr.nparams = 2;

  response.result.id = BLE_SVC_BSS_PRM_RESULT;
  response.result.len = 1;
  response.result_val = res;

  response.status.id = BLE_SVC_BSS_PRM_SENSTAT; /* TODO: multi ? */
  response.status.len = 2;
  response.status_val = state << 4; /* TODO: correct to use u16 ? */

  /* TODO: write to the response characteristic */

  return 0;
}

/****************************************************************************
 * Name: access_bss_ctrl_pt
 *
 * Description:
 *   Callback to access the BSS control point characteristic.
 *
 * Input Parameters:
 *   conn_handle - descr
 *   attr_handle - descr
 *   ctxt - descr
 *   arg - descr
 *
 * Returned Value:
 *   0 on success, NimBLE error code on failure.
 *
 ****************************************************************************/

static int access_bss_ctrl_pt(uint16_t conn_handle, uint16_t attr_handle,
                              struct ble_gatt_access_ctxt *ctxt, void *arg)
{
  int rc = 0;
  uint16_t uuid16 = ble_uuid_u16(ctxt->chr->uuid);
  struct msg_hdr hdr;

  assert(uuid16 == BLE_SVC_BSS_CHR_UUID16_BSS_CONTROL_POINT);
  assert(ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR);

  /* Parse message header */

  rc = parse_msg_header(ctxt->om, &hdr);
  if (rc) return rc;

  /* TODO */

  switch (hdr.id) /* Message (CMD) ID */
    {
    case BLE_SVC_BSS_CMD_GET_SEN_STATUS:
      /* This command has no parameters, so the message is done.
       *
       * We just need to response with Get Sensor Status Response on the
       * BSS Response characteristic.
       */

      rc = send_get_sensor_status_response(0, 0); /* TODO */
      if (rc != 0) return rc;
      break;

    case BLE_SVC_BSS_CMD_SET_SEN:
      /* TODO */
      break;

    default:
      return BLE_SVC_BSS_ERR_CMD_NOT_SUPPORTED;
    }

  return rc;
}

/****************************************************************************
 * Name: access_bss_response
 *
 * Description:
 *   Callback to access the BSS Response characteristic.
 *
 * Input Parameters:
 *   conn_handle - descr
 *   attr_handle - descr
 *   ctxt - descr
 *   arg - descr
 *
 * Returned Value:
 *   0 on success, NimBLE error code on failure.
 *
 ****************************************************************************/

static int access_bss_response(uint16_t conn_handle, uint16_t attr_handle,
                               struct ble_gatt_access_ctxt *ctxt, void *arg)
{
  int rc = 0;
  uint16_t uuid16 = ble_uuid_u16(ctxt->chr->uuid);
  assert(uuid16 == BLE_SVC_BSS_CHR_UUID16_BSS_RESPONSE);

  /* TODO */

  return rc;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ble_svc_bss_init
 *
 * Description:
 *   Initialize the binary sensor service.
 *
 ****************************************************************************/

void ble_svc_bss_init(void)
{
  int rc;

  /* Ensure this function only gets called by sysinit */

  SYSINIT_ASSERT_ACTIVE();

  rc = ble_gatts_count_cfg(ble_svc_bss_defs);
  SYSINIT_PANIC_ASSERT(rc == 0);

  rc = ble_gatts_add_svcs(ble_svc_bss_defs);
  SYSINIT_PANIC_ASSERT(rc == 0);
}
