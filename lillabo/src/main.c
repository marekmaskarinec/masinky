/*
 * DRV8212 A/B leg test
 * --------------------
 *
 * DRV8212 IN1/IN2 truth table (logical levels; 1 = pin driven active):
 *
 *   IN1  IN2   Bridge state
 *   ---  ---   -------------------------------------------------
 *    0    0    COAST   - both outputs Hi-Z          (safe idle / baseline)
 *    1    0    FORWARD - OUT1 high-side ON          (exercises leg 1)
 *    0    1    REVERSE - OUT2 high-side ON          (exercises leg 2 = suspect)
 *    1    1    BRAKE   - both low-side ON           (NEVER commanded here)
 *
 * Safety of the test itself:
 *   - We ALWAYS pass through COAST (0,0) between drive states, so the inputs
 *     never transition straight from (1,0) to (0,1). That makes it impossible
 *     to momentarily land on (1,1)=brake or to cross-conduct during the switch.
 *   - REVERSE (the suspect leg) is kept short, and the whole test is bounded
 *     to a few cycles then parked in COAST, so a damaged leg isn't baked
 *     further while you read the trace.
 */

#include <zephyr/drivers/pwm.h>
#include <zephyr/kernel.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/uuid.h>

static const struct pwm_dt_spec in1 = PWM_DT_SPEC_GET(DT_NODELABEL(in1));
static const struct pwm_dt_spec in2 = PWM_DT_SPEC_GET(DT_NODELABEL(in2));

/* Motor control GATT service.
 *
 * Custom 128-bit UUIDs. Random base 0c103c4b-6ae9-47e2-9983-100f660c9654,
 * low byte of the first field varied per attribute:
 *   service  0c103c01-...   primary service
 *   speed    0c103c02-...   uint8, 0-255
 *   mode     0c103c03-...   uint8, see enum motor_mode below
 */
#define BT_UUID_MOTOR_SERVICE_VAL \
	BT_UUID_128_ENCODE(0x0c103c01, 0x6ae9, 0x47e2, 0x9983, 0x100f660c9654)
#define BT_UUID_MOTOR_SERVICE BT_UUID_DECLARE_128(BT_UUID_MOTOR_SERVICE_VAL)

#define BT_UUID_MOTOR_SPEED_VAL \
	BT_UUID_128_ENCODE(0x0c103c02, 0x6ae9, 0x47e2, 0x9983, 0x100f660c9654)
#define BT_UUID_MOTOR_SPEED BT_UUID_DECLARE_128(BT_UUID_MOTOR_SPEED_VAL)

#define BT_UUID_MOTOR_MODE_VAL \
	BT_UUID_128_ENCODE(0x0c103c03, 0x6ae9, 0x47e2, 0x9983, 0x100f660c9654)
#define BT_UUID_MOTOR_MODE BT_UUID_DECLARE_128(BT_UUID_MOTOR_MODE_VAL)

enum motor_mode
{
	MOTOR_FORWARD = 0,
	MOTOR_BACKWARDS,
	MOTOR_COAST,
	MOTOR_BRAKE,
};

static uint8_t motor_speed; /* 0-255 */
static uint8_t motor_mode = MOTOR_COAST;

static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_MOTOR_SERVICE_VAL),
};

static const struct bt_data sd[] = {
    BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME, sizeof(CONFIG_BT_DEVICE_NAME) - 1),
};

/* Set the bridge state. We set the pin going to 0 first when leaving a drive
 * state, which keeps every intermediate combination a safe one. */
static int
bridge_set(uint8_t v1, uint8_t v2)
{
	printk("v1=%d,v2=%d\n", v1 * 100000 / 255, v2 * 100000 / 255);
	int ret = pwm_set_pulse_dt(&in1, v1 * 100000 / 255);
	if (ret < 0) {
		return ret;
	}
	return pwm_set_pulse_dt(&in2, v2 * 100000 / 255);
}

/* Apply the current motor state to the hardware. Called whenever the Speed or
 * Mode characteristic is written. Fill this in to drive the H-bridge (e.g. PWM
 * duty from `speed`, direction from `mode` via bridge_set()). */
static void
motor_apply(uint8_t mode, uint8_t speed)
{
	switch (mode) {
	case MOTOR_FORWARD: bridge_set(speed, 0); break;
	case MOTOR_BACKWARDS: bridge_set(0, speed); break;
	case MOTOR_BRAKE: bridge_set(speed, speed); break;
	case MOTOR_COAST: bridge_set(0, 0); break;
	}

	printk("Applied motor settings mode=%d,speed=%02x\n", mode, speed);
}

static ssize_t
read_speed(
    struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf, uint16_t len, uint16_t offset)
{
	return bt_gatt_attr_read(conn, attr, buf, len, offset, &motor_speed, sizeof(motor_speed));
}

static ssize_t
write_speed(struct bt_conn *conn, const struct bt_gatt_attr *attr, const void *buf, uint16_t len,
    uint16_t offset, uint8_t flags)
{
	if (offset != 0) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
	}
	if (len != sizeof(motor_speed)) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
	}

	motor_speed = ((const uint8_t *)buf)[0];
	motor_apply(motor_mode, motor_speed);
	return len;
}

static ssize_t
read_mode(
    struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf, uint16_t len, uint16_t offset)
{
	return bt_gatt_attr_read(conn, attr, buf, len, offset, &motor_mode, sizeof(motor_mode));
}

static ssize_t
write_mode(struct bt_conn *conn, const struct bt_gatt_attr *attr, const void *buf, uint16_t len,
    uint16_t offset, uint8_t flags)
{
	uint8_t mode;

	if (offset != 0) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
	}
	if (len != sizeof(motor_mode)) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
	}

	mode = ((const uint8_t *)buf)[0];
	if (mode > MOTOR_BRAKE) {
		return BT_GATT_ERR(BT_ATT_ERR_VALUE_NOT_ALLOWED);
	}

	motor_mode = mode;
	motor_apply(motor_mode, motor_speed);
	return len;
}

BT_GATT_SERVICE_DEFINE(motor_svc, BT_GATT_PRIMARY_SERVICE(BT_UUID_MOTOR_SERVICE),
    BT_GATT_CHARACTERISTIC(BT_UUID_MOTOR_SPEED, BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE,
	BT_GATT_PERM_READ | BT_GATT_PERM_WRITE, read_speed, write_speed, &motor_speed),
    BT_GATT_CHARACTERISTIC(BT_UUID_MOTOR_MODE, BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE,
	BT_GATT_PERM_READ | BT_GATT_PERM_WRITE, read_mode, write_mode, &motor_mode), );

static int
start_adv(void)
{
	int ret = bt_le_adv_start(BT_LE_ADV_CONN_FAST_1, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
	if (ret) {
		printk("Advertising failed to start (%d)\n", ret);
	}
	return ret;
}

/* Restart advertising off the system workqueue: starting connectable adv
 * directly from the disconnected callback fails with -ENOMEM because the
 * just-closed connection slot is not freed until the callback returns. */
static void
adv_work_handler(struct k_work *work)
{
	start_adv();
}

static K_WORK_DEFINE(adv_work, adv_work_handler);

static void
connected(struct bt_conn *conn, uint8_t err)
{
	if (err) {
		printk("Connection failed, err 0x%02x %s\n", err, bt_hci_err_to_str(err));
	} else {
		printk("Connected\n");
	}
}

static void
disconnected(struct bt_conn *conn, uint8_t reason)
{
	printk("Disconnected, reason 0x%02x %s\n", reason, bt_hci_err_to_str(reason));
	k_work_submit(&adv_work);
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
    .connected = connected,
    .disconnected = disconnected,
};

static void
auth_passkey_display(struct bt_conn *conn, unsigned int passkey)
{
	printk("Passkey for %s: %06u\n", bt_conn_dst_str(conn), passkey);
}

static void
auth_cancel(struct bt_conn *conn)
{
	printk("Pairing cancelled: %s\n", bt_conn_dst_str(conn));
}

void
mtu_updated(struct bt_conn *conn, uint16_t tx, uint16_t rx)
{
	printk("Updated MTU: TX: %d RX: %d bytes\n", tx, rx);
}

static struct bt_gatt_cb gatt_callbacks = {.att_mtu_updated = mtu_updated};

static struct bt_conn_auth_cb auth_cb_display = {
    .passkey_display = auth_passkey_display,
    .passkey_entry = NULL,
    .cancel = auth_cancel,
};

static int
init_bt(void)
{
	int ret;
	ret = bt_enable(NULL);
	if (ret) {
		printk("Bluetooth init failed (%d)\n", ret);
		return ret;
	}

	start_adv();

	bt_conn_auth_cb_register(&auth_cb_display);
	bt_gatt_cb_register(&gatt_callbacks);

	printk("Advertising successfully started\n");

	return 0;
}

int
main(void)
{
	int ret;

	bridge_set(0, 0);

	ret = init_bt();
	if (ret) {
		printk("ERROR: Bluetooth setup failed (%d)\n", ret);
		return 0;
	}

	while (1) {
		k_msleep(1000);
	}
	return 0;
}
