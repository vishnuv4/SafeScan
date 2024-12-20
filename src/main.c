/*
 * Copyright (c) 2021 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <zephyr/kernel.h>
#include <stdio.h>
#include <string.h>
#include <zephyr/net/conn_mgr_connectivity.h>
#include <zephyr/net/conn_mgr_monitor.h>

// #include <memfault/metrics/metrics.h>
// #include <memfault/ports/zephyr/http.h>
// #include <memfault/core/data_packetizer.h>
// // #include <memfault/core/trace_event.h>

#include <dk_buttons_and_leds.h>
#include <zephyr/net/mqtt.h>
#include <zephyr/net/socket.h>
#include <zephyr/sys/printk.h>
#include <zephyr/random/random.h>
#include <string.h>
#include <zephyr/posix/unistd.h>
#include <zephyr/net/net_ip.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/net_mgmt.h>

#include <zephyr/logging/log.h>
#include <zephyr/logging/log_ctrl.h>

#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>

#define PN532_DEBUG

#define BUZZER_PIN		30
#define LED1_PIN		7

#define BUTTON1_PIN     8
#define BUTTON2_PIN     9

#define I2C1_NODE_PN532 DT_NODELABEL(pn532_sens1)
#define I2C2_NODE_PN532 DT_NODELABEL(pn532_sens2)
#define I2C3_NODE_PN532 DT_NODELABEL(pn532_sens3)
static const struct i2c_dt_spec dev_i2c[3] = {
    I2C_DT_SPEC_GET(I2C1_NODE_PN532),
    I2C_DT_SPEC_GET(I2C2_NODE_PN532),
    I2C_DT_SPEC_GET(I2C3_NODE_PN532)
};

#define ZEPHYR_USER_NODE DT_PATH(zephyr_user)

const struct device* const gpio0_dev = DEVICE_DT_GET(DT_NODELABEL(gpio0));
const struct device* const gpio1_dev = DEVICE_DT_GET(DT_NODELABEL(gpio1));

// PN532 Command and Response constants
#define PN532_PREAMBLE              0x00
#define PN532_STARTCODE1            0x00
#define PN532_STARTCODE2            0xFF
#define PN532_POSTAMBLE             0x00

#define PN532_HOSTTOPN532           0xD4
#define PN532_PN532TOHOST           0xD5

#define PN532_COMMAND_GETFIRMWAREVERSION   0x02
#define PN532_COMMAND_SAMCONFIGURATION     0x14
#define PN532_COMMAND_INLISTPASSIVETARGET  0x4A

static uint8_t pn532ack[] = {0x00, 0x00, 0xFF, 0x00, 0xFF, 0x00};

#define CHECKRET(val, str)	do { 			\
	if (val < 0)							\
	{										\
		LOG_ERR(str);						\
	}										\
}while(0)

// LOG_MODULE_REGISTER(memfault_sample, CONFIG_MEMFAULT_SAMPLE_LOG_LEVEL);
LOG_MODULE_REGISTER(safescan, LOG_LEVEL_INF);

/* Macros used to subscribe to specific Zephyr NET management events. */
#define L4_EVENT_MASK	      (NET_EVENT_L4_CONNECTED | NET_EVENT_L4_DISCONNECTED)
#define CONN_LAYER_EVENT_MASK (NET_EVENT_CONN_IF_FATAL_ERROR)

static K_SEM_DEFINE(nw_connected_sem, 0, 1);

/* Zephyr NET management event callback structures. */
static struct net_mgmt_event_callback l4_cb;
static struct net_mgmt_event_callback conn_cb;

uint32_t rfids_connected_current[3];
uint32_t rfids_connected_start[3];
uint32_t rfids_connected_end[3];

/* MQTT broker details */
#define MQTT_BROKER_HOSTNAME "broker.hivemq.com"
#define MQTT_BROKER_PORT     1883
#define MQTT_PUB_TOPIC       "nrf7002dk/board/publish"

/* MQTT buffers */
static uint8_t rx_buffer[1024];
static uint8_t tx_buffer[1024];

/* MQTT client and broker storage */
static struct mqtt_client client;
static struct sockaddr_storage broker;
static struct pollfd fds;

/* Flag to indicate IP is ready */
static volatile bool ip_ready = false;

/* Forward declarations */
static int connect_mqtt(void);
static int publish_message(struct mqtt_client *c, const char *topic, const char *msg);

static void mqtt_evt_handler(struct mqtt_client *const c, const struct mqtt_evt *evt)
{
    switch (evt->type) {
    case MQTT_EVT_CONNACK:
        if (evt->result != 0) {
            LOG_ERR("MQTT connect failed: %d", evt->result);
            break;
        }
        LOG_INF("MQTT client connected");
        break;

    case MQTT_EVT_DISCONNECT:
        LOG_INF("MQTT client disconnected %d", evt->result);
        break;

    default:
        break;
    }
}

static int server_resolve(void)
{
    struct addrinfo *res;
    struct addrinfo hints = {
        .ai_family = AF_INET,
        .ai_socktype = SOCK_STREAM
    };

    LOG_INF("Resolving hostname %s", MQTT_BROKER_HOSTNAME);
    int err = getaddrinfo(MQTT_BROKER_HOSTNAME, NULL, &hints, &res);
    if (err) {
        LOG_ERR("Failed to resolve hostname: %d", err);
        return err;
    }

    struct sockaddr_in *broker4 = (struct sockaddr_in *)&broker;
    broker4->sin_family = AF_INET;
    broker4->sin_port = htons(MQTT_BROKER_PORT);
    memcpy(&broker4->sin_addr,
           &((struct sockaddr_in *)res->ai_addr)->sin_addr,
           sizeof(struct in_addr));

    freeaddrinfo(res);
    return 0;
}

static void client_init(struct mqtt_client *client)
{
    mqtt_client_init(client);

    client->broker = (struct sockaddr *)&broker;
    client->evt_cb = mqtt_evt_handler;
    client->client_id.utf8 = "nrf7002dk_client";
    client->client_id.size = strlen("nrf7002dk_client");
    client->password = NULL;
    client->user_name = NULL;
    client->protocol_version = MQTT_VERSION_3_1_1;

    client->rx_buf = rx_buffer;
    client->rx_buf_size = sizeof(rx_buffer);
    client->tx_buf = tx_buffer;
    client->tx_buf_size = sizeof(tx_buffer);

    client->transport.type = MQTT_TRANSPORT_NON_SECURE;
}

static int connect_mqtt(void)
{
    int err = server_resolve();
    if (err) {
        LOG_ERR("Failed to resolve broker");
        return err;
    }

    client_init(&client);

    LOG_INF("Connecting to MQTT broker...");
    err = mqtt_connect(&client);
    if (err) {
        LOG_ERR("Error in mqtt_connect: %d", err);
        return err;
    }

    fds.fd = client.transport.tcp.sock;
    fds.events = POLLIN;

    return 0;
}

static int mqtt_process(void)
{
    int err = poll(&fds, 1, mqtt_keepalive_time_left(&client));
    if (err < 0) {
        LOG_ERR("Error in poll(): %d", errno);
        return err;
    }

    err = mqtt_live(&client);
    if ((err != 0) && (err != -EAGAIN)) {
        LOG_ERR("Error in mqtt_live: %d", err);
        return err;
    }

    if ((fds.revents & POLLIN) == POLLIN) {
        err = mqtt_input(&client);
        if (err != 0) {
            LOG_ERR("Error in mqtt_input: %d", err);
            return err;
        }
    }

    if ((fds.revents & POLLERR) == POLLERR) {
        LOG_ERR("POLLERR");
        return -EIO;
    }

    if ((fds.revents & POLLNVAL) == POLLNVAL) {
        LOG_ERR("POLLNVAL");
        return -EIO;
    }

    return 0;
}

static int publish_message(struct mqtt_client *c, const char *topic, const char *msg)
{
    struct mqtt_publish_param param = {
        .message.topic.qos = MQTT_QOS_1_AT_LEAST_ONCE,
        .message.topic.topic.utf8 = (char *)topic,
        .message.topic.topic.size = strlen(topic),
        .message.payload.data = (uint8_t *)msg,
        .message.payload.len = strlen(msg),
        .message_id = sys_rand32_get(),
        .dup_flag = 0,
        .retain_flag = 0,
    };

    LOG_INF("Publishing: %s to topic: %s", msg, topic);
    return mqtt_publish(c, &param);
}

/* Event handler to detect when we have an IP address */
static void net_event_handler(struct net_mgmt_event_callback *cb,
                              uint32_t mgmt_event, struct net_if *iface)
{
    if (mgmt_event == NET_EVENT_IPV4_ADDR_ADD) {
        LOG_INF("IPv4 address assigned, network ready");
        ip_ready = true;
    }
}

static void button_handler(uint32_t button_states, uint32_t has_changed)
{
	uint32_t buttons_pressed = has_changed & button_states;

    if (buttons_pressed & DK_BTN1_MSK)
    {
        LOG_INF("REGISTER");
        memcpy(rfids_connected_start, rfids_connected_current, 3*sizeof(uint32_t));
        for(int i=0; i<3; ++i)
        {
            LOG_INF("\tConnected to reader %d: %x", i, rfids_connected_start[i]);
        }
    }
    else if (buttons_pressed & DK_BTN2_MSK)
    {
        LOG_INF("CHECK");
        memcpy(rfids_connected_end, rfids_connected_current, 3*sizeof(uint32_t));
        for(int i=0; i<3; ++i)
        {
            LOG_INF("\tConnected to reader %d: %x", i, rfids_connected_end[i]);
        }
        if(memcmp(rfids_connected_end, rfids_connected_start, 3*sizeof(uint32_t)))
        {
            LOG_INF("Some items are not accounted for!");
            gpio_pin_set(gpio0_dev, BUZZER_PIN, 1);
        }
        else
        {
            LOG_INF("All items accounted for!");
            gpio_pin_set(gpio0_dev, BUZZER_PIN, 0);
        }
    }
}

// static void on_connect(void)
// {
// #if IS_ENABLED(MEMFAULT_NCS_LTE_METRICS)
// 	uint32_t time_to_lte_connection;

// 	/* Retrieve the LTE time to connect metric. */
// 	memfault_metrics_heartbeat_timer_read(MEMFAULT_METRICS_KEY(ncs_lte_time_to_connect_ms),
// 					      &time_to_lte_connection);

// 	LOG_INF("Time to connect: %d ms", time_to_lte_connection);
// #endif /* IS_ENABLED(MEMFAULT_NCS_LTE_METRICS) */

// 	LOG_INF("Sending already captured data to Memfault");

// 	/* Trigger collection of heartbeat data. */
// 	memfault_metrics_heartbeat_debug_trigger();

// 	/* Check if there is any data available to be sent. */
// 	if (!memfault_packetizer_data_available()) {
// 		LOG_DBG("There was no data to be sent");
// 		return;
// 	}

// 	LOG_DBG("Sending stored data...");

// 	/* Send the data that has been captured to the memfault cloud.
// 	 * This will also happen periodically, with an interval that can be configured using
// 	 * CONFIG_MEMFAULT_HTTP_PERIODIC_UPLOAD_INTERVAL_SECS.
// 	 */
// 	memfault_zephyr_port_post_data();
// }

static void l4_event_handler(struct net_mgmt_event_callback *cb, uint32_t event,
			     struct net_if *iface)
{
	switch (event) {
	case NET_EVENT_L4_CONNECTED:
		LOG_INF("Network connectivity established");
		k_sem_give(&nw_connected_sem);
		break;
	case NET_EVENT_L4_DISCONNECTED:
		LOG_INF("Network connectivity lost");
		break;
	default:
		LOG_DBG("Unknown event: 0x%08X", event);
		return;
	}
}

static void connectivity_event_handler(struct net_mgmt_event_callback *cb, uint32_t event,
				       struct net_if *iface)
{
	if (event == NET_EVENT_CONN_IF_FATAL_ERROR) {
		__ASSERT(false, "Failed to connect to a network");
		return;
	}
}

static int pn532_write_command(uint8_t *cmd, uint8_t cmd_len, int i2c)
{
    uint8_t frame[8 + cmd_len];
    uint8_t checksum = 0;
    uint8_t len = cmd_len + 1; // Length of data + TFI

    int idx = 0;

    frame[idx++] = PN532_PREAMBLE;
    frame[idx++] = PN532_STARTCODE1;
    frame[idx++] = PN532_STARTCODE2;
    frame[idx++] = len;
    frame[idx++] = ~len + 1;
    frame[idx++] = PN532_HOSTTOPN532; // TFI

    checksum += PN532_HOSTTOPN532;

    // LOG_INF("Building command frame...");
    for (uint8_t i = 0; i < cmd_len; i++)
    {
        frame[idx++] = cmd[i];
        checksum += cmd[i];
        // LOG_INF("  Command Byte %d: 0x%02X", i, cmd[i]);
    }

    frame[idx++] = ~checksum + 1;
    frame[idx++] = PN532_POSTAMBLE;

    // The PN532 expects a leading 0x00 when communicating over I2C
    uint8_t buffer[1 + idx];
    buffer[0] = 0x00; // I2C Start Code
    memcpy(buffer + 1, frame, idx);

    int ret = i2c_write(dev_i2c[i2c].bus, buffer, sizeof(buffer), dev_i2c[i2c].addr);
    if (ret)
    {
        LOG_ERR("I2C write failed: %d", ret);
        return ret;
    }

    // LOG_INF("Command frame sent successfully:");
    // for (int i = 0; i < sizeof(buffer); i++)
    // {
    //     LOG_INF("  Frame Byte %d: 0x%02X", i, buffer[i]);
    // }

    return 0;
}

static bool pn532_is_ready(int i2c)
{
    uint8_t status;
    int ret;

    ret = i2c_read(dev_i2c[i2c].bus, &status, 1, dev_i2c[i2c].addr);
    if (ret)
    {
        return false;
    }

    if (status == 0x01)
    {
        return true;
    }
    else
    {
        return false;
    }
}

static bool pn532_wait_ready(uint16_t timeout_ms, int i2c)
{
    uint16_t timer = 0;

    // LOG_INF("Waiting for PN532 to be ready...");

    while (!pn532_is_ready(i2c))
    {
        k_msleep(500);
        timer += 500;
        if (timer > timeout_ms)
        {
            return false;
        }
    }

    // LOG_INF("PN532 is ready");
    return true;
}

static int pn532_read_data(uint8_t *buffer, uint8_t length, int i2c)
{
    uint8_t rx_buffer[length + 1];
    int ret;

    ret = i2c_read(dev_i2c[i2c].bus, rx_buffer, length + 1, dev_i2c[i2c].addr);
    if (ret)
    {
        LOG_ERR("I2C read failed: %d", ret);
        return ret;
    }

    // Copy data to buffer (skip the status byte)
    memcpy(buffer, rx_buffer + 1, length);

    // LOG_INF("Read data from PN532:");
    // for (int i = 0; i < length; i++)
    // {
    //     LOG_INF("  Data Byte %d: 0x%02X", i, buffer[i]);
    // }

    return 0;
}

static bool pn532_read_ack(int i2c)
{
    uint8_t ack_buffer[6];
    int ret;

    // LOG_INF("Waiting for ACK...");

    if (!pn532_wait_ready(5000, i2c))
    {
        LOG_ERR("Timeout waiting for ACK");
        return false;
    }

    ret = pn532_read_data(ack_buffer, 6, i2c);
    if (ret)
    {
        return false;
    }

    if (memcmp(ack_buffer, pn532ack, 6) == 0)
    {
        // LOG_INF("Received ACK from PN532");
        return true;
    }
    else
    {
        LOG_ERR("Invalid ACK received:");
        for (int i = 0; i < 6; i++)
        {
            LOG_INF("  ACK Byte %d: 0x%02X", i, ack_buffer[i]);
        }
        return false;
    }
}

static bool pn532_send_command_check_ack(uint8_t *cmd, uint8_t cmd_len, uint16_t timeout, int i2c)
{
    int ret;

    // LOG_INF("Sending command and checking for ACK...");

    ret = pn532_write_command(cmd, cmd_len, i2c);
    if (ret)
    {
        return false;
    }

    if (!pn532_wait_ready(timeout, i2c))
    {
        LOG_ERR("PN532 not ready after command");
        return false;
    }

    if (!pn532_read_ack(i2c))
    {
        LOG_ERR("Did not receive ACK");
        return false;
    }

    return true;
}

uint32_t pn532_get_firmware_version(int i2c)
{
    uint8_t cmd = PN532_COMMAND_GETFIRMWAREVERSION;
    uint8_t response[12];
    uint32_t version;

    LOG_INF("Requesting PN532 firmware version...");

    if (!pn532_send_command_check_ack(&cmd, 1, 1000, i2c))
    {
        LOG_ERR("Failed to send GetFirmwareVersion command");
        return 0;
    }

    if (!pn532_wait_ready(1000, i2c))
    {
        return 0;
    }

    // Read response
    int ret = pn532_read_data(response, 12, i2c);
    if (ret)
    {
        return 0;
    }

    // Parse response
    if (response[0] != 0x00 || response[1] != 0x00 || response[2] != 0xFF)
    {
        LOG_ERR("Invalid response header");
        return 0;
    }

    uint8_t len = response[3];
    uint8_t lcs = response[4];
    uint8_t tfi = response[5];

    if ((uint8_t)(len + lcs) != 0x00)
    {
        LOG_ERR("Invalid length checksum");
        return 0;
    }

    if (tfi != PN532_PN532TOHOST)
    {
        LOG_ERR("Invalid TFI");
        return 0;
    }

    uint8_t cmd_code = response[6];
    if (cmd_code != (PN532_COMMAND_GETFIRMWAREVERSION + 1))
    {
        LOG_ERR("Unexpected command response");
        return 0;
    }

    // PD0 to PD3 are the firmware version data
    version = response[7];
    version <<= 8;
    version |= response[8];
    version <<= 8;
    version |= response[9];
    version <<= 8;
    version |= response[10];

    LOG_INF("PN532 Firmware Version: 0x%08X", version);

    return version;
}

bool pn532_SAMConfig(int i2c)
{
    uint8_t cmd[4];
    cmd[0] = PN532_COMMAND_SAMCONFIGURATION;
    cmd[1] = 0x01; // Normal mode
    cmd[2] = 0x14; // Timeout 50ms * 20 = 1 second
    cmd[3] = 0x01; // Use IRQ pin (if available)

    LOG_INF("Configuring PN532 SAM...");

    if (!pn532_send_command_check_ack(cmd, 4, 1000, i2c))
    {
        LOG_ERR("Failed to send SAMConfig command");
        return false;
    }

    if (!pn532_wait_ready(1000, i2c))
    {
        return false;
    }

    // Read response
    uint8_t response[8];
    int ret = pn532_read_data(response, 8, i2c);
    if (ret)
    {
        return false;
    }

    // Check response
    if (response[6] != (PN532_COMMAND_SAMCONFIGURATION + 1))
    {
        LOG_ERR("Invalid SAMConfig response");
        return false;
    }

    LOG_INF("PN532 SAM configuration successful");
    return true;
}

bool pn532_read_passive_target_id(uint8_t *uid, uint8_t *uid_len, uint16_t timeout, int i2c)
{
    uint8_t cmd[3];
    cmd[0] = PN532_COMMAND_INLISTPASSIVETARGET;
    cmd[1] = 0x02; // Max 1 card at a time
    cmd[2] = 0x00; // 106 kbps type A (ISO14443A/MIFARE)

    // LOG_INF("Sending InListPassiveTarget command...");

    if (!pn532_send_command_check_ack(cmd, 3, timeout, i2c))
    {
        LOG_ERR("Failed to send InListPassiveTarget command");
        return false;
    }

    // Wait for response
    if (!pn532_wait_ready(timeout, i2c))
    {
        return false;
    }

    // while(!pn532_wait_ready(timeout));

    // Read response
    uint8_t response[20];
    int ret = pn532_read_data(response, sizeof(response), i2c);
    if (ret)
    {
        LOG_ERR("PN532 read failed");
        return false;
    }

    // Parse response
    if (response[6] != (PN532_COMMAND_INLISTPASSIVETARGET + 1))
    {
        LOG_ERR("Invalid InListPassiveTarget response");
        return false;
    }

    uint8_t nb_targets = response[7];
    if (nb_targets != 1)
    {
        LOG_ERR("No card detected");
        return false;
    }

    uint8_t uid_length = response[12];

    memcpy(uid, &response[13], uid_length);
    *uid_len = uid_length;

    if (!pn532_send_command_check_ack(cmd, 3, timeout, i2c))
    {
        LOG_ERR("Failed to send InListPassiveTarget command");
        return false;
    }

    return true;
}

#define FIRST_PN532     0
#define LAST_PN532      2

int main(void)
{
    gpio_pin_set(gpio0_dev, BUZZER_PIN, 0);

    static struct net_mgmt_event_callback net_cb;
    net_mgmt_init_event_callback(&net_cb, net_event_handler,
                                 NET_EVENT_IPV4_ADDR_ADD);
    net_mgmt_add_event_callback(&net_cb);

    LOG_INF("Bringing up all network interfaces...");
    if (conn_mgr_all_if_up(true)) {
        LOG_ERR("Failed to bring up network interfaces");
        return -1;
    }

    LOG_INF("Connecting all interfaces to network...");
    if (conn_mgr_all_if_connect(true)) {
        LOG_ERR("Failed to connect to network");
        return -1;
    }

    /* Wait until we actually have an IP address assigned */
    while (!ip_ready) {
        k_sleep(K_MSEC(100));
    }

    /* Now that we have an IP address and DNS servers, we can connect to MQTT */
    if (connect_mqtt()) {
        LOG_ERR("Failed to connect to MQTT broker");
        return -1;
    }

    /* Wait for the connection to establish. */
    k_sleep(K_SECONDS(3));

    int err;

	LOG_INF("Safescan has started");

	err = dk_buttons_init(button_handler);
	if (err) {
		LOG_ERR("dk_buttons_init, error: %d", err);
	}
    gpio_pin_set(gpio0_dev, BUZZER_PIN, 0);
	/* Setup handler for Zephyr NET Connection Manager events. */
	net_mgmt_init_event_callback(&l4_cb, l4_event_handler, L4_EVENT_MASK);
	net_mgmt_add_event_callback(&l4_cb);

	/* Setup handler for Zephyr NET Connection Manager Connectivity layer. */
	net_mgmt_init_event_callback(&conn_cb, connectivity_event_handler, CONN_LAYER_EVENT_MASK);
	net_mgmt_add_event_callback(&conn_cb);

	/* Connecting to the configured connectivity layer.
	 * Wi-Fi or LTE depending on the board that the sample was built for.
	 */
	LOG_INF("Bringing network interface up and connecting to the network");

	err = conn_mgr_all_if_up(true);
	if (err) {
		__ASSERT(false, "conn_mgr_all_if_up, error: %d", err);
		return err;
	}

	err = conn_mgr_all_if_connect(true);
	if (err) {
		__ASSERT(false, "conn_mgr_all_if_connect, error: %d", err);
		return err;
	}

	k_sem_take(&nw_connected_sem, K_FOREVER);
	LOG_INF("Connected to network");
	// on_connect();

	int ret;

	ret = gpio_pin_configure(gpio0_dev, BUZZER_PIN, GPIO_OUTPUT_ACTIVE);
	CHECKRET(ret, "Buzzer pin Init failed!\n");
    gpio_pin_set(gpio0_dev, BUZZER_PIN, 0);

	ret = gpio_pin_configure(gpio1_dev, LED1_PIN, GPIO_OUTPUT_ACTIVE);
	CHECKRET(ret, "LED pin Init failed!\n");

    LOG_INF("Safescan Started");

    for(int i=FIRST_PN532; i <= LAST_PN532; ++i)
    {
        ret = device_is_ready(dev_i2c[i].bus);
        while (!ret)
        {
            LOG_ERR("I2C %d not ready! Failing with error code %d", i, ret);
        }
    }
    LOG_INF("I2C bus devices are ready");

    k_msleep(100);

    for(int i = FIRST_PN532; i <= LAST_PN532; ++i)
    {
        uint32_t version = pn532_get_firmware_version(i);
        if (version == 0)
        {
            LOG_ERR("Failed to get PN532 %d firmware version", i);
        }
        else
        {
            LOG_INF("PN532 %d Firmware Version: 0x%08X", i, version);
        }

        if (!pn532_SAMConfig(i))
        {
            LOG_ERR("Failed to configure PN532 %d SAM", i);
        }
        else
        {
            LOG_INF("PN532 %d SAM configured", i);
        }
    }

    char readerstatus[3][50];

	while (1) {

		// gpio_pin_toggle(gpio1_dev, LED1_PIN);
        err = mqtt_process();
        if (err != 0) {
            /* Connection might have dropped, try reconnecting */
            LOG_INF("Reconnecting to MQTT broker...");
            k_sleep(K_SECONDS(5));
            if (connect_mqtt()) {
                LOG_ERR("Reconnection failed");
                continue;
            }
        }

        uint8_t uid[10];
        uint8_t uid_len = 0;

        for(int i = FIRST_PN532; i <= LAST_PN532; ++i)
        {
            if (pn532_read_passive_target_id(uid, &uid_len, 500, i))
            {
                if(uid_len != 4)
                {
                    LOG_ERR("Invalid UID");
                }
                uint32_t combined_uid = 0;
                for(int i=0; i<4; ++i)
                {
                    combined_uid |= (uid[3-i] << (i * 8));
                }
                rfids_connected_current[i] = combined_uid;
                // LOG_INF("Reader %d :: Tag detected! UID : %X", i, combined_uid);
                sprintf(readerstatus[i], "Reader %d :: Tag detected! UID : %X", i, combined_uid);
                LOG_INF("%s", readerstatus[i]);
                publish_message(&client, MQTT_PUB_TOPIC, readerstatus[i]);
            }
            else
            {
                // LOG_INF("Reader %d :: No tag detected", i);
                sprintf(readerstatus[i], "Reader %d :: No tag detected", i);
                LOG_INF("%s", readerstatus[i]);
                rfids_connected_current[i] = 0;
                publish_message(&client, MQTT_PUB_TOPIC, readerstatus[i]);
            }
            k_msleep(200);
        }
        k_msleep(500);
	}
    mqtt_disconnect(&client);
}
