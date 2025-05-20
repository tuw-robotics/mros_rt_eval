#include <math.h>
#include <stdio.h>
#include <sys/time.h>
#include <unistd.h>

#include "driver/uart.h"
#include "driver/usb_serial_jtag.h"

#include "transport/uart.h"
#include "utils/timing_utils.h"

#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"

#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/u_int64.h>
#include <uros_network_interfaces.h>

#include <rmw_microros/rmw_microros.h>

// Kconfig options
#define PUB_TARGET_MS CONFIG_PUB_TARGET_MS
#define SYNC_TRIALS CONFIG_SYNC_TRIALS
#define WARMUP_TRIALS CONFIG_WARMUP_TRIALS
#define TOPIC_NAME CONFIG_TOPIC_NAME
#define NODE_NAME CONFIG_NODE_NAME

static const char *TAG = "uros_latency";

// hard-fail on errors
#define RCCHECK(fn, fmt, ...)                                                                                                                                                                                              \
    do {                                                                                                                                                                                                                   \
        rcl_ret_t _rc = (fn);                                                                                                                                                                                              \
        if (_rc != RCL_RET_OK) {                                                                                                                                                                                           \
            ESP_LOGE(TAG, fmt " (line %d): %d", ##__VA_ARGS__, __LINE__, (int)_rc);                                                                                                                                        \
            vTaskDelete(NULL);                                                                                                                                                                                             \
        }                                                                                                                                                                                                                  \
    } while (0)

// just warn on errors
#define RCSOFTCHECK(fn, fmt, ...)                                                                                                                                                                                          \
    do {                                                                                                                                                                                                                   \
        rcl_ret_t _rc = (fn);                                                                                                                                                                                              \
        if (_rc != RCL_RET_OK) {                                                                                                                                                                                           \
            ESP_LOGW(TAG, fmt " (line %d): %d", ##__VA_ARGS__, __LINE__, (int)_rc);                                                                                                                                        \
        }                                                                                                                                                                                                                  \
    } while (0)

// general C-API error check
#define GENERIC_CHECK(fn)                                                                                                                                                                                                  \
    do {                                                                                                                                                                                                                   \
        int _ret = (fn);                                                                                                                                                                                                   \
        if (_ret != 0) {                                                                                                                                                                                                   \
            ESP_LOGE(TAG, "Error at line %d: %d", __LINE__, _ret);                                                                                                                                                         \
            vTaskDelete(NULL);                                                                                                                                                                                             \
        }                                                                                                                                                                                                                  \
    } while (0)

static esp_err_t mros_sync_agent(wallclock_timestamp_t *timestamp, monotonic_timestamp_t timeout) {
    if (rmw_uros_sync_session(timeout) != RMW_RET_OK) {
        ESP_LOGE(TAG, "Failed to synchronize time with agent");
        return ESP_FAIL;
    }
    if (!rmw_uros_epoch_synchronized()) {
        ESP_LOGE(TAG, "Epoch not synchronized");
        return ESP_FAIL;
    }
    int64_t epoch_nanos = rmw_uros_epoch_nanos();
    timestamp->tv_sec = NS_TO_S(epoch_nanos);
    timestamp->tv_usec = NS_SUBS_TO_USEC(epoch_nanos);
    return ESP_OK;
}

void sync_time_to_agent(void) {
    RCCHECK(rmw_uros_ping_agent(CONFIG_MROS_PING_TIMEOUT_MS, CONFIG_MROS_MAX_PING_ATTEMPTS), "Failed to ping agent");
    wallclock_timestamp_t wts;
    RCCHECK(mros_sync_agent(&wts, CONFIG_MROS_MAX_TIME_SYNC_TIMEOUT_MS), "Failed to sync clock");
    GENERIC_CHECK(settimeofday(&wts, NULL));
}

rcl_publisher_t publisher;
std_msgs__msg__UInt64 latency_msg;
static uint64_t local_us[SYNC_TRIALS];
static uint64_t latency[SYNC_TRIALS];
static int trial_idx = 0;
static esp_timer_handle_t periodic_timer;
EventGroupHandle_t benchmark_event_group_handle = NULL;
#define TRIAL_FINISH BIT0

static void publish_callback(void *arg) {
    (void)arg;
    if (trial_idx >= SYNC_TRIALS) {
        xEventGroupSetBits(benchmark_event_group_handle, TRIAL_FINISH);
        return;
    }
    struct timeval tv;
    gettimeofday(&tv, NULL);
    uint64_t now = (uint64_t)tv.tv_sec * 1000000ULL + tv.tv_usec;
    latency_msg.data = now;
    local_us[trial_idx] = now;

    uint64_t t0 = esp_timer_get_time();
#ifdef CONFIG_PUBLISH_QOS_RELIABLE
    RCSOFTCHECK(rcl_publish(&publisher, &latency_msg, NULL), "publish (reliable) failed");
#else
    RCSOFTCHECK(rcl_publish(&publisher, &latency_msg, NULL), "publish (best_effort) failed");
#endif
    uint64_t t1 = esp_timer_get_time();
    latency[trial_idx] = t1 - t0;
    trial_idx++;
}

void benchmark_task(void *arg) {
    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;

    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
    RCCHECK(rcl_init_options_init(&init_options, allocator), "init_options init failed");

#if defined(CONFIG_MICRO_ROS_ESP_NETIF_WLAN) || defined(CONFIG_MICRO_ROS_ESP_NETIF_ENET)
    rmw_init_options_t *rmw_options = rcl_init_options_get_rmw_init_options(&init_options);
    RCCHECK(rmw_uros_options_set_udp_address(CONFIG_MICRO_ROS_AGENT_IP, CONFIG_MICRO_ROS_AGENT_PORT, rmw_options), "Failed to set UDP address");
#endif

    RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator), "support init failed");

    rcl_node_t node;
    RCCHECK(rclc_node_init_default(&node, NODE_NAME, "", &support), "node_init");

    // create a UInt64 publisher with chosen QoS and topic
#ifdef CONFIG_PUBLISH_QOS_RELIABLE
    RCCHECK(rclc_publisher_init_default(&publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt64), TOPIC_NAME), "pub init (reliable)");
#else
    RCCHECK(rclc_publisher_init_best_effort(&publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt64), TOPIC_NAME), "pub init (best_effort)");
#endif

    rclc_executor_t executor;
    RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator), "exec init");

    sync_time_to_agent();

    printf("Starting micro-ROS benchmark in 5 seconds...\n");
    vTaskDelay(pdMS_TO_TICKS(5000));

    const esp_timer_create_args_t timer_args = {.callback = &publish_callback, .arg = NULL, .dispatch_method = ESP_TIMER_TASK, .name = "pub_bench"};
    ESP_ERROR_CHECK(esp_timer_create(&timer_args, &periodic_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timer, PUB_TARGET_MS * 1000));

    xEventGroupWaitBits(benchmark_event_group_handle, TRIAL_FINISH, false, true, portMAX_DELAY);
    esp_timer_stop(periodic_timer);

    printf("trial,local_us,latency\n");

    uint64_t min = UINT64_MAX, max = 0, sum = 0;

    for (int i = 0; i < SYNC_TRIALS; i++) {
        if (latency[i] < min) {
            min = latency[i];
        }
        if (latency[i] > max) {
            max = latency[i];
        }
        sum += latency[i];
        printf("%d,%llu,%llu\n", i, local_us[i], latency[i]);
    }
    double mean = (double)sum / SYNC_TRIALS;
    double stddev = 0;
    for (int i = 0; i < SYNC_TRIALS; i++) {
        stddev += pow((double)latency[i] - mean, 2);
    }
    stddev = sqrt(stddev / SYNC_TRIALS);

    printf("min: %llu us\n", min);
    printf("max: %llu us\n", max);
    printf("mean: %.2f us\n", mean);
    printf("stddev: %.2f us\n", stddev);

    vTaskDelete(NULL);
}

void app_main(void) {
    benchmark_event_group_handle = xEventGroupCreate();

#ifdef CONFIG_IDF_TARGET_ESP32C6
    usb_serial_jtag_driver_config_t usb_cfg = {
        .rx_buffer_size = 1024,
        .tx_buffer_size = 1024,
    };
    ESP_ERROR_CHECK(usb_serial_jtag_driver_install(&usb_cfg));
#endif
    vTaskDelay(pdMS_TO_TICKS(1000));

#ifdef RMW_UXRCE_TRANSPORT_CUSTOM
    static size_t uart_port = UART_NUM_0;
    rmw_uros_set_custom_transport(true, (void *)&uart_port, serial_open, serial_close, serial_write, serial_read);
#endif

#if defined(CONFIG_MICRO_ROS_ESP_NETIF_WLAN) || defined(CONFIG_MICRO_ROS_ESP_NETIF_ENET)
    ESP_ERROR_CHECK(uros_network_interface_initialize());
#endif

    xTaskCreate(benchmark_task, "sync_bench", 4 * 1024, NULL, 5, NULL);
}
