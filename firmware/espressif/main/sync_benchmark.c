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
#include "freertos/task.h"

#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <std_msgs/msg/int32.h>
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

static uint64_t local_us[SYNC_TRIALS];
static uint64_t remote_ts[SYNC_TRIALS];
static uint64_t latency[SYNC_TRIALS];

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

    RCCHECK(rmw_uros_ping_agent(1000, 5), "ping_agent failed");

    wallclock_timestamp_t wts;

    for (int i = 0; i < SYNC_TRIALS; i++) {
        local_us[i] = esp_timer_get_time();
        uint64_t t0 = esp_timer_get_time();
        RCCHECK(mros_sync_agent(&wts, CONFIG_MROS_MAX_TIME_SYNC_TIMEOUT_MS), "sync_agent failed on trial %d", i);
        uint64_t t1 = esp_timer_get_time();
        latency[i] = t1 - t0;
        remote_ts[i] = (uint64_t)wts.tv_sec * 1000000ULL + wts.tv_usec;
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    printf("trial,local_us,remote_us,latency_us,"
           "delta_local_us,delta_remote_us,error_us\n");

    uint64_t lat_min = UINT64_MAX, lat_max = 0;
    double lat_sum = 0, lat_sumsq = 0;
    int64_t err_min = INT64_MAX, err_max = INT64_MIN;
    double err_sum = 0, err_sumsq = 0;
    int err_n = 0;

    for (int i = 0; i < SYNC_TRIALS; i++) {
        uint64_t dloc = 0, drem = 0;
        int64_t err = 0;
        if (i > 0) {
            dloc = local_us[i] - local_us[i - 1];
            drem = remote_ts[i] - remote_ts[i - 1];
            err = (int64_t)drem - (int64_t)dloc;
        }
        printf(
            "%d,%llu,%llu,%llu,%llu,%llu,%lld\n", i, (unsigned long long)local_us[i], (unsigned long long)remote_ts[i], (unsigned long long)latency[i], (unsigned long long)dloc, (unsigned long long)drem, (long long)err);

        if (i >= WARMUP_TRIALS) {
            uint64_t L = latency[i];
            lat_min = (L < lat_min) ? L : lat_min;
            lat_max = (L > lat_max) ? L : lat_max;
            lat_sum += (double)L;
            lat_sumsq += (double)L * (double)L;
            if (i > 0) {
                err_min = (err < err_min) ? err : err_min;
                err_max = (err > err_max) ? err : err_max;
                err_sum += (double)err;
                err_sumsq += (double)err * (double)err;
                err_n++;
            }
        }
    }

    int lat_n = SYNC_TRIALS - WARMUP_TRIALS;
    double lat_mean = lat_sum / lat_n;
    double lat_var = lat_sumsq / lat_n - lat_mean * lat_mean;
    double lat_stddev = sqrt(lat_var);

    double err_mean = err_sum / err_n;
    double err_var = err_sumsq / err_n - err_mean * err_mean;
    double err_stddev = sqrt(err_var);

    printf("latency min: %llu us\n", lat_min);
    printf("latency max: %llu us\n", lat_max);
    printf("latency mean: %.2f us\n", lat_mean);
    printf("latency stddev: %.2f us\n", lat_stddev);
    printf("latency var: %.2f us\n", lat_var);
    printf("error min: %lld us\n", err_min);
    printf("error max: %lld us\n", err_max);
    printf("error mean: %.2f us\n", err_mean);
    printf("error stddev: %.2f us\n", err_stddev);
    printf("error var: %.2f us\n", err_var);

    vTaskDelete(NULL);
}

void app_main(void) {
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
