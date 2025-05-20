# Results: ESP32-C6 on 18.05.2025

Following results were obtained using the ESP32-C6 and the provided test firmware and test scripts:

## Micro-ROS Publisher

For serial transport at 115200 baud at 100ms publish rate with reliable and best effort respectivly:
- results_pub_serial_115200_100ms_10000_best_effort_reception.csv
- results_pub_serial_115200_100ms_10000_best_effort.csv
- results_pub_serial_115200_100ms_10000_reliable_reception.csv
- results_pub_serial_115200_100ms_10000_reliable.csv

Here, files ending with `reception` contain the host-side measurements while the others represent the ESP32-C6 side measurements. 10000 messages were sent with a 8 byte payload.

For the same setup but with the maximum publish rate:
- results_pub_serial_115200_max_10000_best_effort_reception.csv
- results_pub_serial_115200_max_10000_best_effort.csv
- results_pub_serial_115200_max_10000_reliable_reception.csv
- results_pub_serial_115200_max_10000_reliable.csv

Then, the same tests were performed using UDP transport:
- results_pub_udp_100ms_10000_best_effort_reception.csv
- results_pub_udp_100ms_10000_best_effort.csv
- results_pub_udp_100ms_10000_reliable_reception.csv
- results_pub_udp_100ms_10000_reliable.csv
- results_pub_udp_max_10000_best_effort_reception.csv
- results_pub_udp_max_10000_best_effort.csv
- results_pub_udp_max_10000_reliable_reception.csv
- results_pub_udp_max_10000_reliable.csv

The structure of the csv files depends on if it is a host-side or ESP32-C6 side measurement. These files are created by the latency_eval node within the rt_eval package (see `host` directory for more detail). It was run using `ros2 run rt_eval latency_eval` and `--ros-args -p reliability:=reliable` or `--ros-args -p reliability:=best_effort` respectivly. Host-side measurements (`reception`) contain the following columns:
- `trial`: the number of the received message (0 for the first message with incrementing count)
- `sent_us`: the firmware-side publish time in microseconds (synchronized with the host)
- `recv_us`: the reception time in microseconds
- `latency_us`: the latency in microseconds (recv_us - sent_us)

For the firmware-side measurements, the columns are:
trial,local_us,latency
- `trial`: the number of the published message (0 for the first message with incrementing count)
- `local_us`: the local publish time in microseconds
- `latency`: the latency of the `rcl_publish` call in microseconds

## Micro-ROS Synchronization

To evaluate the synchronization performance, 10000 synchronization calls were performed with serial and UDF transport respectively. The results are stored in the following files:
- results_sync_serial_115200_10000.csv
- results_sync_udp_10000.csv

The synchronization call evaluated is:
```c
static esp_err_t mros_sync_agent(wallclock_timestamp_t *ts, uint32_t to_ms) {
    if (rmw_uros_sync_session(to_ms * 1000) != RMW_RET_OK || !rmw_uros_epoch_synchronized())
        return ESP_FAIL;
    int64_t n = rmw_uros_epoch_nanos();
    ts->tv_sec = n / 1000000000LL;
    ts->tv_usec = (n % 1000000000LL) / 1000LL;
    return ESP_OK;
}
```

The structure of the respective csv files is as follows:
- `trial`: the number of the synchronization call (0 for the first message with incrementing count)
- `local_us`: the local time in microseconds (obtained by `esp_timer_get_time()`)
- `remote_us`: the from the synchronization call obtained time in microseconds
- `latency_us`: the latency of the synchronization call in microseconds
- `delta_local_us`: the difference between two consecutive local timestamps in microseconds
- `delta_remote_us`: the difference between two consecutive remote timestamps in microseconds
- `error_us`: the difference between `delta_remote_us` and `delta_local_us` in microseconds (`delta_remote_us - delta_local_us`)