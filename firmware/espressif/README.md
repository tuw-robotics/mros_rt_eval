# Real Time Evaluation - Firmware Side

This ESP-IDF project provides firmware components for evaluating the timing behaviour of different micro-ROS aspects. Currently, we provide three different benchmarks:
- Agent Synchronization
- Publishing on timer basis (8 byte message)
- Publishing as fast as possible (8 byte message)

For each benchmark, the transport type and other parameters can be adjusted. The benchmarks are designed to be run on the ESP32 based microcontrollers with the esp32c6 only currently tested. The firmware side of the benchmark is structured, so that the results are streamed after all trials are completed. This is done to avoid the overhead of printing the results in between the trials and the jtag interface is used to stream the results. The format of the streamed data is designed to be easily converted into a CSV file.

## How to use
- Open the `firmware` folder as a VSCode Dev Container. This will automatically give you the nessesary environment to build and flash the firmware.
- Adjust the `CMakeLists.txt` file to include the benchmark you want to run. The default is the `sync_benchmark.c` file.
- Set the hardware target by calling `idf.py set-target esp32c6`. This will set the target to the ESP32C6, others have not been tested yet.
- Adjust the transport you want to use by
  - changing the `RMW_UXRCE_TRANSPORT` macro in the `app-colcon.meta` file. The default is `custom`, which uses the custom serial transport. The other option tested so far is `udp`.
  - if `udp` is used, the agent IP and network interface need to be adjusted in the `menuconfig`. For this, run `idf.py menuconfig` and navigate to `micro-ROS Settings -> micro-ROS network interface select -> WLAN interface`. Then back to `micro-ROS Settings` and adjust the `micro-ROS agent IP` to the IP of your agent.
- Configure the test parameters also within the `menuconfig` in the `Application-specific settings` menu. A short description of each parameter is provided there.
- The firmware can then be built and flashed using `idf.py build flash -p /dev/ttyACM0`. The ACM0 port might need adjustment depending on your system and the chosen USB port the ESP32 is connected to.
- After flashing, the firmware can be monitored using `idf.py monitor -p /dev/ttyACM0`. This will show the output of the benchmark. The output is streamed in a way that can easily be converted into a CSV file. To redirect the output directly to a file, you can use `idf.py monitor -p /dev/ttyACM0 > results.log`. This will save the output to a file called `results.log` in the current directory.

## Agent Synchronization
Micro-ROS provides a way to synchronize with the agent. This is crucial, as stamped ROS2 messages need to be within the same timebase in the whole system. Using NTP under the hood, the calls are blocking until the synchronization is complete (or a timeout occurs). For evaluation, the `sync_benchmark.c` file is provided. This file calls the synchronization function:

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

It is called `SYNC_TRIALS` times (configureable in the `menuconfig`), and the time is measured it takes to complete the call. After all calls are completed, the raw data is streamed through the jtag interface as well as min, max, mean and standard deviation are evaluated. The raw data is streamed in a way that can easily be converted into a CSV file and structured in this format:

```
trial,local_us,remote_us,latency_us,delta_local_us,delta_remote_us,error_us
```

Where:
- `trial`: the number of the synchronization call (0 for the first message with incrementing count)
- `local_us`: the local time in microseconds (obtained by `esp_timer_get_time()`)
- `remote_us`: the from the synchronization call obtained time in microseconds
- `latency_us`: the latency of the synchronization call in microseconds
- `delta_local_us`: the difference between two consecutive local timestamps in microseconds
- `delta_remote_us`: the difference between two consecutive remote timestamps in microseconds
- `error_us`: the difference between `delta_remote_us` and `delta_local_us` in microseconds (`delta_remote_us - delta_local_us`)


## Fast Publishing
To evaluate max publishing performance, the `pub_benchmark.c` file is provided. Here, a publisher is setup that publishes the current synchronized timestamp (as a unsigned 64 bit integer) of the microcontroller as fast as possible (not with a timer). As in the synchronization benchmark, the time is measured it takes to complete the call. After all calls are completed, the raw data is streamed through the jtag interface as well as min, max, mean and standard deviation are again evaluated. Here is the format of the raw data:

```
trial,local_us,latency_us
```

Where:
- `trial`: the number of the published message (0 for the first message with incrementing count)
- `local_us`: the local publish time in microseconds
- `latency`: the latency of the `rcl_publish` call in microseconds

Here it is important to note that the `latency_us` corresponds to the blocking time of the publish call. As this is dependant on the QoS used, the QoS can be adjusted within the `menuconfig`.

## Timer Publishing
Similar to the fast publishing benchmark, the `timer_pub_benchmark.c` file is provided. Here, a publisher is setup that publishes the current synchronized timestamp of the microcontroller with a timer. The timer period can be adjusted within the `menuconfig` (`PUB_TARGET_MS`). The results are structured in the same way as in the fast publishing method.

## Notes
- Make sure the micro-ROS agent is running and reachable before starting the benchmark. The agent can be started using the following command for the serial transport:
```bash
docker run -it --rm -v /dev:/dev --privileged --net=host microros/micro-ros-agent:jazzy serial --dev /dev/ttyUSB0
```
and for the UDP transport:
```bash
docker run -it --rm --net=host microros/micro-ros-agent:jazzy udp4 --port 8888
```
- To get the host-side measurements, make sure to run the relevant script described within the `host` folder. The firmware will wait 5s after a reboot with the agent connected before starting the benchmark. This is to give you enough time to potentially start the host-side subscriber.
- To get even more insight into the timing and scheduling behaviour, tracing can be used. For this:
  - call `idf.py build -D USE_TRACING=ON` so that the tracing is enabled in the firmware.
  - run OpenOCD with the apropriate configuration. This would be for esp32c6: `openocd -f interface/esp_usb_jtag.cfg -f board/esp32c6-builtin.cfg`
  - connect to the OpenOCD server using telnet: `telnet localhost 4444`
  - in the telnet window, call `reset init` to reset the target and then `resume` to start the firmware.
  - to start tracing into a file, call `esp sysview start file://traces/mros_trace_01.SVDat`
  - to stop tracing, call `esp sysview stop`
  - you can import the generated file into the Segger SystemView tool. This will give you a detailed insight into the timing behaviour of the firmware. Note though that the tracing will impact the timing behaviour itself. So this should be used with caution.