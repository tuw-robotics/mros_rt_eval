# Micro-ROS Timing Evaluation

In order to validate if micro-ROS is suitable for real-time applications, this repository provides a set of tests to evaluate the timing performance of different micro-ROS aspects. The tests are designed to be run on Espressif based platforms, specifically the ESP32-C6, and cover both synchronization and publishing performance with different transport and configuration options.

Note that the tests do not result in an actual WCET (Worst Case Execution Time) but provide a good indication of the timing performance of micro-ROS. So the results should be interpreted as a performance evaluation rather than a strict timing guarantee. Please check out the `results` directory for which data is available for the different tests.

The evaluation code is split into two parts:
- **Firmware**: The micro-ROS application running on the microcontroller under test. We currently provide firmware for evaluating synchronization and publishing performance with different transport and configuration options. The firmware is structured in a way that allows easy modification and extension for support of additional transports. A more detailed description of the firmware can be found in the `firmware` directory.
- **Host**: The host-side packages used to evaluate the performance of the micro-ROS application. This includes the `rt_eval` package, which provides a node for evaluating the latency of publishing messages.

## Setup
Both the firmware and host-side packages are encapsulated in DevContainers. This allows for easy setup and execution of the tests without the need for complex environment configurations. The DevContainers are designed to be used with Visual Studio Code, but can also be used with other IDEs that support Docker.

To start testing, open both the firmware and host-side packages in Visual Studio Code. The DevContainers will automatically build the necessary environment and install the required dependencies. Then please refer to the respective README files in the `firmware` and `host` directories for more information on how to run the tests.

The basic flow is as follows:
1. Build and flash the firmware with the dsired configuration under test.
    - Setup the configuration based on the README file in the `firmware` directory. This includes setting the transport type (serial or UDP), the publish rate, and the reliability settings.
    - The firmware can be built and flashed using the following command:
    ```bash
    idf.py build flash
    ```
2. Start the micro-ROS agent on the host.
    - The agent can be started using the following command for the serial transport:
    ```bash
    docker run -it --rm -v /dev:/dev --privileged --net=host microros/micro-ros-agent:jazzy serial --dev /dev/ttyUSB0
    ```
    - or for the UDP transport:
    ```bash
    docker run -it --rm --net=host microros/micro-ros-agent:jazzy udp4 --port 8888
    ```
3. Start the host-side evaluation node.
    - Build the host-side packages using the following command:
    ```bash
    colcon build
    ```
    - Source the workspace:
    ```bash
    source install/setup.bash
    ```
    - The evaluation node can be started using:
    ```bash
    ros2 run rt_eval latency_eval
    ```
4. Push the firmware output into a file.
    - Use `idf.py monitor -p /dev/ttyACM0 > results.log` to output into a file. This file can then easily be cleaned up into a CSV file.
5. Stop the host-side evaluation node.
    - The evaluation node can be stopped using `Ctrl+C`. This will stop the node and save the results to a CSV file.

Check out the `results` directory for the generated CSV files. Here you can also find a more detailed description of the data contained in the CSV files.

## TODO
- Add reference to Bachelor thesis
- Refer to the thesis for which KPIs can be derived from the results