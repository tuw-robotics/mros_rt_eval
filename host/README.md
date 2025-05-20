# Real Time Evaluation - Host Side

This part of the repository is intended to be run on the host side of the micro-ROS communication. It is responsible for subscribing to the micro-ROS published timestamp and for the creation of a CSV file with micro-ROS side timestamp and the host side timestamp. The CSV file is used to evaluate the real timing behaviour of the micro-ROS communication.

The flow to evaluate the timing:
1. The micro-ROS side publishes a synchonized publication timestamp as a Uint64 message.
2. The host side subscribes to the message and stores the received timestamp and the current host time in a CSV file.
3. The CSV file can then be used to evaluate the timing behaviour of the micro-ROS communication.

## How to setup

- Open the `host` folder within the VSCode Dev Container. This will automatically give you the nessesary environment to execute the code.
- Build the project with `colcon build` in the terminal.
- Source the workspace with `source install/setup.bash`.
- For starting the measurement, run the script using `ros2 run rt_eval latency_eval`. Without any parameters, the script will default to the topic `latency_usec` with a `best effort` QoS profile and a depth of `10`.
  - Use `--ros-args -p reliability:=reliable -p depth:=20 -p topic:=new_topic` to change the QoS profile and the topic name.
  - You can also adjust the output file name with `--ros-args -p output_csv:=new_file.csv`.