# UWB Package for ROS2


This repository contains two ROS2 packages, `uwb_interfaces` and `uwb-serial-ros2`, which provide an interface for communicating with UWB anchors. These anchors output distance measurements via a serial port. This has been tested on **ROS2 Humble** and is used for distance measurement between UWB antennas.

## Packages

### `uwb_interfaces`
This package defines ROS2 messages and services for UWB data communication. It serves as the interface layer between the UWB serial data and ROS2.

### `uwb-serial-ros2`
This package is responsible for reading data from the serial port and publishing it as ROS2 messages. It parses data from UWB anchors that send distance measurements in the following format:


The serial data is published as `uwb_intefaces/msg/UwbRange` in ROS2.

## Installation

1. Clone the repository:
    ```bash
    git clone https://github.com/DavDori/uwb-ros2.git
    cd uwb-ros2
    ```

3. Install Python dependencies for `uwb-serial-ros2`:
    ```bash
    cd uwb-serial-ros2
    pip install -r requirements.txt
    ```

## Building the packages

Once all dependencies are installed, build the packages in the workspace:

```bash
cd path/to/your/workspace/
colcon build
```

## Running the package

Make sure that all anchors in the UWB network are up and running. Then

1. Connect with your device to the desired UWB antenna via USB;
2. Check the port with the command `ls /dev/tty*`;
3. Modify the configuration file `uwb-serial-ros2/config/params.yaml` with the correct port name;
4. To run the node with the configuration file use the command

    ```
    ros2 launch uwb_serial uwb
    ```