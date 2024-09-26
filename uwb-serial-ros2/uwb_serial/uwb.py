import rclpy
import serial
from rclpy.node import Node
from rclpy.exceptions import InvalidParameterValueException
from rclpy.signals import SignalHandlerOptions
import time
import csv
import os
import numpy as np
from uwb_interfaces.msg import UwbRange
from ament_index_python.packages import get_package_share_directory


# 
def parseSerialDataUWB(serial_data):
    '''
    Function to parse the serial data from a UWB network
    '''    
    if not serial_data.strip():  # Skip empty lines
        return None
    
    sequence_number, id_self, id_other, distance = serial_data.split()
    parsed_data = {
        'seq': int(sequence_number),
        'id_self': int(id_self),
        'id_other': int(id_other),
        'range': float(distance)
    }
    return parsed_data

class UWB(Node):

    def __init__(self):
        super().__init__('uwb_serial')

        self.declare_parameter('usb_port', '/dev/ttyACM0')
        self.declare_parameter('timer_rate_s', 0.02)
        self.declare_parameter('boudrate', 115200)
        self.declare_parameter('topic_name', 'uwb/range')
        self.declare_parameter('anchors_calib_files', ['none'])

        usb_port_name = self.get_parameter('usb_port').get_parameter_value().string_value
        topic_name = self.get_parameter('topic_name').get_parameter_value().string_value
        boudrate = self.get_parameter('boudrate').get_parameter_value().integer_value
        rate = self.get_parameter('timer_rate_s').get_parameter_value().double_value
        self.calib_filenames_ = self.get_parameter('anchors_calib_files').get_parameter_value().string_array_value
        # initialize
        self.calib_data_ = {}

        # Use the specified calibration files to generate a lookup table to estimate the offset
        self.loadCalibrationFiles()

        self.pub_ = self.create_publisher(UwbRange, topic_name, 10)

        self.ser_ = serial.Serial(
            port=usb_port_name,
            baudrate=boudrate
            )
        if(self.ser_.is_open):
            self.ser_.close()
        self.ser_.open()
        # Create a timer to periodically check the serial port
        self.timer_ = self.create_timer(rate, self.timerCallback)
        time.sleep(0.01)

    

    def timerCallback(self):
        try:
            while self.ser_.in_waiting > 0:
                data = self.ser_.readline().decode('utf-8').strip()
                if not data:
                    self.get_logger().warn(f'No data received from serial port.')
                    return

                parsed_data = parseSerialDataUWB(data)
                range_compensated = self.compensateOffset(parsed_data['range'], parsed_data['id_other'])
                
                if range_compensated is None:
                    self.get_logger().warn(f"Unable to compensate range for ID {parsed_data['id_other']}.")
                    return
                
                msg = UwbRange()
                msg.range = range_compensated
                msg.source = parsed_data['id_self']
                msg.target = parsed_data['id_other']
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = 'uwb'+ str(parsed_data['id_self'])

                self.pub_.publish(msg)

        except Exception as err:
            self.get_logger().warning(f'Warning: "{err=}".')


    def loadCalibrationFiles(self):
        '''
        Load .csv files relative to the offset compensation for the UWB. Note that every UWB has unique 
        offset characterization. 
        Expecting the following rules:
            - the first row contains the id of the other anchor
            - the second row contains the offsets
            - the third row contains the groundtruth at which the offset were calculated
        It outputs a dictionary with offsets, groundtruths, and reference values that can be accessed via
        the antenna id used as key.
        '''
        if self.calib_filenames_ == []:
            self.get_logger().info(f'No calibration files were specified. Offsets are not be compensated.')
            return
        for filename in self.calib_filenames_:
            filepath = os.path.join(
                get_package_share_directory('uwb_serial'),
                'config',
                filename,
                )
            
            with open(filepath, newline='') as csvfile:
                csvreader = csv.reader(csvfile, delimiter=',', quotechar='|')
                data = []
                for row in csvreader:
                    data.append(row)

                id = int(data[0][0])
                offsets = np.array(data[1], dtype=float)
                groundtruths = np.array(data[2], dtype=float)
                references = groundtruths - offsets 
                self.calib_data_[id] = {
                    'offsets': offsets,
                    'groundtruths': groundtruths,
                    'references': references
                }


    def compensateOffset(self, distance, id):
        '''
        Uses the calibration data to compensate the distance measurement form a UWB antenna with
        the specified id.
        '''
        if id not in self.calib_data_:
            self.get_logger().warning(f'Trying to access the calibration data with the id {id} that has not been registered.')
            return None
        references = self.calib_data_[id]['references']
        offsets = self.calib_data_[id]['offsets']

        # Find the index of the reference closest to the input distance
        closest_index = np.argmin(np.abs(references - distance))
        offset = offsets[closest_index]
        # Return the value compensated
        return distance - offset
    

    def shutdown(self):
        '''
        Closes the serial port
        '''
        self.get_logger().info(f'Node shutdown: closing serial {self.ser_.port}')
        self.ser_.close()


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)

    node = UWB()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.shutdown()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
