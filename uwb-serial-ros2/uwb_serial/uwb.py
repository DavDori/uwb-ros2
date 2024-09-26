import rclpy
import serial
from rclpy.node import Node
from rclpy.exceptions import InvalidParameterValueException
from rclpy.signals import SignalHandlerOptions
import time 
from uwb_interfaces.msg import UwbRange
    

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
        'dist': float(distance)
    }
    return parsed_data

class UWB(Node):

    def __init__(self):
        super().__init__('uwb_serial')

        self.declare_parameter('usb_port', '/dev/ttyACM0')
        self.declare_parameter('timer_rate_s', 0.02)
        self.declare_parameter('boudrate', 115200)
        self.declare_parameter('topic_name', 'uwb/range')

        usb_port_name = self.get_parameter('usb_port').get_parameter_value().string_value
        topic_name = self.get_parameter('topic_name').get_parameter_value().string_value
        boudrate = self.get_parameter('boudrate').get_parameter_value().integer_value
        rate = self.get_parameter('timer_rate_s').get_parameter_value().double_value

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
                    self.get_logger().warn(f'no data recieved')
                    return

                parsed_data = parseSerialDataUWB(data)
                msg = UwbRange()
                msg.range = parsed_data['dist']
                msg.source = parsed_data['id_self']
                msg.target = parsed_data['id_other']
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = 'uwb'+ str(parsed_data['id_self'])

                self.pub_.publish(msg)
        except Exception as err:
            self.get_logger().warning(f'Warning: "{err=}".')


    def shutdown(self):
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
