#Author: Arnav
#Last updated: 11/11/2022

from sensor_interfaces.srv import PollSensor1, PollSensor2 
from sensor_interfaces.msg import SensorData
import rclpy
from rclpy.node import Node

class SensorServiceClient(Node):

    def __init__(self):
        super().__init__('sensor_service_client')
        self.get_logger().info('Starting client')

        #publisher for sensor topic
        self.publisher = self.create_publisher(SensorData, 'topic', 10)
        timer_period = 0.002  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Sensor client
        self.cli1 = self.create_client(PollSensor1, 'request_sensor_data1')
        self.cli2 = self.create_client(PollSensor2, 'request_sensor_data2')
        self.req1 = PollSensor1.Request()
        self.req2 = PollSensor2.Request()
        while not self.cli1.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service1 not available, waiting again...')
        while not self.cli2.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service2 not available, waiting again...')
        self.last_resp_s1, self.last_resp_s2 = None, None
        self.waiting_for_s1, self.waiting_for_s2 = False, False


    def send_sensor_data_request(self):
        if not self.waiting_for_s1: #if not waiting for response, make a new request
            self.future1 = self.cli1.call_async(self.req1)
            self.waiting_for_s1 = True
        else:
            if self.future1.done(): #while waiting for response, check if response is ready
                try:
                    self.last_resp_s1 = self.future1.result()
                    self.waiting_for_s1 = False
                except Exception as e:
                    self.get_logger().info('Service call failed %r' % (e,))

        if not self.waiting_for_s2: 
            self.future2 = self.cli2.call_async(self.req2)
            self.waiting_for_s2 = True
        else:
            if self.future2.done(): 
                try:
                    self.last_resp_s2 = self.future2.result()
                    self.waiting_for_s2 = False
                except Exception as e:
                    self.get_logger().info('Service call failed %r' % (e,))


    def timer_callback(self):
        '''poll sensor and publish response to topic'''
        arr = SensorData()
        self.send_sensor_data_request()
        if self.last_resp_s1 != None:
            arr.sensor1_array = self.last_resp_s1.sensor_data
        if self.last_resp_s2 != None:
            arr.sensor2_array = self.last_resp_s2.sensor_data            
        self.publisher.publish(arr)


def main(args=None):
    rclpy.init(args=args)
    minimal_client = SensorServiceClient()
    rclpy.spin(minimal_client)

    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()