#Author: Arnav
#Last updated: 11/11/2022

from sensor_interfaces.srv import PollSensor2 
import rclpy
from rclpy.node import Node
import socket
import numpy as np
from pykalman import KalmanFilter


class Sensor2ServiceServer(Node):
    '''ROS2 service server for retrieving data from a 3-DOF sensor with 4000Hz sampling rate and 3ms latency'''

    def __init__(self):
        '''Initialize service. Open socket to sensor. Create timer.'''
        super().__init__('sensor_service_server2')
        #sensor parameters
        self.srv = self.create_service(PollSensor2, 'request_sensor_data2', self.client_callback)
        timer_period = 0.004  # seconds
        self.samples_per_call = 4 #num samples to request from sensor on each call
        self.timer = self.create_timer(timer_period, self.poll_sensor_callback)

        #networking
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM) # Create a TCP/IP socket
        server_address = ('127.0.0.1', 10000)
        print('connecting to {} port {}'.format(*server_address))
        self.sock.connect(server_address)
        self.DOF = 6 # Degrees of freedom
        
        #data filtering
        self.sensor_buffer = np.zeros((10, self.DOF), dtype=np.float32) #raw data buffer from sensor
        self.kf = KalmanFilter(n_dim_state=self.DOF, n_dim_obs=self.DOF)
        self.filtered_sensor_data = np.zeros((self.DOF), dtype=np.float32) #filtered data from sensor


    def poll_sensor_callback(self):
        '''Send request to sensor. Receive data. Append to buffer.'''
        message_string = str(self.samples_per_call)
        message = message_string.encode()
        self.sock.sendall(message)
        byte_data = self.sock.recv(10000)
        data =  np.frombuffer(byte_data)
        self.sensor_buffer = np.roll(self.sensor_buffer, -self.samples_per_call, axis=0)
        self.sensor_buffer[-self.samples_per_call:, :] = data.reshape((self.samples_per_call, self.DOF))
        self.filtered_sensor_data = self.kalman_filter_sensor_data(self.sensor_buffer).astype(np.float32) #most recent filtered data


    def client_callback(self, request, response):
        '''Return most recent filtered sensor data.'''
        response.sensor_data = self.filtered_sensor_data # assign outputs
        self.get_logger().info('sending message to client')
        return response


    def kalman_filter_sensor_data(self, data):
        '''Applies KalmanFilter.smooth() from pykalman to filter sensor data'''
        means, covariances = self.kf.smooth(data)
        return means[-1, :] #return most recent filtered data

    
    def cleanup(self):
        print('closing socket')
        self.sock.close()


def main(args=None):
    rclpy.init(args=args)
    minimal_service = Sensor2ServiceServer()
    rclpy.spin(minimal_service)

    minimal_service.cleanup()
    rclpy.shutdown()


if __name__ == '__main__':
    main()