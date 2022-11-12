# Robotic_HW - Arnav Malawade

## Repository Structure
- src/sensor_service: ROS2 package containing sensor servers and client.
- src/sensor_interfaces: ROS2 package containing relevant messages and service interfaces for handling sensor data.

## Explanation of Solution

### Servers
I implemented two sensor servers, one for a 2000Hz sensor with 1ms latency and one for a 4000Hz sensor with 3ms latency.

The 2000Hz sensor server uses a timer that requests data from the sensor every 2ms. The server requests 2 samples from the sensor each time. 
This enables the server to have an effective rate of 1 sample / ms. The sensor data is collected into a local buffer and processed using kalman filtering to smooth the data and track the hidden state of the sensor features. The most recent filtered state estimate is returned to the client when the server is called. For the 4000Hz sensor, the timer requests data from the sensor every 4ms, and requests 4 samples each time, thus also achieving a 1 sample / ms effective rate.

### Client 
The client is configured to call each server once every 2ms and request the latest filtered sensor data. I implemented a custom service interface called PollSensor[1,2].srv to return the array of sensor data from the server. The client then publishes the most recent data received from each sensor to a topic using SensorData.msg. The client is configured to send asynchronous requests to each server to avoid blocking and deadlocks. Each server is called independently; if a server response is delayed the client publishes the most recently received prior data from the corresponding sensor until the server response updates it (this is self.last_resp_[1,2]). 

### Limitations
I have not used ROS2 before this week and learned a great deal during my process of configuring it and setting up these interfaces for the first time. I was able to resolve most of the bugs I encountered, but one remaining bug is that the code does not properly instantiate the second client for communicating with the second sensor's server. The code works with one client and one sensor server perfectly well, but I was not able to get the second server to run along with the first one in the time I had to implement this solution. 

### If I spent more time on this project in the future, I would:
1. Implement a more future-proof client and service definition using a base class for the sensor server that the two individual sensor servers would use with slightly different configurations. 
2. Use the args passed to the main() of each service and client as a method to set up their parameters (e.g., networking, sampling rate, number of samples to request, etc.) via a centralized configuration file.
3. Build a launch file for spinning all of the processes at the same time. 
4. Identify real-world sensing requirements to align the polling and publishing rates accordingly.

