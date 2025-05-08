import rclpy
import socket
import threading
import json
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

class DataCollector(Node):
    def __init__(self):
        super().__init__('data_collector')
        
        # Data storage
        self.collected_data = {}
        self.data_lock = threading.Lock()
        
        # Socket server configuration
        self.host = '172.19.249.33'  # Listen on all interfaces
        self.port = 9090
        self.max_clients = 5
        self.running = True

        # Initialize ROS2 subscriptions
        self.init_subscriptions()
        
        # Start socket server thread
        self.server_thread = threading.Thread(target=self.socket_server)
        self.server_thread.start()
        
        self.get_logger().info(f'Data Collector initialized | Socket server running on {self.host}:{self.port}')

    def init_subscriptions(self):
        """Initialize ROS2 topic subscriptions"""
        self.create_subscription(
            Float32MultiArray,
            'damiao',
            self.damiao_callback,
            10)

    def damiao_callback(self, msg):
        """Process messages from damiao topic"""
        try:
            with self.data_lock:
                motor_id = int(msg.data[0])
                speed = float(msg.data[2])
                
                self.collected_data[motor_id] = {
                    'speed': speed,
                    'timestamp': self.get_clock().now().nanoseconds
                }
                
                self.get_logger().debug(f"Motor {motor_id} speed: {speed} dps")
                
        except Exception as e:
            self.get_logger().error(f"Error processing damiao message: {str(e)}")

    def socket_server(self):
        """Main socket server loop"""
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            s.bind((self.host, self.port))
            s.listen(self.max_clients)
            
            while self.running:
                try:
                    client_socket, addr = s.accept()
                    self.get_logger().info(f"New client connected: {addr}")
                    client_thread = threading.Thread(
                        target=self.handle_client,
                        args=(client_socket,)
                    )
                    client_thread.start()
                except Exception as e:
                    if self.running:
                        self.get_logger().error(f"Socket error: {str(e)}")

    def handle_client(self, client_socket):
        """Handle individual client connection"""
        try:
            while self.running:
                # Wait for request
                request = client_socket.recv(1024).decode('utf-8').strip()
                
                if not request:
                    break
                    
                if request == 'get_data':
                    # Serialize and send data
                    with self.data_lock:
                        data_copy = self.collected_data.copy()
                    serialized = json.dumps(data_copy)
                    client_socket.sendall(serialized.encode('utf-8'))
                    
                elif request == 'quit':
                    break
                    
        except Exception as e:
            self.get_logger().error(f"Client error: {str(e)}")
        finally:
            client_socket.close()
            self.get_logger().info("Client disconnected")

    def destroy_node(self):
        """Cleanup resources"""
        self.running = False
        # Create a dummy connection to unblock accept()
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                s.connect((self.host, self.port))
                s.close()
        except:
            pass
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    collector = DataCollector()
    
    try:
        rclpy.spin(collector)
    except KeyboardInterrupt:
        collector.get_logger().info('Shutting down data collector')
    finally:
        collector.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
