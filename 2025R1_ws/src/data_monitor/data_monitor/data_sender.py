import rclpy
import socket
import threading
import json
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray


def get_local_ip():
    """
    Get the current connected IPv4 address of the host.

    Returns:
        str: Local IPv4 address of the current machine.
    """
    try:
        # Connect to a known public address (Google's public DNS server)
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as temp_socket:
            # Use Google's DNS server to determine the IP address
            temp_socket.connect(("8.8.8.8", 80))
            local_ip = temp_socket.getsockname()[0]
        return local_ip
    except Exception as e:
        # Default to localhost if auto-detection fails
        print(f"[ERROR] Could not determine local IP address automatically: {e}")
        return "127.0.0.1"  # Fallback to localhost


class DataCollector(Node):
    """A ROS 2 node that collects data from a topic and serves it via a socket server."""
    def __init__(self):
        super().__init__('data_collector')
        
        # Dynamically detect host IPv4 address
        self.host = get_local_ip()  # Automatically detect connected IPv4 address
        self.port = 9090  # Port for the socket server
        self.max_clients = 5
        self.running = True

        # Data storage for received messages
        self.collected_data = {}
        self.data_lock = threading.Lock()

        # Initialize ROS 2 topic subscriptions
        self.init_subscriptions()
        
        # Start socket server in a separate thread
        self.server_thread = threading.Thread(target=self.socket_server, daemon=True)
        self.server_thread.start()

        self.get_logger().info(f'Data Collector initialized. Socket server running on {self.host}:{self.port}')

    def init_subscriptions(self):
        """Initialize ROS 2 topic subscriptions."""
        self.create_subscription(
            Float32MultiArray,  # Message type
            'damiao_status',           # Topic name
            self.damiao_callback,  # Callback function
            10  # QoS (queue size)
        )
        self.get_logger().info("Subscription to topic 'damiao' initialized.")

    def damiao_callback(self, msg):
        """Callback function for topic 'damiao'.

        Args:
            msg (Float32MultiArray): The message received from the 'damiao' topic.
        """
        try:
            with self.data_lock:
                motor_id = int(msg.data[0])
                speed = float(msg.data[2])
                
                # Store the received data in the dictionary
                self.collected_data[motor_id] = {
                    'speed': speed,
                    'timestamp': self.get_clock().now().nanoseconds
                }
                
                self.get_logger().info(f"Motor {motor_id} speed: {speed} dps")
                
        except Exception as e:
            self.get_logger().error(f"Error processing 'damiao' message: {str(e)}")

    def socket_server(self):
        """Main function for the socket server."""
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server_socket:
            server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            server_socket.bind((self.host, self.port))
            server_socket.listen(self.max_clients)
            
            while self.running:
                try:
                    # Accept client connections
                    client_socket, addr = server_socket.accept()
                    self.get_logger().info(f"Client connected: {addr}")

                    # Create a thread to handle this client
                    client_thread = threading.Thread(
                        target=self.handle_client,
                        args=(client_socket,),
                        daemon=True  # Automatically close thread when main thread ends
                    )
                    client_thread.start()
                except Exception as e:
                    if self.running:
                        self.get_logger().error(f"Socket error: {str(e)}")

    def handle_client(self, client_socket):
        """Handle a single client connection.

        Args:
            client_socket (socket.socket): The client socket.
        """
        try:
            while self.running:
                # Receive request from the client
                request = client_socket.recv(1024).decode('utf-8').strip()
                
                if not request:
                    break  # If the client disconnected
                
                # Handle "get_data" command
                if request == 'get_data':
                    with self.data_lock:
                        data_copy = self.collected_data.copy()
                    serialized = json.dumps(data_copy)
                    client_socket.sendall(serialized.encode('utf-8'))
                
                # Handle "quit" command
                elif request == 'quit':
                    break

                # Handle unknown commands
                else:
                    client_socket.sendall(b'Unknown command.\n')
        
        except Exception as e:
            self.get_logger().error(f"Client error: {str(e)}")
        
        finally:
            # Close the client connection
            client_socket.close()
            self.get_logger().info("Client disconnected")

    def destroy_node(self):
        """Clean up resources when the node shuts down."""
        self.running = False
        # Create a dummy connection to unblock the accept() call
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as dummy:
                dummy.connect((self.host, self.port))
                dummy.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    """Main entry point for the ROS 2 node."""
    rclpy.init(args=args)
    collector = DataCollector()
    
    try:
        rclpy.spin(collector)  # Spin the node to process events
    except KeyboardInterrupt:
        # Handle shutdown when Ctrl+C is pressed
        collector.get_logger().info('Shutting down Data Collector node.')
    finally:
        collector.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()