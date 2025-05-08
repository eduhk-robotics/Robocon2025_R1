import socket
import json
import time
import threading

class Client:
    def __init__(self, port=9090):
        self.host = self.get_local_ip()  # Automatically get the current IPv4 address
        self.port = port
        self.client_socket = None
        self.running = True

    def get_local_ip(self):
        """Retrieve the current machine's IPv4 address."""
        try:
            # Create a temporary socket to determine the local IP
            temp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            # Attempt to connect to an external address (we don't actually send data here)
            temp_socket.connect(("8.8.8.8", 80))
            local_ip = temp_socket.getsockname()[0]
            temp_socket.close()
            return local_ip
        except Exception as e:
            print(f"Error getting local IP address: {str(e)}")
            return '127.0.0.1'  # Fallback to localhost if IPv4 cannot be determined

    def connect(self):
        """Connect to the DataCollector server"""
        try:
            self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.client_socket.connect((self.host, self.port))
            print(f"Connected to server at {self.host}:{self.port}")
        except Exception as e:
            print(f"Connection error: {str(e)}")
            self.close_connection()

    def send_request(self, request='get_data'):
        """Send a request to the server and receive data"""
        try:
            if not self.client_socket:
                self.connect()
            self.client_socket.sendall(request.encode('utf-8'))
            
            # Receive data
            data = self.client_socket.recv(1024).decode('utf-8')
            if data:
                try:
                    # Parse JSON data
                    received_data = json.loads(data)
                    return received_data
                except json.JSONDecodeError:
                    print("Invalid JSON data received")
                    return None
            else:
                print("No data received from server")
                return None
                
        except Exception as e:
            print(f"Communication error: {str(e)}")
            self.close_connection()
            return None

    def close_connection(self):
        """Close the socket connection"""
        try:
            if self.client_socket:
                self.client_socket.close()
                self.client_socket = None
                print("Connection closed")
        except Exception as e:
            print(f"Error closing connection: {str(e)}")

    def receive_loop(self):
        """Continuous receiving loop"""
        while self.running:
            data = self.send_request('get_data')
            if data:
                # Format the data as requested
                formatted_output = []
                for motor_id, motor_data in data.items():
                    formatted_output.append(f"Motor {motor_id} speed {motor_data['speed']}")
                # Join all motor data into a single line
                final_output = ", ".join(formatted_output)
                print(f"\n{final_output}")
            time.sleep(1)  # Adjust the delay as needed

def main():
    client = Client()
    try:
        # Start the receiving loop in a separate thread
        receive_thread = threading.Thread(target=client.receive_loop)
        receive_thread.start()

        # Main thread waits for user input to quit
        while True:
            user_input = input("\nEnter 'quit' to stop the client: ")
            if user_input.lower() == 'quit':
                client.running = False
                break
    except KeyboardInterrupt:
        print("\nClient shutdown requested")
        client.running = False
    finally:
        client.close_connection()

if __name__ == "__main__":
    main()