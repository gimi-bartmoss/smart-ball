import socket
import threading
import time

# --- Configuration ---
ESP32_HOST = "192.168.4.1"
ESP32_PORT = 8080

LOCAL_HOST = "localhost"
LOCAL_PORT = 8081

# List to keep track of connected local clients
local_clients = []
clients_lock = threading.Lock()

def connect_to_esp32():
    """
    Connects to the ESP32 server with retry logic.
    Returns the socket object upon successful connection.
    """
    while True:
        try:
            print(f"Attempting to connect to ESP32 at {ESP32_HOST}:{ESP32_PORT}...")
            esp32_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            esp32_socket.connect((ESP32_HOST, ESP32_PORT))
            print("Successfully connected to ESP32.")
            return esp32_socket
        except socket.error as e:
            print(f"Failed to connect to ESP32: {e}. Retrying in 5 seconds...")
            time.sleep(5)

def handle_esp32_data(esp32_socket):
    """
    Reads data from the ESP32 and relays it to all connected local clients.
    """
    while True:
        try:
            data = esp32_socket.recv(1024)
            if not data:
                print("ESP32 connection closed. Reconnecting...")
                esp32_socket.close()
                esp32_socket = connect_to_esp32()
                continue

            disconnected_clients = []
            with clients_lock:
                for client_socket in local_clients:
                    try:
                        client_socket.sendall(data)
                    except socket.error:
                        # Client disconnected, mark for removal
                        disconnected_clients.append(client_socket)
            
            # Remove disconnected clients outside the iteration
            if disconnected_clients:
                with clients_lock:
                    for client in disconnected_clients:
                        if client in local_clients:
                            local_clients.remove(client)
                        print(f"Local client disconnected and removed.")

        except socket.error:
            print("Lost connection to ESP32. Reconnecting...")
            esp32_socket.close()
            esp32_socket = connect_to_esp32()

def handle_local_client(client_socket):
    """
    Handles a single local client connection.
    The main purpose is to detect when a client disconnects.
    """
    with clients_lock:
        local_clients.append(client_socket)
    
    peer_name = client_socket.getpeername()
    print(f"New local client connected: {peer_name}")
    
    try:
        # This loop just waits for the client to close the connection
        while True:
            data = client_socket.recv(1024)
            if not data:
                break
    except socket.error:
        pass # Connection closed
    finally:
        with clients_lock:
            if client_socket in local_clients:
                local_clients.remove(client_socket)
        client_socket.close()
        print(f"Local client disconnected: {peer_name}")


def start_local_server():
    """
    Starts the local TCP server to accept connections from other Python scripts.
    """
    local_server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    local_server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    local_server.bind((LOCAL_HOST, LOCAL_PORT))
    local_server.listen(5)
    print(f"Local relay server listening on {LOCAL_HOST}:{LOCAL_PORT}")

    while True:
        client_socket, addr = local_server.accept()
        client_thread = threading.Thread(target=handle_local_client, args=(client_socket,))
        client_thread.daemon = True
        client_thread.start()

if __name__ == "__main__":
    # First, establish a connection to the ESP32
    esp32_socket = connect_to_esp32()

    # Start a thread to continuously read from ESP32 and relay data
    esp32_thread = threading.Thread(target=handle_esp32_data, args=(esp32_socket,))
    esp32_thread.daemon = True
    esp32_thread.start()
    
    # In the main thread, start the local server to accept local clients
    start_local_server()
