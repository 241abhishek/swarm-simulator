import socket
import select
import json 
import time
import time
import errno
import queue

class Bot_Server():
    def __init__(self, hostname, port, num_robots):
        self.hostname = hostname
        self.port = port
        self.num_robots = num_robots
        self.message_queues = {} # Dictionary to store messages to send to each socket
    
    def start(self):
        print("BOT SERVER: Started") # TODO: Remove
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM) # Open socket
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1) # Allow reuse of addresses (prevents Address Already Bound errors)
        self.server_socket.setblocking(0) # Make socket non-blocking -- instead of waiting for each operation to complete one by one, the program can check the status of multiple sockets simultaneously
        try:
            self.server_socket.bind((self.hostname, self.port)) 
        except socket.error as e:
            print("Bot Server failed to bind: ", e)
        
        self.server_socket.listen(self.num_robots) # Allow up to num_robots client connections

        # List of sockets to monitor for reading (receiving data) and writing (sending data)
        self.read_list = [self.server_socket]
        self.write_list = []

    def recv(self, swarm):
        print("BOT SERVER: Receiving...") # TODO: Remove
        # Use `select` to identify which sockets are ready for I/O
        # This operation is blocking -- if no sockets are ready, it will wait until one is -- provide a small timeout to give the server time to accept any incoming client connections
        # Note: The timeout actually should not have an effect on performance until the server is running after the clients are done, since sockets will not be ready for I/O at this point
        readable, writable, exceptional = select.select(self.read_list, self.write_list, [], 0.2) 

        received_data = []

        # Handle receiving from clients
        for s in readable:
            if s is self.server_socket:
                # The server socket is ready to accept a client connection.
                client_socket, client_address = s.accept()
                client_socket.setblocking(0)

                self.read_list.append(client_socket)
                self.message_queues[client_socket] = queue.Queue()

                print("BOT SERVER: Client connected") # TODO: Remove
            else:
                # A client (whose connection has already been established) has sent data

                message = s.recv(1024) # TODO: this buffer size might need to scale with num_robots

                if message: 
                    # The message was not None -- send a response back to the client
                        message = message.decode("utf-8")
                        message = json.loads(message)

                        received_data.append(message)

                        client_id = message["id"]
                        client_function = message["function"]

                        if client_function == 3:
                            # get_clock
                            response = {
                                "response": swarm[client_id].clock 
                            }
                        elif client_function == 4:
                            response = {
                                "response": (swarm[client_id].x, swarm[client_id].y, swarm[client_id].theta)
                            }
                        elif client_function == 6:
                            response = {
                                "response": swarm[client_id].message_buffer
                            }
                        else:
                            response = {
                                "response": 1
                            }

                        response = json.dumps(response)
                        response = response.encode("utf-8")

                        self.message_queues[s].put(response) # Add response to queue of messages to be sent to client s

                        if s not in self.write_list:
                            self.write_list.append(s) # Add output channel for response
    
                else:
                    # No more data from the client -- close the connection
                    print("SOCKET REMOVED!!")
                    self.read_list.remove(s)
                    if s in self.write_list:
                        self.write_list.remove(s)
                    del self.message_queues[s]
                    s.close()

        # Handle sending to clients
        for s in writable:
            try:
                # Get next message to send to the socket (nonblocking)
                next_msg = self.message_queues[s].get_nowait()
            except queue.Empty:
                # No messages are left, so the socket is no longer writable
                self.write_list.remove(s)
            else:
                # If message was available (queue was non-empty), send message
                s.send(next_msg)
        
        # Handle exceptions
        for s in exceptional:
            print("EXCEPTIONAL LIST")
            self.read_list.remove(s)
            if s in self.write_list:
                self.write_list.remove(s)
            del self.message_queues[s]
            s.close()

        return received_data
    
    def stop(self):
        print("BOT SERVER Stopped...")
        self.server_socket.close()
    
class Bot_Client():
    def __init__(self, hostname, port):
        self.hostname = hostname
        self.port = port
    
    def start(self):
        print("BOT CLIENT: Started.") # TODO: Remove
        self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.client_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

        try:
            self.client_socket.connect((self.hostname, self.port))
        except socket.error as e:
            if e.errno == errno.EINPROGRESS:
                # `[Erno 36] Operation now in progress` occurs when a nonblocking socket operation (e.g. connect) is re-attempted before a previous attempt is finished
                time.sleep(0.05) # Allow for a small delay before retrying
            else:
                print("BOT CLIENT socket failed to connect: ", e) # TODO: Remove, otherwise clients error after server disconnects
    
    def send(self, data):
        # NOTE: the client is actually blocking -- it will wait for a response from the server before doing things
        print("BOT CLIENT: sending")
        # Process data
        data = json.dumps(data)
        data = data.encode("utf-8")

        # Send data
        self.client_socket.sendall(data)

        # Receive response from simulator
        response = self.client_socket.recv(1024)

        # Need to keep the socket open for a tiny bit
        time.sleep(0.1) 

        if not response:
            print("CLIENT STOPPED!!!") # TODO: Remove
            self.client_socket.close()
            # response = None
        else:
            response = response.decode("utf-8")
            response = json.loads(response)

        return response
        
    def stop(self):
        print("BOT CLIENT Stopped") #TODO: Remove
        self.client_socket.close()