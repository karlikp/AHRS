import paho.mqtt.client as mqtt
import os
import queue

class Mqtt:

    # Initializes the object for sending files via MQTT.
    def __init__(self, broker, port=1883, client_id=None):
        self.broker = broker
        self.port = port
        self.client_id = client_id or f"mqtt_client_{os.getpid()}"
        self.client = mqtt.Client(client_id=self.client_id)

        # Set the callback for connection
        self.client.on_connect = self.on_connect
        self.data_queue = queue.Queue()

    # Callback called after successfully connecting to the MQTT broker.
    def on_connect(self, client, userdata, flags, rc):
        print(f"Connected with result code {rc}")
        self.send_from_queue()

    # Sends data from the queue via MQTT in bytearray format.
    def send_from_queue(self):
        """
        Sends data from the queue via MQTT as bytearray.
        """
        while not self.data_queue.empty():
            try:
                data = self.data_queue.get()

                # list are convert to string, then string to bytearray
                if isinstance(data, list):
                    data = str(data).encode('utf-8')  
                elif isinstance(data, dict):
                    data = str(data).encode('utf-8')  
                elif isinstance(data, str):
                    data = data.encode('utf-8')  

                # Wsend as bytearray
                self.client.publish(self.topic, data)
                #print(f"Sent data: {data}")
            except Exception as e:
                print(f"Error sending data: {e}")
                break

    def connect_and_send(self):
        """
        Connects to the MQTT broker and starts sending data.
        """
        try:
            self.client.connect(self.broker, self.port, 60)
            self.client.loop_start()  # Start the communication loop
        except Exception as e:
            print(f"Error connecting to the broker: {e}")

    # Method to add data to the queue
    def add_to_queue(self, data):
        """
        Adds data to the queue for sending.
        """
        self.data_queue.put(data)