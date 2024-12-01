import paho.mqtt.client as mqtt
import base64
import os

class Mqtt:
    def __init__(self, broker, file_path, topic, port=1883, client_id=None):
        """
        Initializes the object for sending files via MQTT.
        
        :param broker: MQTT broker address.
        :param port: MQTT broker port.
        :param topic: The topic to which files will be sent.
        :param client_id: Optionally, a unique client identifier.
        """
        self.file_path = file_path
        self.broker = broker
        self.port = port
        self.topic = topic
        self.client_id = client_id or f"mqtt_client_{os.getpid()}"
        self.client = mqtt.Client(client_id=self.client_id)
        

        # Set the callback for connection
        self.client.on_connect = self.on_connect

    def on_connect(self, client, userdata, flags, rc):
        """
        Callback called after successfully connecting to the MQTT broker.
        :param client: The MQTT client.
        :param userdata: Additional data.
        :param flags: Connection flags.
        :param rc: The connection return code.
        """
        self.send_file()

    def send_file(self, chunk_size=256*1024):  # Domyślnie 256 KB na fragment
        """
        Sends a file via MQTT in chunks in base64 format.
        
        :param chunk_size: Size of each chunk in bytes.
        """
        if not os.path.exists(self.file_path):
            print(f"Error: File '{self.file_path}' does not exist.")
            return

        try:
            with open(self.file_path, "rb") as file:
                chunk_index = 0
                while chunk := file.read(chunk_size):  # Czytaj plik w kawałkach
                    encoded_chunk = base64.b64encode(chunk).decode('utf-8')  # Koduj fragment
                    self.client.publish(self.topic, encoded_chunk)  # Wyślij fragment
                    #print(f"Sent chunk {chunk_index} of size {len(chunk)} bytes.")
                    chunk_index += 1
            print(f"File has been sent in chunks on {self.topic}")
        except Exception as e:
            print(f"Error sending the file: {e}")

    def connect(self):
        """
        Connects to the MQTT broker.
        """
        try:
            self.client.connect(self.broker, self.port, 60)
            #self.client.loop_start()  # Start the communication loop
        except Exception as e:
            print(f"Error connecting to the broker: {e}")
        
    def clear_file(self):
        open(self.file_path, "w").close()

    def get_file_path(self):
        return self.file_path

# # Example usage of the class
# if __name__ == "__main__":
#     # Create an instance of the Mqtt class
#     sender = Mqtt(broker="mqtt.eclipse.org", topic="file_transfer/topic")

#     # Connect to the MQTT broker and send the file
#     sender.connect()

#     # Path to the file to be sent
#     file_path = "/home/karol/Desktop/repos/SLAM/data/package/bmp388.txt"  # Change to the appropriate file path
#     sender.send_file(file_path)
