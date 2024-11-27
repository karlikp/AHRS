import paho.mqtt.client as mqtt
import base64
import os

class MqttFileSender:
    def __init__(self, broker, port=1883, topic="file_transfer/topic", client_id=None):
        """
        Initializes the object for sending files via MQTT.
        
        :param broker: MQTT broker address.
        :param port: MQTT broker port.
        :param topic: The topic to which files will be sent.
        :param client_id: Optionally, a unique client identifier.
        """
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
        print(f"Connected to the MQTT broker. Return code: {rc}")
        # After connection, start sending the file
        self.send_file()

    def send_file(self, file_path):
        """
        Sends a file via MQTT in base64 format.
        
        :param file_path: Path to the file to be sent.
        """
        try:
            # Open the file in binary mode and convert it to base64
            with open(file_path, "rb") as file:
                file_data = file.read()
                encoded_file = base64.b64encode(file_data).decode('utf-8')  # Convert to base64
                print(f"File '{file_path}' encoded to base64.")

            # Send the encoded data via MQTT
            self.client.publish(self.topic, encoded_file)
            print(f"File has been sent to topic: {self.topic}")
        except Exception as e:
            print(f"Error sending the file: {e}")

    def connect(self):
        """
        Connects to the MQTT broker.
        """
        try:
            self.client.connect(self.broker, self.port, 60)
            self.client.loop_start()  # Start the communication loop
        except Exception as e:
            print(f"Error connecting to the broker: {e}")

# Example usage of the class
if __name__ == "__main__":
    # Create an instance of the MqttFileSender class
    sender = MqttFileSender(broker="mqtt.eclipse.org", topic="file_transfer/topic")

    # Connect to the MQTT broker and send the file
    sender.connect()

    # Path to the file to be sent
    file_path = "path_to_your_file.txt"  # Change to the appropriate file path
    sender.send_file(file_path)
