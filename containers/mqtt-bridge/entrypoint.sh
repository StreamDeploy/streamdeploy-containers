#!/usr/bin/env bash
set -euo pipefail
source /opt/ros/${ROS_DISTRO}/setup.bash

python3 - << 'PY'
import os, ssl, json, threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import paho.mqtt.client as mqtt

host=os.getenv("MQTT_HOST","mqtt")
port=int(os.getenv("MQTT_PORT","1883"))
tls=os.getenv("MQTT_TLS","false")=="true"
user=os.getenv("MQTT_USERNAME") or None
pwd=os.getenv("MQTT_PASSWORD") or None
sub_topic=os.getenv("MQTT_SUB_TOPIC","robot/cmd")
pub_topic=os.getenv("MQTT_PUB_TOPIC","robot/chatter")
ros_sub=os.getenv("ROS_SUBSCRIBE","/chatter")
ros_pub=os.getenv("ROS_PUBLISH","/cmd")
qos=int(os.getenv("QOS","0"))

class Bridge(Node):
    def __init__(self, client):
        super().__init__('ros2_mqtt_bridge')
        self.client = client
        self.pub = self.create_publisher(String, ros_pub, 10)
        self.sub = self.create_subscription(String, ros_sub, self.on_ros_msg, 10)
    def on_ros_msg(self, msg):
        try:
            self.client.publish(pub_topic, msg.data, qos=qos)
        except Exception as e:
            self.get_logger().error(f"MQTT publish failed: {e}")

def on_connect(client, userdata, flags, rc):
    client.subscribe(sub_topic, qos=qos)
def on_message(client, userdata, msg):
    node: Bridge = userdata['node']
    m = String(); m.data = msg.payload.decode('utf-8', errors='ignore')
    node.pub.publish(m)

client = mqtt.Client(userdata={})
if user: client.username_pw_set(user, pwd)
if tls:
    client.tls_set(cert_reqs=ssl.CERT_NONE); client.tls_insecure_set(True)
client.on_connect = on_connect
client.on_message = on_message

rclpy.init()
node = Bridge(client)
client._userdata = {'node': node}

def mqtt_loop():
    client.connect(host, port, 60); client.loop_forever()
threading.Thread(target=mqtt_loop, daemon=True).start()

rclpy.spin(node)
PY
