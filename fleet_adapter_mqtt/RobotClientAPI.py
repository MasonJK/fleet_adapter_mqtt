# Copyright 2021 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# RobotClientAPI.py
#
# @author Emima Jiva. (c) ai2-UPV Todos los derechos reservados.
#
# Proyecto "WASHCARROB"
# Version: 1.0
# Rev: 2023. cambios introducidos  

from AWSIoTPythonSDK.MQTTLib import AWSIoTMQTTClient
import json
import math
import numpy as np
import time
from rclpy.time import Time
import logging
import threading

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

def quaternion_from_euler(roll, pitch, yaw):
    """
    Convert an Euler angle to a quaternion.

    Input
    :param roll: The roll (rotation around x-axis) angle in radians.
    :param pitch: The pitch (rotation around y-axis) angle in radians.
    :param yaw: The yaw (rotation around z-axis) angle in radians.

    Output
    :return qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format
    """
    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    return [qx, qy, qz, qw]

def euler_from_quaternion(x, y, z, w):
    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    roll is rotation around x in radians (counterclockwise)
    pitch is rotation around y in radians (counterclockwise)
    yaw is rotation around z in radians (counterclockwise)
    """
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)

    return roll_x, pitch_y, yaw_z # in radians

'''
    The RobotAPI class is a wrapper for API calls to the robot. Here users
    are expected to fill up the implementations of functions which will be used
    by the RobotCommandHandle. For example, if your robot has a REST API, you
    will need to make http request calls to the appropriate endpoints within
    these functions.
'''

class RobotAPI:
    def __init__(self, client_id: str, port: int, iot_endpoint: str, root_ca_path: str, private_key_path: str, \
                cert_path: str, dispenser_topic: str, ingestor_topic: str):
        #Delivery topic
        self.dispenser_topic = dispenser_topic
        self.ingestor_topic = ingestor_topic
        self.start_ae_topic = "rmf_start_ae"

        # Initialize parameters
        self.robotpose = {}
        self.battery = {}
        self.resultgoal = {}
        self.feedback = {}
        self.robots = {}

        # MQTT Connection
        self.client = self.connect_mqtt(client_id, port, iot_endpoint, root_ca_path, private_key_path, cert_path)

        # Start a thread to keep the client running
        self.thread = threading.Thread(target=self.keep_running)
        self.thread.start()


    def stop_connection(self):
        self.logger.info("Disconnecting from AWS IoT Core")
        self.client.disconnect()
        self.thread.join
        self.logger.info("Disconnected")


    def connect_mqtt(self, client_id, port, iot_endpoint, root_ca_path, private_key_path, cert_path):
        # Initialize the MQTT client
        client = AWSIoTMQTTClient(client_id)
        # Configure the client
        client.configureEndpoint(iot_endpoint, port)
        client.configureCredentials(root_ca_path, private_key_path, cert_path)

        # Configure connection settings
        client.configureAutoReconnectBackoffTime(1, 32, 20)
        client.configureOfflinePublishQueueing(-1)  # Infinite offline Publish queueing
        client.configureDrainingFrequency(2)  # Draining: 2 Hz
        client.configureConnectDisconnectTimeout(10)  # 10 sec
        client.configureMQTTOperationTimeout(5)  # 5 sec

        try:
            client.connect()
            logger.info("[RMF] Connected to AWS IoT Core")
        except Exception as e:
            logger.error(f"[RMF] Failed to connect to AWS IoT Core: {str(e)}")
            exit(1)  # Exit the script with a non-zero status

        # Subscribe to topics
        client.subscribe('+/rmf_pose', 1, self.on_message_pose)
        client.subscribe("+/rmf_mission/result", 1, self.on_message_result)
        client.subscribe("+/rmf_mission/feedback", 1, self.on_message_feedback)
        client.subscribe('+/rmf_battery', 1, self.on_message_battery)

        return client


    def keep_running(self):
        while True:
            time.sleep(1)


    def on_message_pose(self, client, userdata, msg):
        # Assuming pose will come with PoseStamped
        decoded_message=str(msg.payload.decode("utf-8"))
        pose=json.loads(decoded_message)['payload']['pose']
        robot = msg.topic.split("/")[0]
        # logger.info(f'{robot}: received pose!')
        self.robotpose[robot] = pose
        self.robots[robot] = robot

    def on_message_result(self, client, userdata, msg):
        decoded_message=str(msg.payload.decode("utf-8"))
        result=json.loads(decoded_message)['payload']
        robot = msg.topic.split("/")[0]
        self.resultgoal[robot] = result

    def on_message_feedback(self, client, userdata, msg):
        decoded_message=str(msg.payload.decode("utf-8"))
        feed=json.loads(decoded_message)['payload']
        robot = msg.topic.split("/")[0]
        self.feedback[robot] = feed

    def on_message_battery(self, client, userdata, msg):
        decoded_message=str(msg.payload.decode("utf-8"))
        bat=json.loads(decoded_message)['payload']
        robot = msg.topic.split("/")[0]
        self.battery[robot] = bat


    def position(self, robot_name: str):
        ''' Return [x, y, theta] expressed in the robot's coordinate frame or
            None if any errors are encountered'''
        if self.robots.get(robot_name) is not None:
            x = round(self.robotpose[robot_name]['position']['x'],2)
            y = round(self.robotpose[robot_name]['position']['y'],2)
            theta = round(euler_from_quaternion(
            self.robotpose[robot_name]['orientation']['x'], self.robotpose[robot_name]['orientation']['y'],
            self.robotpose[robot_name]['orientation']['z'], self.robotpose[robot_name]['orientation']['w'])[2],2)
            return [x, y, theta]
        else:
            logger.error("No position for " + robot_name)
            return None


    # TODO : receive goal received result
    def navigate_to_waypoints(self,
                 robot_name: str,
                 cmd_id: int,
                 poses,
                 map_name: str):
        ''' Request the robot to navigate to poses:[{x,y,theta}] where x, y and
            and theta are in the robot's coordinate convention. This function
            should return True if the robot has accepted the request,
            else False'''
        goal_data = {
            "uuid": cmd_id,
            "communication type": "action",
            "interface": "FollowWaypoints",
            "ros name": "follow_waypoints",
            "payload": {
                "poses": []
            }
        }

        for waypoint_pose in poses:
            orientation = quaternion_from_euler(0,0,round(waypoint_pose[2],2))
            waypoint = {
                "interface": "PoseStamped",
                # TODO : fill in stamp
                "header.stamp": "",
                "header.frame_id": "map",

                "pose.position.x": round(waypoint_pose[0],2),
                "pose.position.y": round(waypoint_pose[1],2),
                "pose.orientation.x": orientation[0],
                "pose.orientation.y": orientation[1],
                "pose.orientation.z": orientation[2],
                "pose.orientation.w": orientation[3],
            }
            goal_data["payload"]["poses"].append(waypoint)
        self.client.publish(robot_name+"rmf_mission/goal" ,json.dumps(goal_data), 1)

        if self.feedback.get(robot_name) is not None and self.feedback[robot_name] == 1:
            self.feedback[robot_name] = 0
            return True
        else:
            return False


    # TODO : receive cancel result
    def stop(self, robot_name: str, cmd_id: int):
        ''' Command the robot to stop.
            Return True if robot has successfully stopped. Else False'''
        cancel_data = {
            "uuid": cmd_id,
            "communication type": "action",
            "interface": "FollowWaypoints",
            "ros name": "follow_waypoints",
        }
        self.client.publish(robot_name+"/rmf_mission/cancel" ,json.dumps(cancel_data), 2)
        return True

    def stop_all_robots(self, cmd_id: int):
        ''' Command the robot to stop.
            Return True if robot has successfully stopped. Else False'''
        cancel_data = {
            "uuid": cmd_id,
            "communication type": "action",
            "interface": "FollowWaypoints",
            "ros name": "follow_waypoints",
        }
        self.client.publish("+/rmf_mission/cancel" ,json.dumps(cancel_data), 2)
        return True



    def navigation_completed(self, robot_name: str, cmd_id: int):
        ''' Return True if the robot has successfully completed its previous
            navigation request. Else False.'''
        #if (self.resultgoal.get(robot_name) is not None and self.resultgoal[robot_name] == 3):
        # TODO : declare status
        if self.resultgoal[robot_name] == 1:
            logger.info("Navigation completed! " +  robot_name)
            return True
        else:
            logger.warn("Navigation not completed " + robot_name)
            return False


    def process_completed(self, robot_name: str, cmd_id: int):
        ''' Return True if the robot has successfully completed its previous
            process request. Else False.'''
        return self.navigation_completed(robot_name, cmd_id)


    def battery_soc(self, robot_name: str):
        ''' Return the state of charge of the robot as a value between 0.0
            and 1.0. Else return None if any errors are encountered'''
        if self.battery.get(robot_name) is not None:
            # logger.info("Battery " + str(self.battery.get(robot_name)))
            return self.battery[robot_name]/100
        else:
            logger.info("Battery none ")
            return None

    # TODO
    def start_process(self,
                      robot_name: str,
                      cmd_id: int,
                      process: str,
                      map_name: str):
        return True

    # TODO
    def navigation_remaining_duration(self, robot_name: str, cmd_id: int):
        return 0.0

    # TODO
    def requires_replan(self, robot_name: str):
        '''Return whether the robot needs RMF to replan'''
        return False

    # TODO
    def pub_dispenser_requests(self, robot_name: str, task_id: str):
        data = { "data": task_id }
        self.client.publish(robot_name + self.dispenser_topic ,json.dumps(data), 1)

    # TODO
    def pub_ingestor_requests(self, robot_name: str, task_id: str):
        data = { "data": task_id }
        self.client.publish(self.ingestor_topic + robot_name ,json.dumps(data), 1)

    # TODO
    def publish_action_execution(self, robot_name: str):
        data = { "data": True }
        self.client.publish(self.start_ae_topic + robot_name ,json.dumps(data), 1)
