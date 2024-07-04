 # RobotDelivery.py
#
# @author Emima Jiva. (c) ai2-UPV Todos los derechos reservados.
#
# Proyecto "WASHCARROB"
# Version: 1.0
# Rev: 2023. cambios introducidos

import rmf_dispenser_msgs.msg as dispenser_msgs
import rmf_ingestor_msgs.msg as ingestor_msgs
import rmf_task_msgs.msg as task_msgs

import rclpy
import time
from rclpy.node import Node
from functools import partial
import json

DISPENSER_RESULT_ACKNOWLEDGED = 0
DISPENSER_RESULT_SUCCESS = 1
DISPENSER_RESULT_FAILED = 2

INGESTOR_RESULT_ACKNOWLEDGED = 0
INGESTOR_RESULT_SUCCESS = 1
INGESTOR_RESULT_FAILED = 2

DISPENSER_REQUEST  = 1
DISPENSER_RESULT = 2
INGESTOR_REQUEST = 3
INGESTOR_RESULT = 4

class DeliveryTask(Node):
    def __init__(self, name, api, disp_res_topic, ing_res_topic):
        super().__init__(name)

        # Variables
        self.list_tasks = {}
        self.state_task = {}
        self.api = api

        self.api.client.subscribe(disp_res_topic, 1, self._dispenser_results_cb)
        self.api.client.subscribe(ing_res_topic, 1, self._ingestor_results_cb)

        self.dispatch_states = self.create_subscription(
            task_msgs.DispatchStates,
            '/dispatch_states',
            self._dispatch_states_cb,
            1
        )
        self.dispenser_sub = self.create_subscription(
            dispenser_msgs.DispenserRequest,
            '/dispenser_requests',
            self._dispenser_request_cb,
            1
        )
        self.dispenser_pub = self.create_publisher(
            dispenser_msgs.DispenserResult,
            '/dispenser_results',
            1
        )
        self.ingestor_sub = self.create_subscription(
            ingestor_msgs.IngestorRequest,
            '/ingestor_requests',
            self._ingestor_request_cb,
            1
        )
        self.ingestor_pub = self.create_publisher(
            ingestor_msgs.IngestorResult,
            '/ingestor_results',
            1
        )

    def _dispatch_states_cb(self, msg):
        for i in range(len(msg.active)):
            task = msg.active[i].task_id
            robot_name = msg.active[i].assignment.expected_robot_name
            #self.get_logger().info('LIST_TASKS ACTIVE: "%s"' % self.list_tasks)
            self.list_tasks[task] =  robot_name

    def _dispenser_request_cb(self, msg):
        task = msg.request_guid
        if self.list_tasks.get(task) is not None:
            robot_name = self.list_tasks[task]
            if (self.state_task.get(robot_name) is None) or  self.state_task.get(robot_name) is not DISPENSER_REQUEST:
                self.state_task[robot_name] = DISPENSER_REQUEST
                self.api.pub_dispenser_requests(robot_name, task)

    def _dispenser_results_cb(self, client, userdata, msg):
        robot_name = msg.topic.split("/")[1]
        self.get_logger().info('REQUEST_ID: "%s"' % robot_name)
        self.state_task[robot_name] = DISPENSER_RESULT
        decoded_message=str(msg.payload.decode("utf-8"))
        request_guid = json.loads(decoded_message)['data']
        self.get_logger().info('REQUEST_ID: "%s"' % request_guid)
        result = dispenser_msgs.DispenserResult()
        result.time = self.get_clock().now().to_msg()
        result.status = DISPENSER_RESULT_SUCCESS
        result.request_guid = request_guid
        self.dispenser_pub.publish(result)

    def _ingestor_request_cb(self, msg):
        task = msg.request_guid
        robot_name = self.list_tasks[task]
        if (self.state_task.get(robot_name) is None) or  self.state_task.get(robot_name) is not INGESTOR_REQUEST:
            self.state_task[robot_name] = INGESTOR_REQUEST
            self.api.pub_ingestor_requests(robot_name, task)

    def _ingestor_results_cb(self, client, userdata, msg):
        robot_name = msg.topic.split("/")[1]
        self.state_task[robot_name] = INGESTOR_RESULT
        decoded_message=str(msg.payload.decode("utf-8"))
        request_guid = json.loads(decoded_message)['data']
        self.get_logger().info('REQUEST_ID_INGESTOR: "%s"' % request_guid)
        result = ingestor_msgs.IngestorResult()
        result.time = self.get_clock().now().to_msg()
        result.status = INGESTOR_RESULT_SUCCESS
        result.request_guid = request_guid
        self.ingestor_pub.publish(result)




