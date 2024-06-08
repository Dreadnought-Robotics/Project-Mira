#!/usr/bin/python3
import json

import rospy
from flask import Flask, jsonify, request
from std_srvs.srv import Empty

from custom_msgs.msg import telemetry


class DockingTransmission:
    def __init__(self) -> None:
        rospy.wait_for_service("/docking/reset")
        self.docking_depth = None
        self.docked = False
        self.waiting_time = 10
        self.waited_for = 0
        self.depth = None
        self.data_transfer = False
        self.depth_sub = rospy.Subscriber(
            "/master/telemetry", telemetry, self.depth_callback
        )
        self.app = Flask(__name__)
        self.teamURL = "/enter_url_here"
        self.reset_service = rospy.ServiceProxy("/docking/reset", Empty)

        self.app.route(self.teamURL, methods=["POST"])(self.receive_data)

    def depth_callback(self, msg):
        self.depth = msg.external_pressure
        if not self.docked and self.depth >= self.docking_depth:
            self.docked = True
            self.init_time = rospy.Time.now().to_sec()
        elif self.docked and self.depth < self.docking_depth:
            self.docked = False
        elif self.docked:
            if self.waited_for > self.waiting_time and not self.data_transfer:
                self.reset_service.call()
            elif self.waited_for < self.waiting_time:
                self.waited_for = rospy.Time.now().to_sec() - self.init_time
        else:
            pass

    def receive_data(self):
        data = request.get_json()
        if data is not None:
            self.data_transfer = True
            with open("powerpuck_data.txt", "w") as file:
                json.dump(data, file)
                file.write("\n")
            print(data)
            return jsonify({"status": "success"}), 200
        else:
            return jsonify({"status": "no data"}), 400


if __name__ == "__main__":
    rospy.init_node("dataTransferServer")
    obj = DockingTransmission()
    obj.app.run(host="192.168.2.5", port=5000, debug=True)
