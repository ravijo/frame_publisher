#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# main.py: script to publish tf from ui
# Author: Ravi Joshi
# Date: 2023/05/28

# import system modules
import os
import sys
from math import radians as toRad
from math import degrees as toDeg

# import PyQT modules
from PyQt5 import QtWidgets, uic
from PyQt5.QtCore import QTimer

# import ROS modules
import rospy
import rospkg
from tf2_ros import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped
from tf.transformations import quaternion_from_euler


class FramePublisherUI(QtWidgets.QMainWindow):
    def __init__(self, uiFile):
        super(FramePublisherUI, self).__init__()
        uic.loadUi(uiFile, self)
        self.setupBroadcaster()
        self.setupDials()
        self.show()

    def setupDials(self):
        self.dialRoll.valueChanged.connect(
            lambda deg: (
                self.textBoxRollDeg.setValue(deg),
                self.textBoxRollRad.setValue(toRad(deg)),
            )
        )
        self.dialYaw.valueChanged.connect(
            lambda deg: (
                self.textBoxYawDeg.setValue(deg),
                self.textBoxYawRad.setValue(toRad(deg)),
            )
        )
        self.dialPitch.valueChanged.connect(
            lambda deg: (
                self.textBoxPitchDeg.setValue(deg),
                self.textBoxPitchRad.setValue(toRad(deg)),
            )
        )

        self.textBoxRollDeg.valueChanged.connect(
            lambda deg: (
                self.dialRoll.setValue(deg),
                self.textBoxRollRad.setValue(toRad(deg)),
            )
        )
        self.textBoxPitchDeg.valueChanged.connect(
            lambda deg: (
                self.dialPitch.setValue(deg),
                self.textBoxPitchRad.setValue(toRad(deg)),
            )
        )
        self.textBoxYawDeg.valueChanged.connect(
            lambda deg: (
                self.dialYaw.setValue(deg),
                self.textBoxYawRad.setValue(toRad(deg)),
            )
        )

        self.textBoxRollRad.valueChanged.connect(
            lambda rad: (
                self.dialRoll.setValue(int(toDeg(rad))),
                self.textBoxRollDeg.setValue(int(toDeg(rad))),
            )
        )
        self.textBoxPitchRad.valueChanged.connect(
            lambda rad: (
                self.dialPitch.setValue(int(toDeg(rad))),
                self.textBoxPitchDeg.setValue(int(toDeg(rad))),
            )
        )
        self.textBoxYawRad.valueChanged.connect(
            lambda rad: (
                self.dialYaw.setValue(int(toDeg(rad))),
                self.textBoxYawDeg.setValue(int(toDeg(rad))),
            )
        )

    def setupBroadcaster(self):
        self.broadcaster = StaticTransformBroadcaster()
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.publishFrame)
        self.timer.start(100)

    def publishFrame(self):
        transformStamped = TransformStamped()

        transformStamped.header.stamp = rospy.Time.now()
        transformStamped.header.frame_id = self.textBoxParentFrameId.text()
        transformStamped.child_frame_id = self.textBoxChildFrameId.text()

        transformStamped.transform.translation.x = self.textBoxX.value()
        transformStamped.transform.translation.y = self.textBoxY.value()
        transformStamped.transform.translation.z = self.textBoxZ.value()

        roll, pitch, yaw = (self.dialRoll.value(), self.dialPitch.value(), self.dialYaw.value())
        q = quaternion_from_euler(toRad(roll), toRad(pitch), toRad(yaw))
        transformStamped.transform.rotation.x = q[0]
        transformStamped.transform.rotation.y = q[1]
        transformStamped.transform.rotation.z = q[2]
        transformStamped.transform.rotation.w = q[3]

        self.broadcaster.sendTransform(transformStamped)


def main():
    rospy.init_node("frame_publisher")

    pkgDir = rospkg.RosPack().get_path('frame_publisher')
    uiFile = os.path.join(pkgDir, "files", "qt.ui")

    app = QtWidgets.QApplication(sys.argv)
    win = FramePublisherUI(uiFile)
    app.exec_()


if __name__ == "__main__":
    main()
