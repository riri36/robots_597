
from mapUtilities import *
from utilities import *
from numpy import cos, sin
import numpy as np


class particle:

    def __init__(self, pose, weight):
        self.pose = pose
        self.weight = weight

    def motion_model(self, v, w, dt):
        #TODO: Implement the motion model for the particle
        """
        v: linear velocity
        w: angular velocity
        dt: time step
        """
        # if np.abs(w) > 1e-6:  # Avoid division by zero
        #     self.pose[0] += (v / w) * (- sin(self.pose[2]) + sin(self.pose[2] + w * dt))
        #     self.pose[1] += (v / w) * (-cos(self.pose[2] + w * dt) + cos(self.pose[2]))
        # else:  # Approximate straight-line motion
        #     self.pose[0] += v * dt * np.cos(self.pose[2])
        #     self.pose[1] += v * dt * np.sin(self.pose[2])

        # self.pose[2] += w * dt
        # self.pose[2] = (self.pose[2] + np.pi) % (2 * np.pi) - np.pi  # Normalize theta to [-π, π]

        # self.pose[0] += float(-(v/w)*sin(self.pose[2]) + (v/w)*sin(self.pose[2] + w*dt))
        # self.pose[1] += float((v/w)*cos(self.pose[2]) - (v/w)*cos(self.pose[2] + w*dt))
        # self.pose[2] += float(w*dt)

        self.pose[0] += v*dt*cos(self.pose[2]+w*dt/2)
        self.pose[1] += v*dt*sin(self.pose[2]+w*dt/2)
        self.pose[2] += w*dt

    # TODO: You need to explain the following function to TA
    def calculateParticleWeight(self, scanOutput: LaserScan, mapManipulatorInstance: mapManipulator, laser_to_odom_transformation: np.array):

        # makes a transform matrix to go from particle pose to world coors
        T = np.matmul(self.__poseToTranslationMatrix(), laser_to_odom_transformation)
        # convert laserscan points to cartesian coors
        _, scanCartesianHomo = convertScanToCartesian(scanOutput)
        # applies dot product to put laser scan points into map frame
        scanInMap = np.dot(T, scanCartesianHomo.T).T

        # get field from mapUtilities
        likelihoodField = mapManipulatorInstance.getLikelihoodField()
        # converts map positions to cell index
        cellPositions = mapManipulatorInstance.position_2_cell(
            scanInMap[:, 0:2])

        lm_x, lm_y = likelihoodField.shape
        # filters cell positions to be within valid map range
        cellPositions = cellPositions[np.logical_and.reduce(
                (cellPositions[:, 0] > 0, -cellPositions[:, 1] > 0, cellPositions[:, 0] < lm_y,  -cellPositions[:, 1] < lm_x))]

        # Uses log likelihood to prevent numerical underflow, since probs are often rly small
        # and log lets you sum rather than multiply.
        # One laser scan has multiple pts, each of which demonstrates if the particle pose matches the map.
        # Summing the logs adds all scan point probabilties, where higher sum=higher weight (pts are better
        # matching the map)
        log_weights = np.log(
            likelihoodField[-cellPositions[:, 1], cellPositions[:, 0]])
        log_weight = np.sum(log_weights)
        weight = np.exp(log_weight)
        weight += 1e-10

        self.setWeight(weight) #updates weight of particles one at a time

    def setWeight(self, weight):
        self.weight = weight

    def getWeight(self):
        return self.weight

    def setPose(self, pose):
        self.pose = pose

    def getPose(self):
        return self.pose[0], self.pose[1], self.pose[2]

    def __poseToTranslationMatrix(self):
        x, y, th = self.getPose()

        translation = np.array([[cos(th), -sin(th), x],
                                [sin(th), cos(th), y],
                                [0, 0, 1]])

        return translation
