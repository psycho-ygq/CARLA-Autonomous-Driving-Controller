""" This module contains controllers to perform lateral and longitudinal control. """

import carla
import math
import numpy as np
from collections import deque
from utils import get_speed


class CUtils(object):
    def __init__(self):
        self.parameters = {}

    def set_param(self, param_name, value):
        self.parameters[param_name] = value

    def get_param(self, param_name, default_value=None):
        return self.parameters.get(param_name, default_value)


class Controller:
    """
    Controller combines both longitudinal and lateral controllers to manage the vehicle's movement in CARLA.
    """

    def __init__(self, vehicle, max_throttle=0.75, max_brake=0.3, max_steering=0.8):
        """
        Initializes the Controller with maximum values for throttle, brake, and steering, along with the vehicle object.

        :param vehicle: The vehicle object to control.
        :param max_throttle: Maximum throttle value.
        :param max_brake: Maximum brake value.
        :param max_steering: Maximum steering angle.
        """
        self.max_brake = max_brake
        self.max_throt = max_throttle
        self.max_steer = max_steering

        self._vehicle = vehicle
        self._world = self._vehicle.get_world()  # Get the world object from the vehicle
        self.past_steering = (
            self._vehicle.get_control().steer
        )  # Store the current steering value for future reference

        self._lon_controller = CustomLongitudinalController(
            self._vehicle
        )  # Longitudinal controller instance
        self._lat_controller = CustomLateralController_v3(
            self._vehicle
        )  # Lateral controller instance

    def run_step(self, target_speed, waypoints):
        """
        Executes one step of control using both the longitudinal and lateral controllers.

        :param target_speed: The desired vehicle speed in km/h.
        :param waypoints: The waypoints to follow, each waypoint is a carla.Waypoint object.
        :return: A carla.VehicleControl object containing the control commands for the vehicle.
        """
        acceleration = self._lon_controller.run_step(target_speed)  # Get acceleration from longitudinal controller
        current_steering = self._lat_controller.run_step(waypoints)  # Get steering angle from lateral controller

        control = carla.VehicleControl()  # Initialize the vehicle control command
        if acceleration >= 0.0:
            control.throttle = min(acceleration, self.max_throt)
            control.brake = 0.0
        else:
            control.throttle = 0.0
            control.brake = min(abs(acceleration), self.max_brake)

        # Steering adjustments to ensure smooth changes and adherence to max steering values
        if current_steering > self.past_steering + 0.1:
            current_steering = self.past_steering + 0.1
        elif current_steering < self.past_steering - 0.1:
            current_steering = self.past_steering - 0.1

        if current_steering >= 0:
            steering = min(self.max_steer, current_steering)
        else:
            steering = max(-self.max_steer, current_steering)

        control.steer = steering
        control.hand_brake = False
        control.manual_gear_shift = False
        self.past_steering = (steering)  # Update the past_steering value for the next control cycle

        return control  # Return the control command for the vehicle


class CustomLongitudinalController:

    def __init__(self, vehicle):
        # Initialize the longitudinal controller with the vehicle object
        self._vehicle = vehicle
        self.vars = CUtils()
        self.target_speed = 0
        self.k_p = 0.5
        self.max_accel = 1.0
        self.max_decel = 1.0

    def run_step(self, target_speed):
        # Get the current speed of the vehicle
        current_speed = get_speed(self._vehicle)

        # Calculate the speed error (difference between target speed and current speed)
        speed_error = target_speed - current_speed

        # Calculate throttle using proportional control (k_p * speed_error)
        throttle = self.k_p * speed_error
        # Limit throttle to be between 0 and the maximum acceleration value
        throttle = max(0.0, min(throttle, self.max_accel))

        # If the speed error is negative (i.e., vehicle is overspeeding), apply braking
        if speed_error < 0:
            # Calculate brake force based on proportional control (k_p * speed_error)
            brake = -self.k_p * speed_error
            # Limit brake force to be between 0 and the maximum deceleration value
            brake = max(0.0, min(brake, self.max_decel))
            throttle = 0.0  # If braking is applied, set throttle to 0
        else:
            brake = 0.0  # No braking is needed if the vehicle is not overspeeding

        # Return the net control command: throttle minus brake force
        return throttle - brake


#########  v3.0
class CustomLateralController_v3:

    def __init__(self, vehicle):
        # Initialize the lateral controller with the vehicle object
        self._vehicle = vehicle
        self.lookahead_distance = 10

    def run_step(self, waypoints):
        # Get the current transform (position and orientation) of the vehicle
        current_transform = self._vehicle.get_transform()
        # Get the current location of the vehicle
        current_location = current_transform.location
        # Get the current heading (yaw) of the vehicle in degrees
        current_heading = current_transform.rotation.yaw

        # Find the closest waypoint from the list of waypoints
        closest_waypoint = self.get_closest_waypoint(waypoints, current_location)

        # Calculate the vector direction from the vehicle to the closest waypoint
        direction_to_waypoint = closest_waypoint.transform.location - current_location
        # Calculate the angle of the vector to the waypoint
        waypoint_angle = math.atan2(direction_to_waypoint.y, direction_to_waypoint.x)
        # Calculate the heading error (difference between the vehicle's current heading and the waypoint angle)
        heading_error = waypoint_angle - math.radians(current_heading)

        # Normalize the heading error to be within the range [-pi, pi] to avoid large errors
        if heading_error > math.pi:
            heading_error -= 2 * math.pi
        elif heading_error < -math.pi:
            heading_error += 2 * math.pi

        # Use the tangent of the heading error to determine the steering angle
        # This provides a non-linear response to the heading error
        steering = math.tan(heading_error)
        # Limit the steering to be between -1.0 and 1.0 to stay within the valid range
        steering = max(-1.0, min(steering, 1.0))

        # Return the steering value that the vehicle should use to follow the path
        return steering

    def get_closest_waypoint(self, waypoints, current_location):
        # Initialize a variable to track the closest waypoint
        closest_distance = float('inf')
        closest_waypoint = None

        # Loop through all the waypoints to find the closest one
        for waypoint in waypoints:
            # Calculate the distance between the vehicle's current location and the waypoint
            distance = waypoint.transform.location.distance(current_location)
            # If this waypoint is closer than the previous closest, update the closest waypoint
            if distance < closest_distance:
                closest_distance = distance
                closest_waypoint = waypoint

        # Return the closest waypoint
        return closest_waypoint


#########  v2.0
class CustomLateralController_v2:

    def __init__(self, vehicle):
        # Initialize the lateral controller with the vehicle object
        self._vehicle = vehicle
        self.lookahead_distance = 10
        self.steer_gain = 2.0

    def run_step(self, waypoints):
        # Get the current transform (position and orientation) of the vehicle
        current_transform = self._vehicle.get_transform()
        # Extract the current location of the vehicle from the transform
        current_location = current_transform.location
        # Extract the current heading (yaw) of the vehicle in degrees
        current_heading = current_transform.rotation.yaw

        # Find the closest waypoint from the list of waypoints
        closest_waypoint = self.get_closest_waypoint(waypoints, current_location)

        # Calculate the vector direction from the vehicle to the closest waypoint
        direction_to_waypoint = closest_waypoint.transform.location - current_location
        # Calculate the angle of the vector to the waypoint (direction to the waypoint)
        waypoint_angle = math.atan2(direction_to_waypoint.y, direction_to_waypoint.x)
        # Calculate the heading error (difference between the vehicle's current heading and the waypoint angle)
        heading_error = waypoint_angle - math.radians(current_heading)

        # Normalize the heading error to be within the range [-pi, pi]
        if heading_error > math.pi:
            heading_error -= 2 * math.pi
        elif heading_error < -math.pi:
            heading_error += 2 * math.pi

        # Calculate the steering value by scaling the heading error with the steering gain
        steering = self.steer_gain * heading_error / math.pi
        # Clamp the steering value to ensure it stays within the valid range of [-1.0, 1.0]
        steering = max(-1.0, min(steering, 1.0))

        # Return the computed steering value
        return steering

    def get_closest_waypoint(self, waypoints, current_location):
        # Initialize the closest distance to infinity and the closest waypoint to None
        closest_distance = float('inf')
        closest_waypoint = None

        # Loop through all the waypoints to find the closest one to the current vehicle location
        for waypoint in waypoints:
            # Calculate the distance from the current location to the waypoint
            distance = waypoint.transform.location.distance(current_location)
            # If this waypoint is closer than the previously closest one, update the closest waypoint
            if distance < closest_distance:
                closest_distance = distance
                closest_waypoint = waypoint

        # Return the closest waypoint
        return closest_waypoint


########  v1.0
class CustomLateralController_v1:

    def __init__(self, vehicle):
        # Initialize the lateral controller with the vehicle object
        self._vehicle = vehicle
        self.vars = CUtils()
        self.lookahead_distance = 10

    def run_step(self, waypoints):
        # Get the current transform (position and orientation) of the vehicle
        current_transform = self._vehicle.get_transform()
        # Extract the current location of the vehicle from the transform
        current_location = current_transform.location
        # Extract the current heading (yaw) of the vehicle in degrees
        current_heading = current_transform.rotation.yaw

        # Find the closest waypoint from the list of waypoints
        closest_waypoint = self.get_closest_waypoint(waypoints, current_location)

        # Calculate the vector direction from the vehicle to the closest waypoint
        direction_to_waypoint = closest_waypoint.transform.location - current_location
        # Calculate the angle of the vector to the waypoint (direction to the waypoint)
        waypoint_angle = math.atan2(direction_to_waypoint.y, direction_to_waypoint.x)
        # Calculate the heading error (difference between the vehicle's current heading and the waypoint angle)
        heading_error = waypoint_angle - math.radians(current_heading)

        # Normalize the heading error to be within the range [-pi, pi]
        if heading_error > math.pi:
            heading_error -= 2 * math.pi
        elif heading_error < -math.pi:
            heading_error += 2 * math.pi

        # Calculate the steering value by scaling the heading error
        steering = 2.0 * heading_error / math.pi
        # Clamp the steering value to ensure it stays within the valid range of [-1.0, 1.0]
        steering = max(-1.0, min(steering, 1.0))

        # Return the computed steering value
        return steering

    def get_closest_waypoint(self, waypoints, current_location):
        # Initialize the closest distance to infinity and the closest waypoint to None
        closest_distance = float('inf')
        closest_waypoint = None

        # Loop through all the waypoints to find the closest one to the current vehicle location
        for waypoint in waypoints:
            # Calculate the distance from the current location to the waypoint
            distance = waypoint.transform.location.distance(current_location)
            # If this waypoint is closer than the previously closest, update the closest waypoint
            if distance < closest_distance:
                closest_distance = distance
                closest_waypoint = waypoint

        # Return the closest waypoint
        return closest_waypoint


