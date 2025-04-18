# CARLA Autonomous Driving Controller

This project implements longitudinal and lateral control algorithms for autonomous vehicles in the CARLA simulation environment. It includes multiple versions of lateral controllers (`CustomLateralController_v1`, `CustomLateralController_v2`, `CustomLateralController_v3`) and a longitudinal controller (`CustomLongitudinalController`). Below is a detailed description of the algorithms used in each controller.

## Longitudinal Controller (`CustomLongitudinalController`)

### Algorithm Description:
The longitudinal controller aims to maintain the target speed of the vehicle. It adjusts the throttle and brake values based on the difference between the current vehicle speed and the target speed using a **Proportional Control** algorithm.

### Control Steps:

1. **Calculate Speed Error**: The error between the target speed and the current speed is calculated first:

   $$
   \text{speed error} = \text{target speed} - \text{current speed}
   $$

 2. **Throttle Calculation**: If the current speed is below the target speed, the longitudinal controller increases the throttle using proportional control. The throttle is scaled by the speed error and is limited to the maximum acceleration value (`max_accel`). 

   $$
   \text{throttle} = k_p \times \text{speed error}
   $$
   
 3. **Brake Calculation**: If the current speed exceeds the target speed (i.e., speed error is negative), braking is applied. The brake force is calculated using proportional control and is limited to the maximum deceleration value (`max_decel`).

   $$
   \text{brake} = -k_p \times \text{speed error}
   $$

 4. **Control Output**: If the throttle value is positive, throttle is applied, and brake is set to zero. If the speed error is negative, brake is applied, and throttle is set to zero.

---

## Lateral Controllers (`CustomLateralController_v1`, `CustomLateralController_v2`, `CustomLateralController_v3`)

The goal of the lateral controller is to adjust the steering angle to make the vehicle follow a predetermined path. This process is known as **path tracking**.

### `CustomLateralController_v1` Algorithm (Version 1.0):

 1. **Calculate Heading Error**: The heading error is computed by calculating the vector direction from the vehicle's position to the closest waypoint. The heading error is the angular difference between the vehicle's current heading and the direction towards the waypoint.

   $$
   \text{heading error} = \text{waypoint angle} - \text{current heading}
   $$

   Where `waypoint_angle` is the direction from the vehicle to the target waypoint, and `current_heading` is the vehicle's current heading.

 2. **Normalize Error**: The heading error is normalized to the range $[-\pi, \pi]$ to avoid angle overflow.

 3. **Calculate Steering Angle**: The steering angle is calculated using proportional control with a proportional gain factor of 2.0. The steering angle is then limited to the range $[-1.0, 1.0]$ to ensure it stays within the acceptable range for the vehicle.

   $$
   \text{steering} = \frac{2.0 \times \text{heading error}}{\pi}
   $$

 4. **Control Output**: The computed steering angle is returned.

### `CustomLateralController_v2` Algorithm (Version 2.0):

Similar to `v1`, `v2` introduces a **gain factor (steer_gain)** to adjust the sensitivity of the steering response:

 1. **Heading Error Calculation**: Same as `v1`, the heading error is computed between the current vehicle position and the target waypoint.

 2. **Gain Control**: Unlike `v1`, `v2` uses a **gain factor (steer gain)** to scale the heading error and adjust the steering response:

   $$
   \text{steering} = \text{steer gain} \times \frac{\text{heading error}}{\pi}
   $$

 3. **Control Output**: The steering angle is limited to the range $[-1.0, 1.0]$.

### `CustomLateralController_v3` Algorithm (Version 3.0):

`v3` is designed similarly to `v1` and `v2`, but it uses a different approach to calculate the steering angle:

 1. **Heading Error Calculation**: Same as the previous versions, the heading error is computed between the vehicle and the target waypoint.

 2. **Non-linear Steering Response**: Unlike `v1` and `v2` that use linear proportional control, `v3` uses a **tangent function (tan)** to calculate the steering angle:

   $$
   \text{steering} = \tan(\text{heading error})
   $$

   Using the tangent function provides a smoother response, especially when the heading error is small, which helps to reduce oversteering.

 3. **Control Output**: The steering angle is also limited to the range $[-1.0, 1.0]$ to ensure it remains within an acceptable range for the vehicle.

### Summary:
- **Longitudinal Control**: Uses proportional control to ensure the vehicle reaches and maintains the target speed, preventing overspeeding or moving too slowly.
- **Lateral Control**: Adjusts the steering angle based on the heading error between the vehicle's current position and the waypoints. Different versions of the lateral controller use various methods (linear proportional control, gain adjustment, non-linear steering response) to provide different tracking behaviors.

---

## Usage:

```python
vehicle = ...  # Get the vehicle object
controller = Controller(vehicle)

target_speed = 30  # Set the target speed in km/h
waypoints = [...]  # Get a series of waypoints

control_command = controller.run_step(target_speed, waypoints)
vehicle.apply_control(control_command)  # Apply the control command

```
---

## Video Example

Here is the embedded video:

<iframe width="560" height="315" src="https://www.youtube.com/embed/vgmIjZysimE" frameborder="0" allow="accelerometer; autoplay; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>



