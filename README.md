# Transfer Arm Control System

ESP32-based control system for a robotic transfer arm that picks up objects from one location and places them in another.

## Hardware Components
- **X-Axis Stepper**: Controls left/right movement (400 steps/rev, 20T pulley, 6mm 2GT belt)
- **Z-Axis Stepper**: Controls up/down movement (400 steps/rev, 20T pulley, 6mm 2GT belt)
- **Servo**: 24V 160kg servo with 270° rotation range for gripper orientation
- **Vacuum System**: Solenoid-controlled venturi suction for grabbing wood diamonds
- **Homing Switches**: Active HIGH limit switches for X and Z axes

## Operation Sequences

### Homing Sequence
1. **Begin Homing** - Initialize motors and prepare system
2. **X-Axis Homing** - Move X-axis in negative direction until home switch activates (HIGH)
3. **Z-Axis Homing** - Move Z-axis in negative direction until home switch activates (HIGH)
4. **Servo Homing** - Set servo to neutral position (90°)
5. **Verification** - Confirm all axes are properly homed with timeout protection

### Pick Cycle
1. **Initialization** - System power-on, home steppers and servo
2. **Wait for Signal** - From limit switch (active high) or Stage 1 machine
3. **Pick Position** - Move X-axis to pickup, lower Z-axis, rotate servo
4. **Activate Vacuum** - Turn on solenoid relay to create venturi suction
5. **Lift Object** - Raise Z-axis with object
6. **Move to Place** - Move X-axis to destination, rotate servo
7. **Lower Object** - Lower Z-axis to target
8. **Release Object** - Turn off solenoid relay to release suction
9. **Return Home** - Raise Z-axis, move X-axis to home, reset servo

## Pin Layout
Using Freenove ESP32 breakout board:
- Left side (top to bottom): 34, 35, 32, 33, 25, 26, 27, 14, 12, 13
- Right side (top to bottom): 23, 22, 21, 19, 18, 5, 4, 0, 2, 15

### Pin Assignments
- **Inputs**:
  - Limit Switch: Pin 34 (active high)
  - Stage 1 Signal: Pin 35
  - X-Axis Home Switch: Pin 32 (active high)
  - Z-Axis Home Switch: Pin 33 (active high)
- **Outputs**:
  - X-Axis Stepper Step: Pin 25
  - X-Axis Stepper Dir: Pin 26
  - Z-Axis Stepper Step: Pin 27
  - Z-Axis Stepper Dir: Pin 14
  - Servo Control: Pin 12
  - Solenoid Relay: Pin 13 