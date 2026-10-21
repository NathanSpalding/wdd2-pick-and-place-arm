# Notes for future students

What worked, what didn't, and what to do differently.

## Worked well

- **Encoders on every joint.** AS5600s through a TCA9548A read within 0.61° of a reference and showed errors the servos hide (up to 5.7° of sag at the shoulder with 500 g).
- **Integral trim on the encoder angle.** A slow outer loop on top of the servo's own loop cut steady-state error 2 to 20 times on the arm joints.
- **Grasp detection from servo current.** An INA219 on the gripper supply detected 70 of 70 grasps with a simple median-current threshold.
- **Simulating first.** The URDF model served the simulator, the inverse kinematics and the torque analysis.

## Shortfalls

- **Base ringing.** The base servo carries the arm's overturning moment on its spline and has play at the horn, so the base rings at about 3 Hz after each swing. Let bearings carry the loads and drive the base through a stiff reduction.
- **Shoulder stick-slip.** The shoulder servo moves in 0.5–0.8° jumps under gravity load, so it can settle about 1° off target. Use a low-backlash actuator or an external reduction.

## Practical advice

- Check the relay flyback diode's polarity before first power-up.
- Set every buck converter with no servo connected.
- Fix encoder magnets and mounts firmly; if one moves, re-measure the zero.
- Log everything at 50 Hz from the start. Slow serial prints miss short events such as the gripper's closing current peak.
- Keep the gripper vertical when lifting heavy payloads; tilting it lets off-centre weights rotate in the jaws.
