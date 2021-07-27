# NeedleValveController
Arduino code to calibrate and control the needle valves in the plumbing system. The needle valves control filling and venting of N2O on the rocket.

The valve is opened and closed by a geared DC motor with an encoder. Two spur gears are used to transmit the torque to the valve stem. 
As the valve stem rotates, the screw thread causes it to move up and down slightly. There is a blue, plastic spacer that stops the valve from closing fully (even if the motor malfunctions and tries to drive the valve shut continuously). This ensures that nitrous can always leak slightly from the flight tank which prevents a situation where the rocket cannot be depressurised.

To calibrate the zero position of the encoder, the motor slowly moves the valve stem down until it hits the hard endstop. The motor stopping is detected by the arduino by a drop in encoder pulses. The arduino then commands the motor to move the valve stem back up slightly to a soft endstop. The zero point is set here which makes sure the motor never runs into the hard endstop during normal operation.

![Needle Valve Picture](https://user-images.githubusercontent.com/87128082/127150365-3fe3d8cd-bc26-4a85-a843-ac6b9aa92c3a.jpg)
