# Field-Oriented-Control-for-BLDC-motors

The goal of this project is to get familiar with Field-Oriented Control (FOC) for BLDC motors. To test this control strategy, I designed the mechanical parts and built a physical test rig.

https://github.com/user-attachments/assets/1e0964cc-aaa5-48fc-aaf7-70208950e4f5

## Control Theory Approaches
I tried two separate approaches for the control system:
### PID Control: Implemented the same type of PID system utilized in my previous project to handle feedback.
### LQR Control: Attempted a Linear Quadratic Regulator (LQR) approach. This required modeling the physical parameters of the system, which I estimated using data and values derived from the CAD assembly.
Note: I did not manage to get the LQR controller working, likely due to errors from estimated material properties and unverified motor specifications.

## Mechanical Design
I designed the structural parts to house and support the motor and components. The layout and CAD assembly of the design are shown below:

<img width="600" alt="5D805113-263E-41D6-8755-A54465EABAEB" src="https://github.com/user-attachments/assets/5f19599c-c0d4-44da-b06d-7c912c754d89" />

