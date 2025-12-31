# Inverted Pendulum — Swing-up + LQR Hybrid

A hybrid controller for a pendulum-on-cart system: global energy-shaping swing-up followed by an LQR balance controller with automatic fallback to swing-up.

Requirements

- Python 3.8+

- numpy (pip install numpy)

- nmotion_transport hardware interface (vendor library / drivers — must be installed separately)

- Access to the motor controller serial/COM port (e.g. "COM3") and proper device IDs for driver/encoder

- (Optional) matplotlib / pandas for log plotting and analysis

Ensure proper hardware wiring, correct COM port, and that the motor driver/encoder firmware is installed and functional before running the code.