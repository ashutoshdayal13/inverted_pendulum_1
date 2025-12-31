import numpy as np
from scipy.linalg import solve_continuous_are

# --- YOUR PARAMETERS ---
M = 0.200    # Mass of Cart (kg)
m = 0.170    # Mass of Pendulum (kg)
L = 0.5      # Total length of rod (m)
l = 0.3      # Distance to Center of Mass (m)
I = (1/12) * m * L**2  # Inertia of rod about COM (kg*m^2)
g = 9.81     # Gravity
r = 0.0075   # Pulley Radius (m) -> ASSUMED 7.5mm. CHANGE IF WRONG.
Kt = 0.0306  # Motor Torque Constant (Nm/A) approx for 270kV

# --- PHYSICS MODEL (State Space) ---
# State: [x, x_dot, theta, theta_dot]
# Input: Force (F)
d = I*(M+m) + M*m*l**2 # Denominator

A = np.array([
    [0, 1, 0, 0],
    [0, 0, -(m**2 * g * l**2)/d, 0],
    [0, 0, 0, 1],
    [0, 0, (m * g * l * (M + m))/d, 0]
])

B = np.array([
    [0],
    [(I + m*l**2)/d],
    [0],
    [-(m*l)/d]
])

# --- LQR COSTS ---
# Q: Penalty on state error [x, x_dot, theta, theta_dot]
# High value = tighter control.
Q = np.diag([10, 1, 100, 10]) 
# R: Penalty on actuator effort (electricity)
R = np.array([[0.1]])

# Solve Riccati Equation
P = solve_continuous_are(A, B, Q, R)
K = np.linalg.inv(R) @ B.T @ P

print("="*40)
print(f"Calculated Gains (K) for r={r}m:")
print(f"K = {K}")
print("="*40)
print("Copy this array into your main robot code.")
print("To convert Force to Current (Amps):")
print(f"Current_Command = (Force * {r}) / {Kt}")