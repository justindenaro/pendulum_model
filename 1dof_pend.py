import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp

# Simplified Pendulum Dynamics (Cart and rod with friction)
def pendcart(t, y, M, m, l, I, g, F, b):
    x, x_dot, theta, theta_dot = y

    #Nonlinear ODEs
    st = np.sin(theta)
    ct = np.cos(theta)

    # Effective force includes the external force and the friction
    F_eff = F - b * x_dot

    # Denominator of the system matrix
    det = (M + m) * (I + m * l**2) - (m * l * ct)**2

    # Modified accelerations including damping b
    x_accel = ((I + m*l**2)*(F_eff + m*l*theta_dot**2*st) - (m**2*l**2*g*ct*st)) / det
    theta_accel = ((M+m)*m*g*l*st - m*l*ct*(F_eff + m*l*theta_dot**2*st)) / det

    return [x_dot, x_accel, theta_dot, theta_accel]

def cart_control(sol, t):
    # Placeholder for cart control logic
    # Input current ode sol of pendulum, utilize Kp, Ki, Kd to figure out cart movement
    # Return new force
    pass

# Parameters
flag = 1  # Simulation running flag
l = 1.0 # length (m)
g = 9.81 # gravity (m/s^2)
m = 0.5 # mass pendulum (kg)
M = 2.0 # mass cart (kg)
I = (m*l**2)/12 # moment of inertia (kg*m^2) (ML^2/12 for symmetric rod)
F = 0.0 # Initial external force on cart (N)
b = 0.7 # Coefficient of friction for cart
params = (M, m, l, I, g, F, b)

# Initial conditions and common terms
x0 = 0.0 # initial cart position (m)
v0 = 0.1 #initial cart velocity (m/s)
theta0 = np.pi/2 # initial pendulum angle (radians)
omega0 = 9.9 # initial angular velocity (rad/s)
y0 = [x0, v0, theta0, omega0]   # Initial state vector
t_span = (0, 60) #time span for simulation
t_eval = (0, 60, 1000) # time points, 1000 from 0-60 seconds

# Solve ODE
sol = solve_ivp(
    fun=pendcart,
    t_span=t_span,
    y0=y0,
    t_eval=t_eval,
    args=params
)

print(sol)

# Visualization
plt.figure(figsize=(10,4))
plt.plot(sol.t, sol.y[0], label='Cart Position (m)')
plt.plot(sol.t, sol.y[2], label='Pendulum Angle (rad)')
plt.title('Dynamics with cart Friction')
plt.xlabel('Time (s)')
plt.legend()
plt.grid(True)
plt.show()