import numpy as np
import scipy.interpolate as interp

# Initialize random values
""" x_initial = -np.random.rand() * 10
y_initial = np.random.rand() * 2 - 1
z_initial = np.random.rand() * 8 + 2 """

x_initial = -10
y_initial = -1
z_initial = 10

# Define waypoints
wpts = np.array([[x_initial, 0], 
                 [0, 0], 
                 [z_initial, 0]])

# Compute basic time determination
distance = np.sqrt(x_initial**2 + y_initial**2)
t_end = distance
tpts = np.array([0, t_end])

# Number of samples
numsamples = 100
tsamples = np.linspace(0, t_end, numsamples)

# Function to generate minimum jerk trajectory
def min_jerk_trajectory(wpts, tpts, tsamples):
    q, qd, qdd, qddd = [], [], [], []
    
    for i in range(wpts.shape[0]):
        # Interpolating cubic spline to simulate min jerk trajectory
        cs = interp.CubicSpline(tpts, wpts[i], bc_type=((1, 0), (1, 0)))  # First derivatives at endpoints = 0
        q.append(cs(tsamples))          # Position
        qd.append(cs(tsamples, 1))      # Velocity
        qdd.append(cs(tsamples, 2))     # Acceleration
        qddd.append(cs(tsamples, 3))    # Jerk
    
    return np.array(q), np.array(qd), np.array(qdd), np.array(qddd)

# Compute trajectory
q, qd, qdd, qddd = min_jerk_trajectory(wpts, tpts, tsamples)

# Print results
print("Positions:\n", q)
print("Velocities:\n", qd)
print("Accelerations:\n", qdd)
print("Jerks:\n", qddd)
print("Time: ", tsamples)
