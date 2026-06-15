import numpy as np
from dynamic_model import TractorTrailerDynamicModel

model = TractorTrailerDynamicModel()
initial_pos = np.array([0.0, 0.0, 0.0, -2.12, 0.0, 0.0, -2.76, 0.0, 0.0])
initial_vel = np.array([5.0, 0.0, 0.0, 5.0, 0.0, 0.0, 5.0, 0.0, 0.0])
state = {'positions': initial_pos, 'velocities': initial_vel}

dt = 0.001
delta = np.radians(10.0)

for i in range(1000):
    res = model.step(state, delta, dt=dt)
    state = {'positions': res['positions'], 'velocities': res['velocities']}
    
    if np.any(np.isnan(state['velocities'])) or np.max(np.abs(state['velocities'])) > 100:
        print(f"Exploded at step {i}, t={i*dt}s")
        print("Velocities:", state['velocities'])
        print("Accelerations:", res['accelerations'])
        break

print("Done")
