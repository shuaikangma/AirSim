from drone_explore_env import DroneExploreEnv
import numpy as np

env1 = DroneExploreEnv(drone_name = 'Drone_1')
env2 = DroneExploreEnv(drone_name = 'Drone_1')

start_time1 = np.uint64(0)
final_time1 = np.uint64(0)

i=0
while(i<5000):
    if i == 0:
        start_time = env1.step(0)
    elif i == 4999:
        final_time = env1.step(0)
    else:
        env2.step(0)
    env2.step(0)
    i += 1

print(start_time)
print(final_time)
print(final_time - start_time)

