from drone_explore_env import DroneExploreEnv
import numpy as np

env1 = DroneExploreEnv(drone_name = 'Drone_1')
env1.render()

start_time1 = np.uint64(0)
final_time1 = np.uint64(0)

i=0
while(i<5000):
    action = env1.random_action()
    if i == 0:
        start_time = env1.step(action)
    elif i == 4999:
        final_time = env1.step(action)
    else:
        env1.step(action)
    i += 1

print(start_time)
print(final_time)
print(final_time - start_time)

# a = np.array([[0,1,2,3,2,3],[0,1,2,3,0,1]])
# print(a)
# b = np.where(a != 1)
# print(len(b[0]))

