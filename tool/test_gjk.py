# script to test the GJK algorithm
# generating random rectangular and compare the result

import numpy as np
from arm_control.collision import GJK
from loguru import logger as log
import pickle

import time

log.remove()
TEST_NUM = 10000
error_count = 0

error_collision = []
start = time.time()
for i in range(TEST_NUM):
    # generate two random rectangular in -1 to 1 (x, y, z)

    left_down = np.random.rand(3) - 0.5
    right_up = np.random.rand(3) - 0.5
    # get the max_x, max_y, max_z, min_x, min_y, min_z

    rect_a = np.array(
        np.meshgrid(*[np.linspace(*x, num=10) for x in zip(left_down, right_up)])
    ).T.reshape(-1, 3)
    a_max = rect_a.max(axis=0)
    a_min = rect_a.min(axis=0)
    left_down = np.random.rand(3) - 0.5
    right_up = np.random.rand(3) - 0.5
    rect_b = np.array(
        np.meshgrid(*[np.linspace(*x, num=10) for x in zip(left_down, right_up)])
    ).T.reshape(-1, 3)
    b_max = rect_b.max(axis=0)
    b_min = rect_b.min(axis=0)
    if (a_max > b_min).all() and (a_min < b_max).all():
        overlap = True
    else:
        overlap = False
    # check if the two rectangulars are intersecting
    detector = GJK()

    result = detector.super_check(rect_a, rect_b)
    # check if the result is correct
    if result != overlap:
        print(f"error: {i}")
        print(f"{i}: {result} {overlap}")
        error_collision.append([rect_a, rect_b, overlap])
        error_count += 1
end = time.time()
# save the error cases
pickle.dump(error_collision, open("doc/error_collision.pkl", "wb"))
print(f"error rate: {error_count / TEST_NUM}")
print(f"total time: {end - start}")
print(f"average time: {(end - start) / TEST_NUM}")
