# debug the GJK algorithm using the generated error cases
import numpy as np
import pickle
import matplotlib.pyplot as plt
from arm_control.collision import GJK
from loguru import logger as log

bad_cases = pickle.load(open("doc/error_collision.pkl", "rb"))

set_a = bad_cases[0][0]
set_b = bad_cases[0][1]
overlap = bad_cases[0][2]
fig, ax = plt.subplots(subplot_kw={"projection": "3d"})
ax.scatter(set_a[:, 0], set_a[:, 1], set_a[:, 2], c="r")
ax.scatter(set_b[:, 0], set_b[:, 1], set_b[:, 2], c="b")
# plt.show()

detector = GJK()
collision = detector.check_collision(set_a, set_b)
log.debug(f"collision: {collision} overlap: {overlap}")
simplex = np.array(detector.simplex)
ax.scatter(simplex[:, 0], simplex[:, 1], simplex[:, 2], c="g")
plt.show()
