# class for collision detection, using GJK algorithm
import numpy as np
from loguru import logger as log

log.remove()


class GJK:
    def __init__(self) -> None:
        self.simplex = []
        self.init_direction = np.random.rand(3) - 0.5
        log.debug(f"init direction: {self.init_direction}")
        self.abandoned_simplex = []
        self.max_iterations = 100

    def super_check(self, set_a, set_b, times=3):
        """use check_collision multiple times to increase the chance of correct result

        Args:
            set_a (_type_): _description_
            set_b (_type_): _description_
            times (int, optional): _description_. Defaults to 3.
        """
        for i in range(times):
            if self.check_collision(set_a, set_b):
                return True
        return False

    def check_collision(self, set_a, set_b):
        """check if two sets of points are intersecting

        Args:
            set_a (_type_): _description_
            set_b (_type_): _description_

        Returns:
            true if intersecting, false otherwise
        """
        self.__init__()
        # 1. initialize the simplex with the first point
        new_point = self.support(set_a, set_b, self.init_direction)
        self.simplex.append(new_point)
        log.debug(f"first point: {self.simplex[0]}")
        # 2. update the direction using the opposite of the first point as direction
        direction = -self.simplex[0]
        new_point = self.support(set_a, set_b, direction)
        self.simplex.append(new_point)
        if np.dot(self.simplex[1], direction) <= 0:
            # if the new point does not pass the origin
            log.debug(f"the second point does not pass the origin")
            return False
        log.debug(f"second point: {self.simplex[1]}")
        # 3. update the direction using the prependicular vector of the line
        direction = np.cross(
            np.cross(self.simplex[0] - self.simplex[1], -self.simplex[1]),
            self.simplex[0] - self.simplex[1],
        )
        new_point = self.support(set_a, set_b, direction)
        self.simplex.append(new_point)
        log.debug(f"third point: {self.simplex[2]}")
        if np.dot(self.simplex[2], direction) <= 0:
            log.debug(f"the third point does not pass the origin")
            return False
        direction = self.get_new_direction()
        # main loop
        for _ in range(self.max_iterations):
            # 4. get the new point
            new_point = self.support(set_a, set_b, direction)
            log.debug(f"new point: {new_point}")
            self.simplex.append(new_point)
            if np.dot(self.simplex[3], direction) <= 0:
                return False

            # 5. check if the new point is in the simplex
            if self.check_origin_tetrahedron():
                log.debug(f"collision detected after {_} iterations")
                return True

            # 6. update the direction
            direction = self.get_new_direction()
            log.debug(f"new direction: {direction}")
        log.debug("max iterations reached")
        return False

    def get_new_direction(self):
        N = len(self.simplex)
        new_direction = None
        repeated = False
        if N == 4:  # delete the point that is furthest away from the origin
            pop_index = np.argmax(np.linalg.norm(self.simplex, axis=1))

            if any(
                np.array_equal(self.simplex[pop_index], x)
                for x in self.abandoned_simplex
            ):  ## use any to check if the point is in the abandoned simplex, use 'in' is not working
                repeated = True
                self.simplex.pop(pop_index)
            else:
                self.abandoned_simplex.append(self.simplex.pop(pop_index))

        # get the direction from the triangle
        # the normal of the triangle
        # the direction should towards the origin
        new_direction = np.cross(
            self.simplex[1] - self.simplex[0],
            self.simplex[2] - self.simplex[0],
        )
        if repeated:  # add small disturbance to the direction
            dir_norm = np.linalg.norm(new_direction)
            new_direction += 0.1 * dir_norm * (np.random.rand(3) - 0.5)
        if np.dot(new_direction, -self.simplex[0]) < 0:
            new_direction = -new_direction
        return new_direction

    def support(self, set_a, set_b, direction):
        """calculate the support point of two sets of points in a given direction

        Args:
            set_a (_type_): _description_
            set_b (_type_): _description_
            direction (_type_): _description_
        """
        return self.support_point(set_a, direction) - self.support_point(
            set_b, -direction
        )

    def support_point(self, points, direction):
        """calculate the support point of a set of points in a given direction

        Args:
            points (_type_): _description_
            direction (_type_): _description_

        Returns:
            _type_: _description_
        """
        index = np.argmax(np.dot(points, direction))
        log.debug(f"support point index: {index}")
        return points[index]

    def check_origin_tetrahedron(self):
        """check if the origin is in the tetrahedron"""
        v1 = self.simplex[0]
        v2 = self.simplex[1]
        v3 = self.simplex[2]
        v4 = self.simplex[3]
        return (
            self.same_side(v1, v2, v3, v4)
            and self.same_side(v2, v3, v4, v1)
            and self.same_side(v3, v4, v1, v2)
            and self.same_side(v4, v1, v2, v3)
        )

    def same_side(self, v1, v2, v3, v4):
        normal = np.cross(v2 - v1, v3 - v1)
        return np.dot(normal, v4 - v1) * np.dot(normal, -v1) > 0


if __name__ == "__main__":
    # init two sets of points
    ## using meshgrid to generate a cube
    import matplotlib.pyplot as plt

    left_down = np.array([0, -1, -1])
    right_up = np.array([1, 1, 1])
    set_a = np.array(
        np.meshgrid(*[np.linspace(*x, num=10) for x in zip(left_down, right_up)])
    ).T.reshape(-1, 3)

    left_down = np.array([0.5, 1.1, 1.1])
    right_up = np.array([2, 2, 2])
    set_b = np.array(
        np.meshgrid(*[np.linspace(*x, num=10) for x in zip(left_down, right_up)])
    ).T.reshape(-1, 3)
    # plot the cube for verification, 3d plot
    fig, ax = plt.subplots(subplot_kw={"projection": "3d"})
    ax.scatter(set_a[:, 0], set_a[:, 1], set_a[:, 2], c="r")
    ax.scatter(set_b[:, 0], set_b[:, 1], set_b[:, 2], c="b")
    plt.show()

    detector = GJK()
    collision = detector.check_collision(set_a, set_b)
    print(collision)
