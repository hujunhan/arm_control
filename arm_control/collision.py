# class for collision detection, using GJK algorithm
import numpy as np


class GJK:
    def __init__(self) -> None:
        self.simplex = [None] * 4
        self.init_direction = np.array([1, 0, 0])
        self.max_iterations = 100

    def check_collision(self, set_a, set_b):
        """check if two sets of points are intersecting

        Args:
            set_a (_type_): _description_
            set_b (_type_): _description_

        Returns:
            true if intersecting, false otherwise
        """
        # 1. initialize the simplex with the first point
        self.simplex[0] = self.support(set_a, set_b, self.init_direction)

        # 2. update the direction using the opposite of the first point as direction
        direction = -self.simplex[0]
        self.simplex[1] = self.support(set_a, set_b, direction)
        if np.dot(self.simplex[1], direction) <= 0:
            # if the new point does not pass the origin
            return False

        # 3. update the direction using the prependicular vector of the line
        direction = np.cross(
            np.cross(self.simplex[0] - self.simplex[1], -self.simplex[1]),
            self.simplex[0] - self.simplex[1],
        )
        self.simplex[2] = self.support(set_a, set_b, direction)
        if np.dot(self.simplex[2], direction) <= 0:
            return False
        direction = self.get_new_direction()
        # main loop
        for _ in range(self.max_iterations):
            # 4. get the new point
            self.simplex[3] = self.support(set_a, set_b, direction)
            if np.dot(self.simplex[3], direction) <= 0:
                return False

            # 5. check if the new point is in the simplex
            if self.check_origin_tetrahedron():
                print(f"collision detected after {_} iterations")
                return True

            # 6. update the direction
            direction = self.get_new_direction()

    def get_new_direction(self):
        N = len(self.simplex)
        new_direction = None
        if N == 4:  # delete the point that is furthest away from the origin
            self.simplex.pop(np.argmax(np.linalg.norm(self.simplex, axis=1)))

        # get the direction from the triangle
        # the normal of the triangle
        # the direction should towards the origin
        new_direction = np.cross(
            self.simplex[1] - self.simplex[0],
            self.simplex[2] - self.simplex[0],
        )
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
