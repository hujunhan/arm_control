import numpy as np


def gjk_intersection(a, b, max_iterations=100):
    simplex = [None] * 4
    simplex[0] = support(a, b, np.array([1, 0, 0]))
    direction = -simplex[0]
    simplex[1] = support(a, b, direction)

    if np.dot(simplex[1], direction) <= 0:
        return False

    direction = -simplex[1]
    simplex[2] = support(a, b, direction)

    if np.dot(simplex[2], direction) <= 0:
        return False

    direction = np.cross(simplex[1] - simplex[0], simplex[2] - simplex[0])

    for i in range(max_iterations):
        simplex[3] = support(a, b, direction)

        if np.dot(simplex[3], direction) <= 0:
            return False

        if contains_origin(simplex, direction):
            return True

        direction = compute_new_direction(simplex)

    return False


def support(a, b, direction):
    return support_point(a, direction) - support_point(b, -direction)


def support_point(points, direction):
    index = np.argmax(np.dot(points, direction))
    return points[index]


def contains_origin(simplex, direction):
    a, b, c, d = simplex
    ab = b - a
    ac = c - a
    ad = d - a
    ao = -a

    abp = np.cross(ab, ac)
    if np.dot(abp, ao) > 0:
        simplex.remove(d)
        return False

    acp = np.cross(ac, ab)
    if np.dot(acp, ao) > 0:
        simplex.remove(d)
        return False

    if np.dot(ab, ao) > 0:
        simplex.remove(c)
        direction = np.cross(ab, ao)
    else:
        if np.dot(ac, ao) > 0:
            simplex.remove(b)
            direction = np.cross(ac, ao)
        else:
            return True

    simplex[0] = a
    simplex[1] = b
    simplex[2] = c
    simplex[3] = support_point(points, direction)

    return False


def compute_new_direction(simplex):
    c, b, a = simplex[-1], simplex[-2], simplex[-3]
    ab = b - a
    ac = c - a
    abp = np.cross(ab, ac)

    if np.dot(abp, -a) > 0:
        return np.cross(np.cross(ab, -a), ab)

    return abp
