#!usr/bin/python
import numpy as np
import time
import sys
from numpy.linalg import pinv


def AgentTrajectories(p0i, pfi, v0i, vfi, t0, tf):
    """

    :param p0i: an array with 3 elements
    :param pfi: an array with 3 elements
    :param v0i: an array with 3 elements
    :param vfi: an array with 3 elements
    :param t0: initial time
    :param tf: final time
    :return:
    """
    # initial position constraint matrix and vector
    # Change to Yiming's format

    p0 = np.array([[p0i[0]], [p0i[1]], [p0i[2]]])
    pf = np.array([[pfi[0]], [pfi[1]], [pfi[2]]])
    v0 = np.array([[v0i[0]], [v0i[1]], [v0i[2]]])
    vf = np.array([[vfi[0]], [vfi[1]], [vfi[2]]])

    Ai = np.array([[t0 ** 3 / 6, 0, 0, t0 ** 2 / 2, 0, 0, t0, 0, 0, 1, 0, 0],
                   [0, t0 ** 3 / 6, 0, 0, t0 ** 2 / 2, 0, 0, t0, 0, 0, 1, 0],
                   [0, 0, t0 ** 3 / 6, 0, 0, t0 ** 2 / 2, 0, 0, t0, 0, 0, 1],
                   [t0 ** 2 / 2, 0, 0, t0, 0, 0, 1, 0, 0, 0, 0, 0],
                   [0, t0 ** 2 / 2, 0, 0, t0, 0, 0, 1, 0, 0, 0, 0],
                   [0, 0, t0 ** 2 / 2, 0, 0, t0, 0, 0, 1, 0, 0, 0]])
    Bi = np.append(p0, v0, axis=0)
    # final position constraint matrix and vector
    Af = np.array([[tf ** 3 / 6, 0, 0, tf ** 2 / 2, 0, 0, tf, 0, 0, 1, 0, 0],
                   [0, tf ** 3 / 6, 0, 0, tf ** 2 / 2, 0, 0, tf, 0, 0, 1, 0],
                   [0, 0, tf ** 3 / 6, 0, 0, tf ** 2 / 2, 0, 0, tf, 0, 0, 1],
                   [tf ** 2 / 2, 0, 0, tf, 0, 0, 1, 0, 0, 0, 0, 0],
                   [0, tf ** 2 / 2, 0, 0, tf, 0, 0, 1, 0, 0, 0, 0],
                   [0, 0, tf ** 2 / 2, 0, 0, tf, 0, 0, 1, 0, 0, 0]])
    Bf = np.append(pf, vf, axis=0)

    A = np.append(Ai, Af, axis=0)
    B = np.append(Bi, Bf, axis=0)
    Ainv = np.linalg.inv(A)
    C = np.matmul(Ainv, B)
    # % this is a faster version of inverse(A) * B.
    C = np.concatenate(C)
    cx = C[0::3]
    cy = C[1::3]
    cz = C[2::3]

    return cx, cy, cz
