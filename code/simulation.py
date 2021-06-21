import pendulum as p
import numpy as np

properties = {'m' : 1.9,
            'g' : 9.81,
            'arm' : 0.14,
            'R' : 4.2,
            'I' : 0.035,
            'k_m' : 0.0584,
            'I_m' : 0.00403,
            't' : np.linspace(0, 10, 1000),
            'L' : np.array([-550, -60, -0.0885, 1000]),
            'x0' : np.array([-0.4, 0.5, 1, 0])}

sim = p.Pendulum(properties)

if __name__ == '__main__':
    sim.solve()