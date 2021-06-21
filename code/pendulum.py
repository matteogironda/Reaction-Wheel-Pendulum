import numpy as np
from scipy.integrate import odeint
from matplotlib import pyplot as plt

### Inverted pendulum dynamics simulation ###

class Pendulum:

    def __init__(self, prop):

        self.m = prop['m']
        self.g = prop['g']
        self.arm = prop['arm']
        self.R = prop['R']
        self.I = prop['I']
        self.k_m = prop['k_m']
        self.I_m = prop['I_m']
        self.t = prop['t']
        self.L = prop['L']
        self.x0 = prop['x0']
        
    def odes(self, x, t):

        r = 0
        x1 = x[0]; x2 = x[1]; x3 = x[2]; x4 = x[3]
        u = np.matmul(self.L,x)

        x1_dot = x2 
        x2_dot = self.m*self.g*self.arm/self.I * x1 + self.k_m**2/(self.I*self.R) * x3 - u * -self.k_m/(self.I*self.R)
        x3_dot = -self.k_m**2/(self.I_m*self.R) * x3 - u * -self.k_m/(self.I_m*self.R)
        x4_dot = r - x1 

        return [x1_dot, x2_dot, x3_dot, x4_dot]

    def controller(self):
        
        u = -np.matmul(self.L, self.x) 

        return u

    def solve(self):

        x = odeint(self.odes, self.x0, self.t)
        theta = x[:,0]
        theta_dot = x[:,1]
        omega = x[:,2]

        u = []
        for i in range(len(self.t)):
            u.append(np.matmul(-self.L, x[i,:]))

        print(u[1])
        # plot the results
        plt.plot(self.t,theta)
        plt.plot(self.t,theta_dot)
        #plt.plot(self.t,omega)
        #plt.plot(self.t,u)

        plt.legend(['theta','theta_dot','omega'])
        plt.show()

    def animate(self):
        pass

