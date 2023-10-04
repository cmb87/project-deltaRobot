import numpy as np 
import matplotlib.pyplot as plt
from scipy.spatial import Delaunay

# See https://hypertriangle.com/~alex/delta-robot-tutorial/

class Delta:

    sqrt3 = np.sqrt(3.0)
    pi = np.pi
    sin120 = sqrt3/2.0   
    cos120 = -0.5        
    tan60 = sqrt3
    sin30 = 0.5
    tan30 = 1/sqrt3
    
    def __init__(self,e=0.7,f=0.11,re=0.25,rf=0.4):
        self.e = e   
        self.f = f    
        self.re = re
        self.rf = rf
 

    def _calcAngleYZ(self, x0, y0, z0):

        y1 = -0.5 * 0.57735 * self.f # f/2 * tg 30
        y0 -= 0.5 * 0.57735 * self.e    # shift center to edge
        # z = a + b*y
        a = (x0*x0 + y0*y0 + z0*z0 + self.rf*self.rf - self.re*self.re - y1*y1)/(2*z0)
        b = (y1-y0)/z0
        # discriminant
        d = -(a+b*y1)*(a+b*y1)+self.rf*(b*b*self.rf+self.rf) 
        if (d < 0):
            return -1, None # non-existing point

        yj = (y1 - a*b - np.sqrt(d))/(b*b + 1) # choosing outer point
        zj = a + b*yj

        if (yj>y1):
            theta = 180.0*np.arctan(-zj/(y1 - yj))/np.pi + 180.0
        else:
            theta = 180.0*np.arctan(-zj/(y1 - yj))/np.pi

        return 0, theta


    def inverse(self, x0, y0, z0):
        # inverse kinematics: (x0, y0, z0) -> (theta1, theta2, theta3)
        # returned status: 0=OK, -1=non-existing position
        status, theta1 = self._calcAngleYZ(x0, y0, z0)

        if (status == 0):
            status, theta2 = self._calcAngleYZ(x0*self.cos120 + y0*self.sin120, y0*self.cos120-x0*self.sin120, z0)  # rotate coords to +120 deg
        if (status == 0):
            status, theta3 = self._calcAngleYZ(x0*self.cos120 - y0*self.sin120, y0*self.cos120+x0*self.sin120, z0)  # rotate coords to -120 deg

        return status, [theta1, theta2, theta3]


    def calcForward(self,theta):
        [theta1,  theta2,  theta3] = theta
        t = (self.f-self.e)*self.tan30/2
        dtr = self.pi/180.0
    
        theta1 *= dtr
        theta2 *= dtr
        theta3 *= dtr
    
        y1 = -(t + self.rf*np.cos(theta1))
        z1 = -self.rf*np.sin(theta1)
    
        y2 = (t + self.rf*np.cos(theta2))*self.sin30
        x2 = y2*self.tan60
        z2 = -self.rf*np.sin(theta2)
    
        y3 = (t + self.rf*np.cos(theta3))*self.sin30
        x3 = -y3*self.tan60
        z3 = -self.rf*np.sin(theta3)
    
        dnm = (y2-y1)*x3-(y3-y1)*x2
    
        w1 = y1*y1 + z1*z1
        w2 = x2*x2 + y2*y2 + z2*z2
        w3 = x3*x3 + y3*y3 + z3*z3
        
        # x = (a1*z + b1)/dnm
        a1 = (z2-z1)*(y3-y1)-(z3-z1)*(y2-y1)
        b1 = -((w2-w1)*(y3-y1)-(w3-w1)*(y2-y1))/2.0
    
        # y = (a2*z + b2)/dnm
        a2 = -(z2-z1)*x3+(z3-z1)*x2
        b2 = ((w2-w1)*x3 - (w3-w1)*x2)/2.0
    
        # a*z^2 + b*z + c = 0
        a = a1*a1 + a2*a2 + dnm*dnm
        b = 2*(a1*b1 + a2*(b2-y1*dnm) - z1*dnm*dnm)
        c = (b2-y1*dnm)*(b2-y1*dnm) + b1*b1 + dnm*dnm*(z1*z1 - self.re*self.re)
    
        # discriminant
        d = b*b - 4.0*a*c
        if (d < 0): return -1 # non-existing point
    
        z0 = -0.5*(b+np.sqrt(d))/a
        x0 = (a1*z0 + b1)/dnm
        y0 = (a2*z0 + b2)/dnm
        return [x0,  y0,  z0]

if __name__ == "__main__":

    np.random.seed(42)

    thetaMin, thetaMax = -90.0, 180.0
    nmax = 10000
    theta = thetaMin + (thetaMax-thetaMin) * np.random.rand(nmax,3)

    delta = Delta(e=0.138,f=0.474,re=0.384,rf=0.18)

    
    x0 = delta.calcForward([40.0,40.0,40.0])
    valid, theta1 = delta.inverse(*[-0.2, 0.0, -0.4194879595738985])
    valid, theta2 = delta.inverse(*[+0.2, 0.0, -0.4194879595738985])
    valid, theta3 = delta.inverse(*[0.0, -0.2, -0.4194879595738985])
    valid, theta4 = delta.inverse(*[0.0, +0.2, -0.4194879595738985])
    print(theta1)
    print(theta2)
    print(theta3)
    print(theta4)
    sys.exit()


    x = np.stack([x  for x in [delta.calcForward(theta[n,:])  for n in range(nmax)] if not x == -1 ])
    x = x[x[:,2]<=0, :]

    x2d = x[:,[0,2]]
    tri = Delaunay(x2d)

    # Calculate surface area
    surface_triangles = x2d[tri.simplices]

    triangle_areas = np.linalg.norm(np.cross(surface_triangles[1,:] - surface_triangles[0,:],
                                            surface_triangles[2,:] - surface_triangles[0,:])) / 2
    surface_area = np.sum(triangle_areas)
    print(triangle_areas)
    print(surface_area)

    plt.triplot(x[:,0], x[:,2], tri.simplices)

    plt.title(f"e={delta.e}, f={delta.f}, re={delta.re}, rf={delta.rf}")
    plt.plot([-delta.e/2,delta.e/2], [0,0],'k')
    plt.plot(x[:,0], x[:,2],'.')
    plt.axis("equal")
    plt.axis([-0.4,0.4,-0.9,0.01])
    plt.grid(True)
    plt.show()

