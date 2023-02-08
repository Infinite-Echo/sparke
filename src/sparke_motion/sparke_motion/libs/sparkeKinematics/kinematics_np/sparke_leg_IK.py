import numpy as np
from . import leg_transformations as legtf
from . import base_transformations as basetf

class SparkeLeg():
    def __init__(self, leg_id):
        self.init_variables(leg_id)
        self.initialize_leg_transforms()

    def init_variables(self, leg_id):
        self.x_dir, self.y_dir = legtf.get_dirs(leg_id)
        self.len1 = 0.055
        self.len2 = 0.125
        self.len3 = 0.135

    def initialize_leg_transforms(self):
        self.t_01 = legtf.create_T01(0)
        self.t_12 = legtf.create_T12(0)
        self.t_23 = legtf.create_T23(0)

    def update_Tb0(self, Tm):
        self.t_b0 = legtf.create_Tb0(Tm, self.x_dir, self.y_dir)

    def solve_angles(self, Tm, x_ee, y_ee, z_ee):
        self.update_Tb0(Tm)
        self.solve_theta1(y_ee, z_ee)
        self.solve_theta3(x_ee, z_ee)
        self.solve_theta2(x_ee, z_ee)

    def solve_theta1(self, y_ee, z_ee): #CONFIRMED WORKING DO NOT GET RID OF
        y_ee, z_ee = abs(y_ee), abs(z_ee)
        y0, z0 = abs(self.t_b0[1, 3]), abs(self.t_b0[2, 3])
        c = np.sqrt(((z_ee-z0)**2) + ((y_ee-y0)**2))
        b = np.sqrt((c**2)-(self.len1**2))
        thetaA = np.arctan2(abs(z_ee-z0), abs(y_ee-y0))
        thetaB = np.arctan2(b, self.len1)
        self.theta1 = thetaB - thetaA

    def solve_theta3(self, x_ee, z_ee):
        x_ee, z_ee = abs(x_ee), abs(z_ee)
        t_b1 = self.get_tb1()
        x1, z1 = abs(t_b1[0,3]), abs(t_b1[2,3])
        x1_ee = x_ee - x1
        z1_ee = z_ee - z1
        a = (x1_ee**2) + (z1_ee**2) - (self.len2**2) - (self.len3**2)
        b = -2*self.len2*self.len3
        self.theta3 = np.arccos(a/b)

    def solve_theta2(self, x_ee, z_ee):
        x_ee, z_ee = abs(x_ee), abs(z_ee)
        t_b1 = self.get_tb1()
        x1, z1 = abs(t_b1[0,3]), abs(t_b1[2,3])
        x1_3 = abs(x_ee - x1) * -1
        z1_3 = z_ee - z1
        alpha = np.arctan2(z1_3, x1_3)
        c = np.sqrt((x1_3**2) + (z1_3**2))
        a = (self.len3**2) - (self.len2**2) - (c**2)
        b = -2*self.len2*c
        beta = np.arccos(a/b)
        self.theta2 = alpha - beta

    def get_tb1(self):
        self.t_01 = legtf.create_T01(self.theta1)
        t_b1 = np.matmul(self.t_b0, self.t_01)
        return t_b1