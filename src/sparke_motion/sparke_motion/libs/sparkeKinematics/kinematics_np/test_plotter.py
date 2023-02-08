import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from sparke_leg_IK import SparkeLeg as spLegIK
import base_transformations as basetf
import leg_transformations as legtf

def plot_points(arr1, arr2, arr3, arr4, arr5, arr6, arr7, arr8):
    # Create figure and 3D axis
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # Plot the points from arr1
    ax.plot(arr1[:,0], arr1[:,1], arr1[:,2], c='r', marker='o')
    ax.plot(arr3[:,0], arr3[:,1], arr3[:,2], c='r', marker='o')
    ax.plot(arr5[:,0], arr5[:,1], arr5[:,2], c='r', marker='o')
    ax.plot(arr7[:,0], arr7[:,1], arr7[:,2], c='r', marker='o')

    # Plot the points from arr2
    ax.plot(arr2[:,0], arr2[:,1], arr2[:,2], c='b', marker='^')
    ax.plot(arr4[:,0], arr4[:,1], arr4[:,2], c='b', marker='^')
    ax.plot(arr6[:,0], arr6[:,1], arr6[:,2], c='b', marker='^')
    ax.plot(arr8[:,0], arr8[:,1], arr8[:,2], c='b', marker='^')

    # Set axis labels
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')

    # Show the plot
    plt.show()

def main():
    Tm = basetf.create_base_transformation(0, 0, -0.05, 0, -np.pi/24, 0)
    tb0 = legtf.create_Tb0(Tm, 1, 1)
    t01 = legtf.create_T01(0)
    t12 = legtf.create_T12(np.pi/4)
    t23 = legtf.create_T23(np.pi/2)
    tb1 = np.matmul(tb0, t01)
    tb2 = np.matmul(tb1, t12)
    tb3 = np.matmul(tb2, t23)
    x_ee = tb3[0,3] + 0.1
    y_ee = tb3[1,3]
    z_ee = tb3[2,3]
    leg1 = spLegIK(1)
    leg1.solve_angles(Tm, x_ee, y_ee, z_ee)
    print(f'theta1: {leg1.theta1}')
    print(f'theta2: {leg1.theta2}')
    print(f'theta3: {leg1.theta3}')

    
    tb0 = legtf.create_Tb0(Tm, 1, -1)
    t01 = legtf.create_T01(0)
    t12 = legtf.create_T12(np.pi/4)
    t23 = legtf.create_T23(np.pi/2)
    tb1 = np.matmul(tb0, t01)
    tb2 = np.matmul(tb1, t12)
    tb3 = np.matmul(tb2, t23)
    x_ee = tb3[0,3]
    y_ee = tb3[1,3]
    z_ee = tb3[2,3]
    leg2 = spLegIK(2)
    leg2.solve_angles(Tm, x_ee, y_ee, z_ee)
    print(f'theta1: {leg2.theta1}')
    print(f'theta2: {leg2.theta2}')
    print(f'theta3: {leg2.theta3}')

    
    tb0 = legtf.create_Tb0(Tm, -1, 1)
    t01 = legtf.create_T01(0)
    t12 = legtf.create_T12(np.pi/4)
    t23 = legtf.create_T23(np.pi/2)
    tb1 = np.matmul(tb0, t01)
    tb2 = np.matmul(tb1, t12)
    tb3 = np.matmul(tb2, t23)
    x_ee = tb3[0,3]
    y_ee = tb3[1,3]
    z_ee = tb3[2,3]
    leg3 = spLegIK(3)
    leg3.solve_angles(Tm, x_ee, y_ee, z_ee)
    print(f'theta1: {leg3.theta1}')
    print(f'theta2: {leg3.theta2}')
    print(f'theta3: {leg3.theta3}')

    
    tb0 = legtf.create_Tb0(Tm, -1, -1)
    t01 = legtf.create_T01(0)
    t12 = legtf.create_T12(np.pi/4)
    t23 = legtf.create_T23(np.pi/2)
    tb1 = np.matmul(tb0, t01)
    tb2 = np.matmul(tb1, t12)
    tb3 = np.matmul(tb2, t23)
    x_ee = tb3[0,3]
    y_ee = tb3[1,3]
    z_ee = tb3[2,3]
    leg4 = spLegIK(4)
    leg4.solve_angles(Tm, x_ee, y_ee, z_ee)
    print(f'theta1: {leg4.theta1}')
    print(f'theta2: {leg4.theta2}')
    print(f'theta3: {leg4.theta3}')

    f_Tb0 = legtf.create_Tb0(Tm, 1, 1)
    f_t01 = legtf.create_T01(0)
    f_t12 = legtf.create_T12(np.pi/4)
    f_t23 = legtf.create_T23(np.pi/2)
    f_Tb1 = np.matmul(f_Tb0, f_t01)
    f_Tb2 = np.matmul(f_Tb1, f_t12)
    f_Tb3 = np.matmul(f_Tb2, f_t23)
    f_x0, f_y0, f_z0 = f_Tb0[0,3], f_Tb0[1,3], f_Tb0[2,3]
    f_x1, f_y1, f_z1 = f_Tb1[0,3], f_Tb1[1,3], f_Tb1[2,3]
    f_x2, f_y2, f_z2 = f_Tb2[0,3], f_Tb2[1,3], f_Tb2[2,3]
    f_x3, f_y3, f_z3 = f_Tb3[0,3], f_Tb3[1,3], f_Tb3[2,3]

    f2_Tb0 = legtf.create_Tb0(Tm, 1, -1)
    f2_t01 = legtf.create_T01(0)
    f2_t12 = legtf.create_T12(np.pi/4)
    f2_t23 = legtf.create_T23(np.pi/2)
    f2_Tb1 = np.matmul(f2_Tb0, f2_t01)
    f2_Tb2 = np.matmul(f2_Tb1, f2_t12)
    f2_Tb3 = np.matmul(f2_Tb2, f2_t23)
    f2_x0, f2_y0, f2_z0 = f2_Tb0[0,3], f2_Tb0[1,3], f2_Tb0[2,3]
    f2_x1, f2_y1, f2_z1 = f2_Tb1[0,3], f2_Tb1[1,3], f2_Tb1[2,3]
    f2_x2, f2_y2, f2_z2 = f2_Tb2[0,3], f2_Tb2[1,3], f2_Tb2[2,3]
    f2_x3, f2_y3, f2_z3 = f2_Tb3[0,3], f2_Tb3[1,3], f2_Tb3[2,3]

    f3_Tb0 = legtf.create_Tb0(Tm, -1, 1)
    f3_t01 = legtf.create_T01(0)
    f3_t12 = legtf.create_T12(np.pi/4)
    f3_t23 = legtf.create_T23(np.pi/2)
    f3_Tb1 = np.matmul(f3_Tb0, f3_t01)
    f3_Tb2 = np.matmul(f3_Tb1, f3_t12)
    f3_Tb3 = np.matmul(f3_Tb2, f3_t23)
    f3_x0, f3_y0, f3_z0 = f3_Tb0[0,3], f3_Tb0[1,3], f3_Tb0[2,3]
    f3_x1, f3_y1, f3_z1 = f3_Tb1[0,3], f3_Tb1[1,3], f3_Tb1[2,3]
    f3_x2, f3_y2, f3_z2 = f3_Tb2[0,3], f3_Tb2[1,3], f3_Tb2[2,3]
    f3_x3, f3_y3, f3_z3 = f3_Tb3[0,3], f3_Tb3[1,3], f3_Tb3[2,3]

    f4_Tb0 = legtf.create_Tb0(Tm, -1, -1)
    f4_t01 = legtf.create_T01(0)
    f4_t12 = legtf.create_T12(np.pi/4)
    f4_t23 = legtf.create_T23(np.pi/2)
    f4_Tb1 = np.matmul(f4_Tb0, f4_t01)
    f4_Tb2 = np.matmul(f4_Tb1, f4_t12)
    f4_Tb3 = np.matmul(f4_Tb2, f4_t23)
    f4_x0, f4_y0, f4_z0 = f4_Tb0[0,3], f4_Tb0[1,3], f4_Tb0[2,3]
    f4_x1, f4_y1, f4_z1 = f4_Tb1[0,3], f4_Tb1[1,3], f4_Tb1[2,3]
    f4_x2, f4_y2, f4_z2 = f4_Tb2[0,3], f4_Tb2[1,3], f4_Tb2[2,3]
    f4_x3, f4_y3, f4_z3 = f4_Tb3[0,3], f4_Tb3[1,3], f4_Tb3[2,3]

    i_Tb0 = legtf.create_Tb0(Tm, 1, 1)
    i_t01 = legtf.create_T01(leg1.theta1)
    i_t12 = legtf.create_T12(leg1.theta2 )
    i_t23 = legtf.create_T23(leg1.theta3 )
    i_Tb1 = np.matmul(i_Tb0, i_t01)
    i_Tb2 = np.matmul(i_Tb1, i_t12)
    i_Tb3 = np.matmul(i_Tb2, i_t23)
    i_x0, i_y0, i_z0 = i_Tb0[0,3], i_Tb0[1,3], i_Tb0[2,3]
    i_x1, i_y1, i_z1 = i_Tb1[0,3], i_Tb1[1,3], i_Tb1[2,3]
    i_x2, i_y2, i_z2 = i_Tb2[0,3], i_Tb2[1,3], i_Tb2[2,3]
    i_x3, i_y3, i_z3 = i_Tb3[0,3], i_Tb3[1,3], i_Tb3[2,3]

    i2_Tb0 = legtf.create_Tb0(Tm, 1, -1)
    i2_t01 = legtf.create_T01(leg2.theta1)
    i2_t12 = legtf.create_T12(leg2.theta2 )
    i2_t23 = legtf.create_T23(leg2.theta3 )
    i2_Tb1 = np.matmul(i2_Tb0, i2_t01)
    i2_Tb2 = np.matmul(i2_Tb1, i2_t12)
    i2_Tb3 = np.matmul(i2_Tb2, i2_t23)
    i2_x0, i2_y0, i2_z0 = i2_Tb0[0,3], i2_Tb0[1,3], i2_Tb0[2,3]
    i2_x1, i2_y1, i2_z1 = i2_Tb1[0,3], i2_Tb1[1,3], i2_Tb1[2,3]
    i2_x2, i2_y2, i2_z2 = i2_Tb2[0,3], i2_Tb2[1,3], i2_Tb2[2,3]
    i2_x3, i2_y3, i2_z3 = i2_Tb3[0,3], i2_Tb3[1,3], i2_Tb3[2,3]

    i3_Tb0 = legtf.create_Tb0(Tm, -1, 1)
    i3_t01 = legtf.create_T01(leg3.theta1)
    i3_t12 = legtf.create_T12(leg3.theta2 )
    i3_t23 = legtf.create_T23(leg3.theta3 )
    i3_Tb1 = np.matmul(i3_Tb0, i3_t01)
    i3_Tb2 = np.matmul(i3_Tb1, i3_t12)
    i3_Tb3 = np.matmul(i3_Tb2, i3_t23)
    i3_x0, i3_y0, i3_z0 = i3_Tb0[0,3], i3_Tb0[1,3], i3_Tb0[2,3]
    i3_x1, i3_y1, i3_z1 = i3_Tb1[0,3], i3_Tb1[1,3], i3_Tb1[2,3]
    i3_x2, i3_y2, i3_z2 = i3_Tb2[0,3], i3_Tb2[1,3], i3_Tb2[2,3]
    i3_x3, i3_y3, i3_z3 = i3_Tb3[0,3], i3_Tb3[1,3], i3_Tb3[2,3]

    i4_Tb0 = legtf.create_Tb0(Tm, -1, -1)
    i4_t01 = legtf.create_T01(leg4.theta1)
    i4_t12 = legtf.create_T12(leg4.theta2 )
    i4_t23 = legtf.create_T23(leg4.theta3 )
    i4_Tb1 = np.matmul(i4_Tb0, i4_t01)
    i4_Tb2 = np.matmul(i4_Tb1, i4_t12)
    i4_Tb3 = np.matmul(i4_Tb2, i4_t23)
    i4_x0, i4_y0, i4_z0 = i4_Tb0[0,3], i4_Tb0[1,3], i4_Tb0[2,3]
    i4_x1, i4_y1, i4_z1 = i4_Tb1[0,3], i4_Tb1[1,3], i4_Tb1[2,3]
    i4_x2, i4_y2, i4_z2 = i4_Tb2[0,3], i4_Tb2[1,3], i4_Tb2[2,3]
    i4_x3, i4_y3, i4_z3 = i4_Tb3[0,3], i4_Tb3[1,3], i4_Tb3[2,3]

    # Create an empty numpy array to hold scatter points
    points_arr1 = np.empty((0,3))
    points_arr2 = np.empty((0,3))
    points_arr3 = np.empty((0,3))
    points_arr4 = np.empty((0,3))
    points_arr5 = np.empty((0,3))
    points_arr6 = np.empty((0,3))
    points_arr7 = np.empty((0,3))
    points_arr8 = np.empty((0,3))

    # Add scatter points to the array
    points_arr1 = np.append(points_arr1, [[0,0,0], [f_x0, f_y0, f_z0], [f_x1, f_y1, f_z1]], axis=0)
    points_arr1 = np.append(points_arr1, [[f_x2, f_y2, f_z2], [f_x3, f_y3, f_z3]], axis=0)

    points_arr2 = np.append(points_arr2, [[0,0,0], [i_x0, i_y0, i_z0], [i_x1, i_y1, i_z1]], axis=0)
    points_arr2 = np.append(points_arr2, [[i_x2, i_y2, i_z2], [i_x3, i_y3, i_z3]], axis=0)

    points_arr3 = np.append(points_arr3, [[0,0,0], [f2_x0, f2_y0, f2_z0], [f2_x1, f2_y1, f2_z1]], axis=0)
    points_arr3 = np.append(points_arr3, [[f2_x2, f2_y2, f2_z2], [f2_x3, f2_y3, f2_z3]], axis=0)

    points_arr4 = np.append(points_arr4, [[0,0,0], [i2_x0, i2_y0, i2_z0], [i2_x1, i2_y1, i2_z1]], axis=0)
    points_arr4 = np.append(points_arr4, [[i2_x2, i2_y2, i2_z2], [i2_x3, i2_y3, i2_z3]], axis=0)

    points_arr5 = np.append(points_arr5, [[0,0,0], [f3_x0, f3_y0, f3_z0], [f3_x1, f3_y1, f3_z1]], axis=0)
    points_arr5 = np.append(points_arr5, [[f3_x2, f3_y2, f3_z2], [f3_x3, f3_y3, f3_z3]], axis=0)

    points_arr6 = np.append(points_arr6, [[0,0,0], [i3_x0, i3_y0, i3_z0], [i3_x1, i3_y1, i3_z1]], axis=0)
    points_arr6 = np.append(points_arr6, [[i3_x2, i3_y2, i3_z2], [i3_x3, i3_y3, i3_z3]], axis=0)

    points_arr7 = np.append(points_arr7, [[0,0,0], [f4_x0, f4_y0, f4_z0], [f4_x1, f4_y1, f4_z1]], axis=0)
    points_arr7 = np.append(points_arr7, [[f4_x2, f4_y2, f4_z2], [f4_x3, f4_y3, f4_z3]], axis=0)

    points_arr8 = np.append(points_arr8, [[0,0,0], [i4_x0, i4_y0, i4_z0], [i4_x1, i4_y1, i4_z1]], axis=0)
    points_arr8 = np.append(points_arr8, [[i4_x2, i4_y2, i4_z2], [i4_x3, i4_y3, i4_z3]], axis=0)

    plot_points(points_arr1, points_arr2, points_arr3, points_arr4, points_arr5, points_arr6, points_arr7, points_arr8)

if __name__ == '__main__':
    main()