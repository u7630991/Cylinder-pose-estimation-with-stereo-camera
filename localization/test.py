import utils
import operation
import matplotlib.pyplot as plt
import numpy as np

'''
# Generate edge points of a rectangle
edge_points = utils.gen_rectangle_edge_points(4, 3)
#print(edge_points)
plt.scatter(edge_points[:,0], edge_points[:,1])

edge_points2 = operation.transform_2d(edge_points, 3, 4, 90)
plt.scatter(edge_points2[:,0], edge_points2[:,1])
plt.show()

# Generate surface points of the walls of a cuboid
wall_points = utils.gen_cuboid_wall_points(3, 2)
# print(wall_points)
ax = plt.axes(projection='3d')
ax.scatter(wall_points[:,0],wall_points[:,1],wall_points[:,2])
plt.show()


# Generate surface points of the circumference points of a cylinder
circumference_points = utils.gen_cylinder_circumference_points(30,9)
print(circumference_points.shape)
ax = plt.axes(projection='3d')
ax.scatter(circumference_points[:,0],circumference_points[:,1],circumference_points[:,2])
plt.show()


# Generate a rectangle and its rotated variants
rect = utils.gen_rectangle_edge_points(3, 4)
rect_rotated1 = operation.transform_2d(rect, 1, 2, np.pi/8)
rect_rotated2 = operation.transform_2d(rect, 0, 2, np.pi/8)
score1 = utils.compare_rectangles(rect_rotated1, rect)
score2 = utils.compare_rectangles(rect_rotated2, rect)
print('score1:', score1)
print('score2:', score2)
'''

cyli = utils.gen_cylinder_circumference_points(25,9)
cyli_rotated1 = operation.transform_3d(cyli, 10, 1, 1, np.pi/3, np.pi/7, np.pi/5)
cyli_rotated2 = operation.transform_3d(cyli, 15, 1, 5, np.pi/3, np.pi/7, np.pi/5)
result1 = utils.compare_cylinders(cyli_rotated1, cyli)
result2 = utils.compare_cylinders(cyli_rotated2, cyli)
print('result1:', result1)
print('result2:', result2)
ax = plt.axes(projection='3d')
ax.scatter(cyli[:,0],cyli[:,1],cyli[:,2])
ax.scatter(cyli_rotated1[:,0],cyli_rotated1[:,1],cyli_rotated1[:,2])
ax.scatter(cyli_rotated2[:,0],cyli_rotated2[:,1],cyli_rotated2[:,2])
#plt.show()
print(cyli.size)
