from stl import mesh
from mpl_toolkits import mplot3d
from matplotlib import pyplot

figure = pyplot.figure()
axes = mplot3d.Axes3D(figure)

imu_mesh = mesh.Mesh.from_file('IMU.stl')
axes.add_collection3d(mplot3d.art3d.Poly3DCollection(imu_mesh.vectors))

scale = imu_mesh.points.flatten(-1)
axes.auto_scale_xyz(scale, scale, scale)
print(imu_mesh.vectors)

pyplot.show()