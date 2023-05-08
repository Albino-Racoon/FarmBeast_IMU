
import rospy
from sensor_msgs.msg import Imu
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from tf.transformations import euler_from_quaternion
import numpy as np
import matplotlib.animation as animation

x = []
y = []
z = []
last_gyro = np.array([0.0, 0.0, 0.0])
last_time = None

def callback(data):
    global x, y, z, last_gyro, last_time
    gyro_x = data.angular_velocity.x
    gyro_y = data.angular_velocity.y
    gyro_z = data.angular_velocity.z

    # Calculate time difference between current and last measurement
    current_time = rospy.Time.now()
    if last_time is None:
        time_diff = rospy.Duration.from_sec(0.0)
    else:
        time_diff = current_time - last_time

    # Calculate displacement in x, y, and z directions
    displacement_x = (gyro_x + last_gyro[0]) / 2 * time_diff.to_sec()
    displacement_y = (gyro_y + last_gyro[1]) / 2 * time_diff.to_sec()
    displacement_z = (gyro_z + last_gyro[2]) / 2 * time_diff.to_sec()

    # Update last gyro and time
    last_gyro = np.array([gyro_x, gyro_y, gyro_z])
    last_time = current_time

    # Check if the robot has moved
    if abs(displacement_x) < 0.0001 and abs(displacement_y) < 0.0001 and abs(displacement_z) < 0.0001:
        return

    # Update x, y, and z positions
    x.append(x[-1] + displacement_x)
    y.append(y[-1] + displacement_y)
    z.append(z[-1] + displacement_z)

def update_plot(num):
    global x, y, z
    x = x[-1000:]  # truncate to the same length
    y = y[-1000:]
    z = z[-1000:]
    ax.cla()
    ax.plot(x, y, z)
    ax.set_xlabel('X position')
    ax.set_ylabel('Y position')
    ax.set_zlabel('Z position')
    ax.set_title('IMU Pot')
    plt.draw()


if __name__ == '__main__':
    rospy.init_node("imu_path_node")

    # Subscribe to the Imu topic
    rospy.Subscriber("/imu/data", Imu, callback)

    # Initialize the plot
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    x.append(0)
    y.append(0)
    z.append(0)
    ax.plot(x, y, z)
    ax.set_xlabel('X position')
    ax.set_ylabel('Y position')
    ax.set_zlabel('Z position')
    ax.set_title('IMU path')

    # Animate the plot
    ani = animation.FuncAnimation(fig, update_plot, interval=10)

    plt.show()

    rospy.spin()

