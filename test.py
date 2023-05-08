
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


"""""""""
import rospy
from sensor_msgs.msg import Imu
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from tf.transformations import euler_from_quaternion
import numpy as np

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
    if displacement_x == 0 and displacement_y == 0 and displacement_z == 0:
        return

    # Update x, y, and z positions
    x.append(x[-1] + displacement_x)
    y.append(y[-1] + displacement_y)
    z.append(z[-1] + displacement_z)

    # Plot the path
    ax.cla()
    ax.plot(x, y, z)
    ax.set_xlabel('X position')
    ax.set_ylabel('Y position')
    ax.set_zlabel('Z position')
    ax.set_title('IMU path')
    plt.draw()
    plt.pause(0.001)

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
    plt.ion()
    plt.show()

    rospy.spin()

# kompas
import rospy
from sensor_msgs.msg import Imu
import tkinter as tk
from tf.transformations import euler_from_quaternion
import numpy as np


class Compass(tk.Canvas):
    def __init__(self, master=None, **kw):
        super().__init__(master, **kw)
        self._size = min(self.winfo_reqwidth(), self.winfo_reqheight())
        self.configure(width=self._size, height=self._size)
        self._needle = self.create_line(0, 0, 0, 0, width=0.0000000000005, fill="gray")
        self._heading = 0
        self.draw()
        self.delete()

    def set_heading(self, heading):
        self._heading = heading
        self.draw_needle()

    def draw_needle(self):
        if self._needle is not None:
            self.delete(self._needle)
        r = self._size / 2
        angle = np.radians(-self._heading + 90)
        x1 = r
        y1 = r
        x2 = r + r * 0.8 * np.cos(angle)
        y2 = r + r * 0.8 * np.sin(angle)
        self._needle = self.create_line(x1, y1, x2, y2, width=5, fill="red")

    def draw(self):
        # Clear the existing drawing
        self.delete('all')

        # Draw the outer circle
        r = self._size / 2
        self.create_oval(r / 10, r / 10, r * 1.9, r * 1.9, width=2)

        # Draw the cardinal directions
        # directions = [' Sever ', ' Vzhod ', ' Jug ', ' Zahod ']
        directions = [" ", " ", " ", " "]
        for i, direction in enumerate(directions):
            angle = np.radians(i * 90)
            x = r * np.cos(angle) + r
            y = r * np.sin(angle) + r
            self.create_text(x, y, text=direction)

        # Remove the previous needle
        self.delete("needle")

        # Draw the new compass needle
        angle = np.radians(-self._heading + 90)
        x1 = r
        y1 = r
        x2 = r + r * 0.8 * np.cos(angle)
        y2 = r + r * 0.8 * np.sin(angle)
        self.create_line(x1, y1, x2, y2, width=5, tags=("needle",))


if __name__ == '__main__':
    rospy.init_node("compass_node")
    root = tk.Tk()
    root.title("Compass")

    # Create a canvas to draw the compass
    canvas_width = 500
    canvas_height = 500
    canvas = Compass(root, width=canvas_width, height=canvas_height)
    canvas.pack()


    def callback(data):
        orientation_q = data.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)

        # Update the compass heading
        canvas.set_heading(np.degrees(yaw))


    # Subscribe to the Imu topic
    rospy.Subscriber("/imu/data", Imu, callback)

    root.mainloop()
"""""""""
"""""""""
import rospy
from sensor_msgs.msg import Imu
import tkinter as tk
from std_msgs.msg import String
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import matplotlib.pyplot as plt
import numpy as np
import sys
sys.setrecursionlimit(10**6)

#TO RUN (venv) rakun@rakunComp:~/catkin_ws/src/farmbeast_simulation/src$ rosrun farmbeast_simulation test.py

def callback(data):
    global roll, pitch, yaw
    orientation_q = data.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
    roll_label.config(text="Roll: {:.2f}".format(roll))
    pitch_label.config(text="Pitch: {:.2f}".format(pitch))
    yaw_label.config(text="Yaw: {:.2f}".format(yaw))

    # Rotate the needle to point in the direction of the current yaw value
    canvas.delete(needle)  # Remove the existing needle
    canvas.create_rectangle(0, 0, canvas_width, canvas_height, fill='white') # Draw a white rectangle to clear the canvas
    x, y = canvas_width/2, canvas_height/2  # Center of the canvas
    length = min(canvas_width, canvas_height) * 0.4  # Length of the needle
    angle = -np.degrees(yaw)  # Convert yaw to degrees and negate to rotate clockwise
    x2 = x + length * np.sin(np.radians(angle))
    y2 = y + length * np.cos(np.radians(angle))
    canvas.create_line(x, y, x2, y2, fill='red', width=3, arrow='last', tag='needle')


def update_gui():
    global bar_plot
    root.after(100, update_gui) # Update every 100 milliseconds
    root.update()

    # Clear the existing plot
    plt.clf()

    # Create a new bar plot with the current roll, pitch and yaw values
    data = [roll, pitch, yaw]
    labels = ['Roll', 'Pitch', 'Yaw']
    bar_plot = plt.bar(labels, data)

    # Update the plot
    for i, rect in enumerate(bar_plot):
        height = rect.get_height()
        plt.text(rect.get_x() + rect.get_width() / 2.0, height, '{:.2f}'.format(data[i]), ha='center', va='bottom')

    plt.pause(0.001)

class Compass(tk.Canvas):
    def __init__(self, master=None, **kw):
        super().__init__(master, **kw)
        self._size = min(self.winfo_reqwidth(), self.winfo_reqheight())
        self.configure(width=self._size, height=self._size)
        self._heading = 0
        self.draw()

    def set_heading(self, heading):
        self._heading = heading
        self.draw()

    def draw(self):
        # Clear the existing drawing
        self.delete('all')

        # Draw the outer circle
        r = self._size / 2
        self.create_oval(r / 10, r / 10, r * 1.9, r * 1.9, width=2)

        # Draw the cardinal directions
        directions = ['N', 'E', 'S', 'W']
        for i, direction in enumerate(directions):
            angle = np.radians(i * 90)
            x = r * np.cos(angle) + r
            y = r * np.sin(angle) + r
            self.create_text(x, y, text=direction)

        # Draw the compass needle
        angle = np.radians(-self._heading + 90)
        x1 = r
        y1 = r
        x2 = r + r * 0.8 * np.cos(angle)
        y2 = r + r * 0.8 * np.sin(angle)
        self.create_line(x1, y1, x2, y2, width=3)

if __name__ == '__main__':
    rospy.init_node("imu_gui_node1")
    root = tk.Tk()
    root.title("IMU Data")

    # Create labels to display the roll, pitch and yaw values
    roll_label = tk.Label(root, text="Roll: ")
    roll_label.pack()
    pitch_label = tk.Label(root, text="Pitch: ")
    pitch_label.pack()
    yaw_label = tk.Label(root, text="Yaw: ")
    yaw_label.pack()

    # Create a bar plot to display the roll, pitch and yaw values
    data = [0, 0, 0]
    labels = ['Roll', 'Pitch', 'Yaw']
    bar_plot = plt.bar(labels, data)

    # Create a canvas to draw the compass
    canvas_width = 200
    canvas_height = 200
    canvas = tk.Canvas(root, width=canvas_width, height=canvas_height)
    canvas.pack()

    # Define the points for the compass needle
    needle_points = [100, 0, 90, 100, 110, 100]
    needle = canvas.create_line(canvas_width/2, canvas_height/2, canvas_width/2, canvas_height/2-1, fill='red', width=3, arrow='last', tag='needle')


    # Subscribe to the Imu topic
    rospy.Subscriber("/imu/data", Imu, callback)

    update_gui()
    root.mainloop()

"""""""""""""""""""""""""""

""""""""""""""""""""""

import rospy
from sensor_msgs.msg import Imu
import tkinter as tk
from std_msgs.msg import String
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import matplotlib.pyplot as plt

#TO RUN (venv) rakun@rakunComp:~/catkin_ws/src/farmbeast_simulation/src$ rosrun farmbeast_simulation test.py

def callback(data):
    global roll, pitch, yaw
    orientation_q = data.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
    roll_label.config(text="Roll: {:.2f}".format(roll))
    pitch_label.config(text="Pitch: {:.2f}".format(pitch))
    yaw_label.config(text="Yaw: {:.2f}".format(yaw))

def update_gui():
    global bar_plot
    root.after(100, update_gui) # Update every 100 milliseconds
    root.update()

    # Clear the existing plot
    plt.clf()

    # Create a new bar plot with the current roll, pitch and yaw values
    data = [roll, pitch, yaw]
    labels = ['Roll', 'Pitch', 'Yaw']
    bar_plot = plt.bar(labels, data)

    # Update the plot
    for i, rect in enumerate(bar_plot):
        height = rect.get_height()
        plt.text(rect.get_x() + rect.get_width() / 2.0, height, '{:.2f}'.format(data[i]), ha='center', va='bottom')

    plt.pause(0.001)

if __name__ == '__main__':
    rospy.init_node("imu_gui_node")
    root = tk.Tk()
    root.title("IMU Data")

    # Create labels to display the roll, pitch and yaw values
    roll_label = tk.Label(root, text="Roll: ")
    roll_label.pack()
    pitch_label = tk.Label(root, text="Pitch: ")
    pitch_label.pack()
    yaw_label = tk.Label(root, text="Yaw: ")
    yaw_label.pack()

    # Create a bar plot to display the roll, pitch and yaw values
    data = [0, 0, 0]
    labels = ['Roll', 'Pitch', 'Yaw']
    bar_plot = plt.bar(labels, data)

    # Subscribe to the Imu topic
    rospy.Subscriber("/imu/data", Imu, callback)

    update_gui()
    root.mainloop()
    
"""""""""""
