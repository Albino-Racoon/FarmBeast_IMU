# !/usr/bin/env python

import math

import pygame
import rospy
from OpenGL.GL import *
from OpenGL.GLU import *
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion

# Začetek pyGame-a sam sm glup pa ga neznam normalno vzpstavit notr u isti GUI
pygame.init()
pygame.display.set_caption("IMU Data Položaj robota z wireframeom")
size = width, height = 800, 600
screen = pygame.display.set_mode(size, pygame.OPENGL | pygame.DOUBLEBUF)

# Baje da OpenGL nuca to ka ti js vem kodo sm ukradu
glClearColor(0.0, 0.0, 0.0, 1.0)
glMatrixMode(GL_PROJECTION)
gluPerspective(45, width / height, 0.1, 100.0)
glMatrixMode(GL_MODELVIEW)
glLoadIdentity()
glTranslatef(0.0, 0.0, -5.0)

# Globalne spremenljiuke za roll pitch pa yaw
roll = 0.0
pitch = 0.0
yaw = 0.0


def callback(data):
    global roll, pitch, yaw
    orientation_q = data.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
    (roll, pitch, yaw) = euler_from_quaternion(orientation_list)


def draw_cube():
    glBegin(GL_LINES)
    # Front face
    glVertex3f(-1.0, -1.0, 1.0)
    glVertex3f(1.0, -1.0, 1.0)

    glVertex3f(1.0, -1.0, 1.0)
    glVertex3f(1.0, 1.0, 1.0)

    glVertex3f(1.0, 1.0, 1.0)
    glVertex3f(-1.0, 1.0, 1.0)

    glVertex3f(-1.0, 1.0, 1.0)
    glVertex3f(-1.0, -1.0, 1.0)

    # Back face
    glVertex3f(-1.0, -1.0, -1.0)
    glVertex3f(1.0, -1.0, -1.0)

    glVertex3f(1.0, -1.0, -1.0)
    glVertex3f(1.0, 1.0, -1.0)

    glVertex3f(1.0, 1.0, -1.0)
    glVertex3f(-1.0, 1.0, -1.0)

    glVertex3f(-1.0, 1.0, -1.0)
    glVertex3f(-1.0, -1.0, -1.0)

    # Connect front and back faces
    glVertex3f(-1.0, -1.0, 1.0)
    glVertex3f(-1.0, -1.0, -1.0)

    glVertex3f(1.0, -1.0, 1.0)
    glVertex3f(1.0, -1.0, -1.0)

    glVertex3f(1.0, 1.0, 1.0)
    glVertex3f(1.0, 1.0, -1.0)

    glVertex3f(-1.0, 1.0, 1.0)
    glVertex3f(-1.0, 1.0, -1.0)

    glEnd()

    glBegin(GL_LINE_LOOP)
    # Top face
    glVertex3f(-1.0, 1.0, 1.0)
    glVertex3f(1.0, 1.0, 1.0)
    glVertex3f(1.0, 1.0, -1.0)
    glVertex3f(-1.0, 1.0, -1.0)
    glEnd()

    glBegin(GL_LINE_LOOP)
    # Bottom face
    glVertex3f(-1.0, -1.0, 1.0)
    glVertex3f(1.0, -1.0, 1.0)
    glVertex3f(1.0, -1.0, -1.0)
    glVertex3f(-1.0, -1.0, -1.0)
    glEnd()


rospy.init_node('imu_viewer')
rospy.Subscriber('/imu/data', Imu, callback)

while not rospy.is_shutdown():
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            pygame.quit()
            quit()

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)

    glLoadIdentity()
    glTranslatef(0.0, 0.0, -5.0)
    glRotatef(math.degrees(pitch), 1.0, 0.0, 0.0)
    glRotatef(math.degrees(yaw), 0.0, 1.0, 0.0)
    glRotatef(math.degrees(roll), 0.0, 0.0, 1.0)

    draw_cube()

    pygame.display.flip()
"""""""""""
import sys
from PyQt5 import QtWidgets
from OpenGL.GL import *
from OpenGL.GLU import *
import pygame
import math

# Your code for initializing Pygame and OpenGL, defining the callback function, and drawing the cube goes here

class MainWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()

        # Set up the IMU Data Viewer
        pygame.init()
        pygame.display.set_caption("IMU Data Viewer")
        size = width, height = 800, 600
        self.gl_widget = QtWidgets.QOpenGLWidget()
        self.gl_widget.setFixedSize(size[0], size[1])
        self.gl_widget.initializeGL()
        self.gl_widget.paintGL = self.paint_gl
        self.gl_widget.resizeGL = self.resize_gl

        # Set up other components in the GUI
        self.label = QtWidgets.QLabel("Hello World!")
        self.button = QtWidgets.QPushButton("Click me")
        self.button.clicked.connect(self.button_clicked)

        # Add the components to a layout
        layout = QtWidgets.QVBoxLayout()
        layout.addWidget(self.gl_widget)
        layout.addWidget(self.label)
        layout.addWidget(self.button)

        # Create a central widget with the layout
        central_widget = QtWidgets.QWidget()
        central_widget.setLayout(layout)
        self.setCentralWidget(central_widget)

    def paint_gl(self):
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        glLoadIdentity()
        glTranslatef(0.0, 0.0, -5.0)
        glRotatef(math.degrees(pitch), 1.0, 0.0, 0.0)
        glRotatef(math.degrees(yaw), 0.0, 1.0, 0.0)
        glRotatef(math.degrees(roll), 0.0, 0.0, 1.0)
        draw_cube()

    def resize_gl(self, width, height):
        glViewport(0, 0, width, height)
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        gluPerspective(45, width / height, 0.1, 100.0)
        glMatrixMode(GL_MODELVIEW)

    def button_clicked(self):
        self.label.setText("Button clicked!")

if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())
"""""""""