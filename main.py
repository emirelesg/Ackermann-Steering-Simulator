#!/usr/bin/python3

"""
    Ackermann Steering Simulator with Tkinter and Pyplot

    Enrique Mireles 
    18.02.2019
"""

from time import time
import tkinter as tk
import numpy as np
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.patches import Ellipse, Circle, Polygon, Rectangle
from matplotlib.figure import Figure
from matplotlib import transforms
import matplotlib.image as image
import matplotlib  

class Plot():
    """
        Wrapper for a pyplot figure embbedded in a Tkinter window.
    """

    def __init__(self, root, row=0, column=0, title="", xlabel="", ylabel="", xlim=(0,100), ylim=(0,100), scroll_gap=0):
        self.figure = Figure(figsize=(5, 4), dpi=100)
        self.ax = self.figure.add_subplot(1, 1, 1)
        self.ax.grid(True)
        self.ax.set_title(title)
        self.ax.set_xlabel(xlabel)
        self.ax.set_ylabel(ylabel)
        self.scroll_gap = scroll_gap
        self.default_xlim = xlim
        self.default_ylim = ylim
        self.ax.set_xlim(self.default_xlim)
        self.ax.set_ylim(self.default_ylim)
        self.x = [0]
        self.y = [0]
        self.line, = self.ax.plot(self.x, self.y, 'r-')
        self.canvas = FigureCanvasTkAgg(self.figure, root)
        self.canvas.get_tk_widget().grid(row=row, column=column)
        self.bg = self.figure.canvas.copy_from_bbox(self.ax.bbox)

    def addPoint(self, x, y):
        """
            Given a pair x, y, adds the point to the plot.
            The plot has a limit of 300 points.
        """
        if (len(self.x) > 300):
            self.y.pop(0)
            self.x.pop(0)
        self.x.append(x)
        self.y.append(y)
        self.update()
    
    def clear(self):
        """
            Resets scale and clears plot.
        """

        self.ax.set_ylim(self.default_ylim)
        self.ax.set_xlim(self.default_xlim)
        self.x = [0]
        self.y = [0] 
        self.update()

    def changeScale(self, k):
        """
            Given a factor k updates the scale of the plot. 
        """

        r = np.ceil(20 / k)
        self.default_xlim = (-r, r)
        self.default_ylim = (-r, r)

    def update(self):
        """
            Updates the x and y data of the plot. If a scroll gap is
            specified, then the plot starts to scroll before it reaches this gap.
        """

        # Update the data of the plot to match that of the arrays.
        self.line.set_xdata(self.x)
        self.line.set_ydata(self.y)
        
        # Scroll plot in increasing X and Y values. 
        # Todo: Negative scroll values.
        lastx = self.x[-1]
        lasty = self.y[-1]
        if (lastx + self.scroll_gap > self.default_xlim[1]):
            self.ax.set_xlim((lastx - (self.default_xlim[1] - self.default_xlim[0] - self.scroll_gap), lastx+self.scroll_gap))
        if (lasty + self.scroll_gap > self.default_ylim[1]):
            self.ax.set_ylim((lasty - (self.default_ylim[1] - self.default_ylim[0] - self.scroll_gap), lasty+self.scroll_gap))
        
        # Redraw plot (slow)
        self.figure.canvas.draw()
        self.figure.canvas.flush_events()

class App:
    """
        This class creates the main window for the simulation.
        Handles all gui elements and plots.

    """ 
    WINDOW_TITLE = "Ackermann Steering Model Simulator - Enrique Mireles"   # Defines the window title
    PADY = 10               # Y padding for all elements in the GUI.
    PADY_BOTTOM = (0, 30)   # Bottom padding for elements in the GUI.
    PADX = 10               # X padding for all elements in the GUI.
    SIM_DT = 0.1            # Dt for the simulation in seconds.
    LOOP_DT = 100           # Dt for the tkinter window in milliseconds.
    INITIAL_ZOOM = 3        # initial zoom level when a simulation is started.
    V0 = 3                  # Starting velocity of the vehicle.
    LF = 1.5                # Front distance from the center of mass of the car.
    LB = 1.5                # Back distance from the center of mass of the car.
    LW = 1                  # Half of the width of the vehicle.

    def __init__(self, root):

        # Configure root window.
        self.root = root
        self.root.wm_title(self.WINDOW_TITLE)

        # Bind function to watch for keyPressed in the window.
        self.root.bind("<Key>", self.keyPressed)

        # Create top, bottom and control frames for separating the elements.
        self.top = tk.Frame()
        self.bottom = tk.Frame()
        self.controls = tk.Frame()
        self.top.grid(in_=self.root, row=0, column=0, columnspan=3, sticky=tk.NSEW, pady=self.PADY, padx=self.PADX)
        self.controls.grid(in_=self.root, row=1, column=0, sticky=tk.NSEW, pady=self.PADY, padx=self.PADX)
        self.bottom.grid(in_=self.root, row=2, column=0, columnspan=3, sticky=tk.NSEW, pady=self.PADY, padx=self.PADX)
        
        # Define controls for the top frame.
        self.button_close = tk.Button(self.root, text ="X", command=self.root.destroy)
        # self.button_load = tk.Button(self.root, text ="Load", command=self.load)
        self.button_play = tk.Button(self.root, text ="Play", command=self.start)
        self.button_pause = tk.Button(self.root, text ="Pause", command=self.pause)
        # self.button_load.pack(in_=self.top, padx=5, side=tk.LEFT)
        self.button_play.pack(in_=self.top, padx=5, side=tk.LEFT)
        self.button_pause.pack(in_=self.top, padx=5, side=tk.LEFT)
        self.button_close.pack(in_=self.top, padx=5, side=tk.RIGHT)

        # Define controls for the control frame.
        self.ctrl_angle = tk.Scale(self.root, from_=-45, to=45, length=200, orient=tk.HORIZONTAL)
        self.label_angle = tk.Label(self.root, text="Steering Angle [°]")
        self.ctrl_vel = tk.Scale(self.root, from_=0, to=5, length=200, orient=tk.HORIZONTAL)
        self.label_vel = tk.Label(self.root, text="Velocity [m/s]")
        self.ctrl_zoom = tk.Scale(self.root, from_=1, to=4, length=200, orient=tk.HORIZONTAL, command=self.scaleChanged)
        self.label_zoom = tk.Label(self.root, text="Zoom Level")
        self.label_angle.grid(in_=self.controls, row=0, column=0, sticky=tk.W)
        self.ctrl_angle.grid(in_=self.controls, row=1, column=0, pady=self.PADY_BOTTOM)
        self.label_vel.grid(in_=self.controls, row=2, column=0, sticky=tk.W)
        self.ctrl_vel.grid(in_=self.controls, row=3, column=0, pady=self.PADY_BOTTOM)
        self.label_zoom.grid(in_=self.controls, row=4, column=0, sticky=tk.W)
        self.ctrl_zoom.grid(in_=self.controls, row=5, column=0)

        # Define controls for the bottom frame.
        self.var_status = tk.StringVar()
        self.var_time = tk.StringVar()
        label_name = tk.Label(self.root, text="Advanced Robotics | Enrique Mireles Gutierrez | 18.02.2019 ")
        label_time = tk.Label(self.root, textvariable=self.var_time)
        label_status = tk.Label(self.root, textvariable=self.var_status)
        label_status.pack(in_=self.bottom, side=tk.RIGHT, padx=self.PADX)
        label_time.pack(in_=self.bottom, side=tk.RIGHT, padx=self.PADX)
        label_name.pack(in_=self.bottom, side=tk.LEFT)

        # Define plots
        self.scale = self.INITIAL_ZOOM      # Scale of plots
        self.t = 0                          # Ellapsed time
        self.prev_phi = 0                   # Previous angle of the vehicle
        self.running = False                # is the simulation currently running?
        self.car = Plot(self.root,          # Create a plot for drawing the vehicle's path.
            row=1, 
            column=1,
            title="Vehicle Motion [Ackermann Steering]",
            xlabel="Distance [m]",
            ylabel="Distance [m]",
            xlim=(-5, 5),
            ylim=(-5, 5),
            scroll_gap=5
        )
        self.steering = Plot(self.root,     # Create a plot for drawing the steering angle.
            row=1, 
            column=2,
            title="Steering Angle",
            xlabel="Time [s]",
            ylabel="Angle [°]",
            xlim=(0, 10),
            ylim=(-50, 50)
        )

        # Aditional Graphics
        # self.car_box = Rectangle((0, 0), self.LB + self.LF, self.LW, color='b', zorder=2, alpha=0.5)
        # self.car.ax.add_patch(self.car_box)
        # self.mass_center = Circle((0, 0), 0.2, color='b', zorder=2, alpha=0.5)
        # self.car.ax.add_patch(self.mass_center)
        # self.car_img = self.car.ax.imshow(image.imread('police-car.png'), aspect='auto', extent=(-self.LB, self.LF, -self.LW, self.LW), zorder=2)

        # Set the default state of the application.
        self.resetSimulation()
        self.pause()

    def load(self):
        pass

    def keyPressed(self, event):
        """
            This function is called everytime a key is pressed.
        """
        
        # Retrieve key pressed in ASCII
        pressed = event.char
        if (pressed == 'a'):

            # Increase steering angle.
            self.ctrl_angle.set(self.ctrl_angle.get() + 5)

        elif (pressed == 'd'):
            
            # Decrease steering angle.
            self.ctrl_angle.set(self.ctrl_angle.get() - 5)

        elif (pressed == 'r'):
            
            # Restart simulation.
            self.pause()
            self.resetSimulation()
            self.start()

        elif (pressed == 's'):
            
            # Start or stop the simulation.
            if (self.running):
                self.pause()
            else:
                self.start()

        elif (pressed == 'q'):
            
            # Quit simulation.
            self.root.quit()

    def start(self):
        """
            Prepare the simulation and start calculating the trajectory.
        """

        self.var_status.set("Running")
        self.button_pause['state'] = "normal"
        self.button_play['state'] = "disabled"
        self.running = True
        self.resetSimulation()
        self.runSimulation()

    def pause(self):
        """
            Stop the simulation and stay in the same state.
        """

        self.var_time.set("T: 0s")
        self.var_status.set("Paused")
        self.button_pause['state'] = "disabled"
        self.button_play['state'] = "normal"
        self.running = False

    def resetSimulation(self):
        """
            Clear all plots and reset state variables.
        """

        # Reset control values.
        self.ctrl_zoom.set(self.INITIAL_ZOOM)
        self.ctrl_angle.set(0)
        self.ctrl_vel.set(self.V0)

        # Clear plots and reset scale.
        self.car.changeScale(self.INITIAL_ZOOM)
        self.car.clear()
        self.steering.clear()
        self.t = 0
        self.prev_phi = 0
    
    def scaleChanged(self, val):
        """
            Function called when the zoom control is changed.
            Updates the scale of the car plot.
        """

        self.car.changeScale(int(val))

    def runSimulation(self):
        """
            Main loop for calculating the path of the car.
        """

        # Record starting time of this loop cycle.
        start_time = time()
        
        # Get values from sliders.
        vel = self.ctrl_vel.get()
        steering = np.deg2rad(self.ctrl_angle.get())


        # Calculate heading of the vehicle.
        beta = np.arctan(self.LB * np.tan(steering) / (self.LB + self.LF))
        phi = self.prev_phi + vel * self.SIM_DT * np.cos(beta) * np.tan(steering) / (self.LB + self.LF)
        self.prev_phi = phi

        # Calculate vehicles's position.
        x = self.car.x[-1] + vel * self.SIM_DT * np.cos(beta + phi)
        y = self.car.y[-1] + vel * self.SIM_DT * np.sin(beta + phi)

        # Plot new position.
        self.car.addPoint(x, y)

        # Plot steering angle.
        self.steering.addPoint(self.t, self.ctrl_angle.get())

        # Update time label.
        self.var_time.set("T: %.1fs" % self.t)

        # tr = transforms.Affine2D().translate(x, y).rotate(phi)
        # self.car_img.set_extent((x-self.LB, x+self.LF, y-self.LW, y+self.LW))
        # self.car_img.set_transform(tr + self.car.ax.transData)
        # self.car_box.xy = (
        #     x - (self.LF + self.LB) / 2,
        #     y - self.LW / 2
        # )
        # l_wheels = np.sqrt(self.LF ** 2 + 1 ** 1)
        # theta_wheels = np.arctan2(1, self.LF)
        # self.mass_center.center = (
        #     x + l_wheels * np.cos(phi + theta_wheels),
        #     y + l_wheels * np.sin(phi + theta_wheels)
        # )

        # Increase time to prepare for next cycle.
        self.t += self.SIM_DT
        
        # Calculate elapsed time.
        dt = (time()-start_time) * 1000
        print("Cycle Time: %.1fms" % (dt)) 
        
        # If the user has not pressed the pause button, continue.
        if (self.running):

            # Compensate for the delay in the loop. The cycle must run at LOOP_DT intervals.
            self.root.after(int(self.LOOP_DT-dt), self.runSimulation)

def main():
    """
        Create main window and start application.
    """

    matplotlib.use("TkAgg")  
    root = tk.Tk()
    app = App(root)
    root.mainloop()

if __name__ == "__main__":
    main()