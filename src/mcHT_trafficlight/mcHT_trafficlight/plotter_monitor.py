#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist, Pose2D
import matplotlib.pyplot as plt
from collections import deque
import numpy as np

class LivePlotNode(Node):
    def __init__(self):
        super().__init__('live_plot_node')

        # robot/wheel parameters (must match your odometry node)
        self.r = 0.05    # wheel radius [m]
        self.L = 0.174   # wheel base [m]

        # data buffers
        self.start_time = None
        self.times_v,   self.v_vals   = deque(maxlen=1000), deque(maxlen=1000)
        self.times_w,   self.w_vals   = deque(maxlen=1000), deque(maxlen=1000)
        self.ctrl_t,    self.cmd_v    = deque(maxlen=1000), deque(maxlen=1000)
        self.ctrl_t_w,  self.cmd_w    = deque(maxlen=1000), deque(maxlen=1000)
        self.err_t,     self.ed       = deque(maxlen=1000), deque(maxlen=1000)
        self.err_t_th,  self.etheta   = deque(maxlen=1000), deque(maxlen=1000)
        self.x_data,    self.y_data   = deque(maxlen=1000), deque(maxlen=1000)
        self.goals     = []            # list of (xg, yg)
        self.last_goal = None

        # most recent wheel rates
        self.wl = None
        self.wr = None

        # subscribers
        self.create_subscription(Float32, 'VelocityEncL', self.wl_cb, 10)
        self.create_subscription(Float32, 'VelocityEncR', self.wr_cb, 10)
        self.create_subscription(Twist,   'cmd_vel',      self.cmd_vel_cb, 10)
        self.create_subscription(Pose2D,  'pose',         self.pose_cb,    10)
        self.create_subscription(Pose2D,  'goal',         self.goal_cb,    10)

        # single figure with 3×2 grid
        plt.ion()
        self.fig, axes = plt.subplots(3, 2, figsize=(12, 10))
        (self.ax1, self.ax2), (self.ax3, self.ax4), (self.ax5, self.ax6) = axes
        self.ax6.axis('off')  # unused subplot

        # 1) Angular velocity ω(t)
        self.line_w, = self.ax1.plot([], [], label='ω (rad/s)')
        self.ax1.set_title('Angular velocity ω(t)')
        self.ax1.set_xlabel('Time [s]'); self.ax1.set_ylabel('ω [rad/s]')

        # 2) Linear velocity v(t)
        self.line_v, = self.ax2.plot([], [], label='v (m/s)')
        self.ax2.set_title('Linear velocity v(t)')
        self.ax2.set_xlabel('Time [s]'); self.ax2.set_ylabel('v [m/s]')

        # 3) XY trajectory + goals
        self.line_xy, = self.ax3.plot([], [], '-o', markersize=2, label='Path')
        self.goal_scatter = self.ax3.scatter([], [], c='r', marker='x', label='Goals')
        self.ax3.set_title('Robot position (x,y)')
        self.ax3.set_xlabel('x [m]'); self.ax3.set_ylabel('y [m]')
        self.ax3.axis('equal'); self.ax3.legend()

        # 4) Control inputs
        self.line_cv1, = self.ax4.plot([], [], label='cmd_vel.linear.x')
        self.line_cv2, = self.ax4.plot([], [], label='cmd_vel.angular.z')
        self.ax4.set_title('Control inputs')
        self.ax4.set_xlabel('Time [s]'); self.ax4.set_ylabel('Cmd')
        self.ax4.legend()

        # 5) Errors ed(t), eθ(t)
        self.line_e1, = self.ax5.plot([], [], label='ed (m)')
        self.line_e2, = self.ax5.plot([], [], label='eθ (rad)')
        self.ax5.set_title('Errors')
        self.ax5.set_xlabel('Time [s]'); self.ax5.legend()

        # timer to refresh plots
        self.create_timer(0.1, self.update_plots)
        self.get_logger().info("LivePlotNode initialized; all plots in one window.")

    def _stamp(self):
        now = self.get_clock().now()
        if self.start_time is None:
            self.start_time = now
            return 0.0
        return (now - self.start_time).nanoseconds * 1e-9

    def wl_cb(self, msg: Float32):
        self.wl = msg.data
        self._process_velocity()

    def wr_cb(self, msg: Float32):
        self.wr = msg.data
        self._process_velocity()

    def _process_velocity(self):
        if self.wl is None or self.wr is None:
            return
        v = self.r * (self.wr + self.wl) / 2.0
        w = self.r * (self.wr - self.wl) / self.L
        t = self._stamp()
        self.times_v.append(t);   self.v_vals.append(v)
        self.times_w.append(t);   self.w_vals.append(w)

    def cmd_vel_cb(self, msg: Twist):
        t = self._stamp()
        self.ctrl_t.append(t);  self.cmd_v.append(msg.linear.x)
        self.ctrl_t_w.append(t);self.cmd_w.append(msg.angular.z)

    def pose_cb(self, msg: Pose2D):
        self.x_data.append(msg.x); self.y_data.append(msg.y)
        if self.last_goal is not None:
            ed, etheta = self.compute_errors(
                msg.x, msg.y, self.last_goal[0], self.last_goal[1], msg.theta
            )
            t = self._stamp()
            self.err_t.append(t);   self.ed.append(ed)
            self.err_t_th.append(t);self.etheta.append(etheta)

    def goal_cb(self, msg: Pose2D):
        self.last_goal = (msg.x, msg.y)
        self.goals.append(self.last_goal)

    @staticmethod
    def compute_errors(xr, yr, xg, yg, theta_r):
        ed = np.hypot(xg - xr, yg - yr)
        thetag = np.arctan2(yg - yr, xg - xr)
        etheta = np.arctan2(np.sin(thetag - theta_r), np.cos(thetag - theta_r))
        return ed, etheta

    def update_plots(self):
        # ω(t)
        self.line_w.set_data(self.times_w, self.w_vals)
        if self.times_w:
            self.ax1.set_xlim(self.times_w[0], self.times_w[-1])
            self.ax1.set_ylim(min(self.w_vals)-0.1, max(self.w_vals)+0.1)

        # v(t)
        self.line_v.set_data(self.times_v, self.v_vals)
        if self.times_v:
            self.ax2.set_xlim(self.times_v[0], self.times_v[-1])
            self.ax2.set_ylim(min(self.v_vals)-0.1, max(self.v_vals)+0.1)

        # XY + goals
        self.line_xy.set_data(self.x_data, self.y_data)
        if self.goals:
            gx, gy = zip(*self.goals)
            self.goal_scatter.set_offsets(np.c_[gx, gy])
            all_x = list(self.x_data)+list(gx)
            all_y = list(self.y_data)+list(gy)
            self.ax3.set_xlim(min(all_x)-0.1, max(all_x)+0.1)
            self.ax3.set_ylim(min(all_y)-0.1, max(all_y)+0.1)

        # control inputs
        self.line_cv1.set_data(self.ctrl_t,    self.cmd_v)
        self.line_cv2.set_data(self.ctrl_t_w,  self.cmd_w)
        if self.ctrl_t:
            self.ax4.set_xlim(self.ctrl_t[0], self.ctrl_t[-1])
            combined = list(self.cmd_v)+list(self.cmd_w)
            self.ax4.set_ylim(min(combined)-0.1, max(combined)+0.1)

        # errors
        self.line_e1.set_data(self.err_t,    self.ed)
        self.line_e2.set_data(self.err_t_th, self.etheta)
        if self.err_t:
            self.ax5.set_xlim(self.err_t[0], self.err_t[-1])
            combined_e = list(self.ed)+list(self.etheta)
            self.ax5.set_ylim(min(combined_e)-0.1, max(combined_e)+0.1)

        # single‐canvas draw
        self.fig.tight_layout()
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

def main(args=None):
    rclpy.init(args=args)
    node = LivePlotNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()