import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose2D
import matplotlib.pyplot as plt
from collections import deque
import numpy as np

class LivePlotNode(Node):
    def __init__(self):
        super().__init__('live_plot_node')

        # data buffers
        self.start_time = None
        self.times_v,   self.v_vals    = deque(maxlen=1000), deque(maxlen=1000)
        self.times_w,   self.w_vals    = deque(maxlen=1000), deque(maxlen=1000)
        self.ctrl_t,    self.cmd_v     = deque(maxlen=1000), deque(maxlen=1000)
        self.ctrl_t_w,  self.cmd_w     = deque(maxlen=1000), deque(maxlen=1000)
        self.err_t,     self.ed        = deque(maxlen=1000), deque(maxlen=1000)
        self.err_t_th,  self.etheta    = deque(maxlen=1000), deque(maxlen=1000)
        self.x_data,    self.y_data    = deque(maxlen=1000), deque(maxlen=1000)
        self.goals                         = []  # list of (xg, yg)
        self.last_goal                     = None

        # buffers for “remaining to goal”
        self.rem_t,    self.rem_dist     = deque(maxlen=1000), deque(maxlen=1000)
        self.rem_t_th, self.rem_angle    = deque(maxlen=1000), deque(maxlen=1000)

        # for pose‐based velocity
        self.prev_x = None
        self.prev_y = None
        self.prev_theta = None
        self.prev_time_pose = None

        # subscriptions
        self.create_subscription(Pose2D, 'pose',    self.pose_cb,    10)
        self.create_subscription(Pose2D, 'goal',    self.goal_cb,    10)
        self.create_subscription(Twist,  'cmd_vel', self.cmd_vel_cb, 10)

        # set up single 3×2 figure
        plt.ion()
        self.fig, axes = plt.subplots(3, 2, figsize=(12, 10))
        (self.ax1, self.ax2), (self.ax3, self.ax4), (self.ax5, self.ax6) = axes

        # 1) ω(t)
        self.line_w, = self.ax1.plot([], [], label='ω (rad/s)')
        self.ax1.set_title('Angular vel ω(t)')
        self.ax1.set_xlabel('Time [s]'); self.ax1.set_ylabel('ω [rad/s]')

        # 2) v(t)
        self.line_v, = self.ax2.plot([], [], label='v (m/s)')
        self.ax2.set_title('Linear vel v(t)')
        self.ax2.set_xlabel('Time [s]'); self.ax2.set_ylabel('v [m/s]')

        # 3) XY + goals
        self.line_xy,     = self.ax3.plot([], [], '-o', markersize=2, label='Path')
        self.goal_scatter = self.ax3.scatter([], [], c='r', marker='x', label='Goals')
        self.ax3.set_title('Robot XY trajectory')
        self.ax3.set_xlabel('x [m]'); self.ax3.set_ylabel('y [m]')
        self.ax3.axis('equal'); self.ax3.legend()

        # 4) control inputs
        self.line_cv1, = self.ax4.plot([], [], label='cmd_vel.lin.x')
        self.line_cv2, = self.ax4.plot([], [], label='cmd_vel.ang.z')
        self.ax4.set_title('Control inputs')
        self.ax4.set_xlabel('Time [s]'); self.ax4.set_ylabel('cmd')
        self.ax4.legend()

        # 5) errors
        self.line_e1, = self.ax5.plot([], [], label='ed (m)')
        self.line_e2, = self.ax5.plot([], [], label='eθ (rad)')
        self.ax5.set_title('Tracking errors')
        self.ax5.set_xlabel('Time [s]'); self.ax5.legend()

        # 6) remaining to goal
        self.line_r1, = self.ax6.plot([], [], label='dist to goal (m)')
        self.line_r2, = self.ax6.plot([], [], label='ang to goal (rad)')
        self.ax6.set_title('Remaining to Goal')
        self.ax6.set_xlabel('Time [s]'); self.ax6.set_ylabel('Remaining')
        self.ax6.legend()

        # refresh timer
        self.create_timer(0.1, self.update_plots)
        self.get_logger().info("LivePlotNode up: 6 sub‐plots in one window.")

    def _stamp(self):
        now = self.get_clock().now()
        if self.start_time is None:
            self.start_time = now
            return 0.0
        return (now - self.start_time).nanoseconds * 1e-9

    def pose_cb(self, msg: Pose2D):
        now = self.get_clock().now()

        # record trajectory
        self.x_data.append(msg.x)
        self.y_data.append(msg.y)

        # compute v, ω from pose difference
        if self.prev_time_pose is not None:
            dt = (now - self.prev_time_pose).nanoseconds * 1e-9
            if dt > 0:
                dx = msg.x - self.prev_x
                dy = msg.y - self.prev_y
                dth = msg.theta - self.prev_theta
                dth = np.arctan2(np.sin(dth), np.cos(dth))
                v = np.hypot(dx, dy) / dt
                w = dth / dt
                t = self._stamp()
                self.times_v.append(t); self.v_vals.append(v)
                self.times_w.append(t); self.w_vals.append(w)

        # save for next
        self.prev_time_pose = now
        self.prev_x = msg.x
        self.prev_y = msg.y
        self.prev_theta = msg.theta

        # compute errors & remaining to goal
        if self.last_goal:
            ed, et = self.compute_errors(
                msg.x, msg.y,
                self.last_goal[0], self.last_goal[1],
                msg.theta
            )
            t = self._stamp()
            # errors subplot (ax5)
            self.err_t.append(t);    self.ed.append(ed)
            self.err_t_th.append(t); self.etheta.append(et)
            # remaining subplot (ax6)
            self.rem_t.append(t);     self.rem_dist.append(ed)
            self.rem_t_th.append(t);  self.rem_angle.append(et)

    def goal_cb(self, msg: Pose2D):
        self.last_goal = (msg.x, msg.y)
        self.goals.append(self.last_goal)

    def cmd_vel_cb(self, msg: Twist):
        t = self._stamp()
        self.ctrl_t.append(t);    self.cmd_v.append(msg.linear.x)
        self.ctrl_t_w.append(t);  self.cmd_w.append(msg.angular.z)

    @staticmethod
    def compute_errors(xr, yr, xg, yg, θr):
        ed = np.hypot(xg - xr, yg - yr)
        θg = np.arctan2(yg - yr, xg - xr)
        et = np.arctan2(np.sin(θg - θr), np.cos(θg - θr))
        return ed, et

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
            allx = list(self.x_data)+list(gx)
            ally = list(self.y_data)+list(gy)
            self.ax3.set_xlim(min(allx)-0.1, max(allx)+0.1)
            self.ax3.set_ylim(min(ally)-0.1, max(ally)+0.1)

        # control inputs
        self.line_cv1.set_data(self.ctrl_t,    self.cmd_v)
        self.line_cv2.set_data(self.ctrl_t_w,  self.cmd_w)
        if self.ctrl_t:
            self.ax4.set_xlim(self.ctrl_t[0], self.ctrl_t[-1])
            cmb = list(self.cmd_v)+list(self.cmd_w)
            self.ax4.set_ylim(min(cmb)-0.1, max(cmb)+0.1)

        # errors
        self.line_e1.set_data(self.err_t,    self.ed)
        self.line_e2.set_data(self.err_t_th, self.etheta)
        if self.err_t:
            self.ax5.set_xlim(self.err_t[0], self.err_t[-1])
            cmb_e = list(self.ed)+list(self.etheta)
            self.ax5.set_ylim(min(cmb_e)-0.1, max(cmb_e)+0.1)

        # remaining to goal
        self.line_r1.set_data(self.rem_t,     self.rem_dist)
        self.line_r2.set_data(self.rem_t_th,  self.rem_angle)
        if self.rem_t:
            self.ax6.set_xlim(self.rem_t[0], self.rem_t[-1])
            cmb_r = list(self.rem_dist)+list(self.rem_angle)
            self.ax6.set_ylim(min(cmb_r)-0.1, max(cmb_r)+0.1)

        # redraw all
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