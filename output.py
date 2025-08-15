import time
import math
import matplotlib.pyplot as plt


class RbsAnimation:
    """Rigid Body System Simulation"""

    # constructor
    def __init__(self, update_time_step, rigid_body_system, adaptive=False, ratio=1):

        self.time_step = update_time_step
        self.next_time = 0.0
        self.adaptive = adaptive  # adaptive update time flag
        self.ratio = ratio
        self.wall_time = time.time()

        # init figure and axes
        plt.ion() # interactive mode to ensure plots can be updated without closing them
        fig = plt.figure(1)
        fig.clear()
        # fig.set_size_inches(4, 8)
        self.fps_text = fig.text(0.8, 0.01, '(0 fps)', color=[0.5, 0.5, 0.5])
        self.fig = fig

        ax = plt.axes(xlabel='x(m)', ylabel='z(m)')
        ax.axis('equal')
        ax.set(xlim=(-1, 1), ylim=(-2, 2))
        # ax.set(xlim=(-0.25, 0.02), ylim=(-0.2, 0))

        # fig.tight_layout()

        # add horizontal line
        ax.plot([-1000, 1000], [0, 0], color=[0.5, 0.5, 0.5], lw=1)
        self.ax = ax

        # add ground and rigid body line objects
        self.body_lines = []
        for ix in range(len(rigid_body_system.body_list)):
            line, = ax.plot([], [], 'r-', lw=3)
            self.body_lines.append(line)
        
        print('figure initialized')

        plt.show(block=False)

    # Update animation

    def update_animation(self, t, rigid_body_system):

        if t >= self.next_time:

            for ix in range(len(self.body_lines)):
                body = rigid_body_system.body_list[ix]
                x_w, z_w = body.update_body_geometry()

                line = self.body_lines[ix]
                # print(line)
                line.set_data(x_w, z_w)

            # force figure update
            self.fig.canvas.draw_idle()  # schedule gui to redraw
            self.fig.canvas.flush_events()  # flush gui events
            # time.sleep(0.00005)  # Simulate a delay

            # update next time
            if self.adaptive is True:
                current_wall_time = time.time()
                wall_time_diff = current_wall_time - self.wall_time
                self.time_step = self.ratio * wall_time_diff

                # self.fps_text.set_text('(%2.0f fps)' % (1 / wall_time_diff))

                # save wall time for next update
                self.wall_time = current_wall_time

            self.next_time = t + self.time_step

######################################################################

def plot_figure_2(t, x, z, p):
    fig = plt.figure(2)

    fig.clear()
    fig.set_size_inches(5, 3)
    _, axs = plt.subplots(2, 1, num=2)

    # plot trajectory of point mass in space
    axs[0].plot(x, z, 'k', linewidth=2)
    axs[0].set(xlabel='x (m)', ylabel='z (m)')

    # plot orientation
    p_deg = [angle * 180 / math.pi for angle in p]
    axs[1].plot(t, p_deg, 'k', linewidth=2)
    axs[1].set(xlabel='time (s)', ylabel='pitch (deg)')

    fig.show()

######################################################################
def plot_grf(t, grf_x, grf_z, slide_flag):
    fig = plt.figure(3)
    fig.clear()
    # fig.set_size_inches(6, 4)

    ax = plt.axes(xlabel='t (s)', ylabel='GRF (N)')
    ax.plot(t, grf_x, 'k', lw=2)
    ax.plot(t, grf_z, 'r', lw=2)
    slide_flag_100 = [flag * 100 for flag in slide_flag]
    ax.plot(t, slide_flag_100, 'b', lw=1)

    ax.set(ylim=(-200, 2000))

    # fig.tight_layout()
    plt.show()


def plot_joint_angles(t, qa, qk, qh):
    fig = plt.figure(4)

    fig.clear()
    # fig.set_size_inches(6, 6)
    _, axs = plt.subplots(3, 1, num=4)

    # hip angle
    qh_deg = [angle * 180 / math.pi for angle in qh]
    axs[0].plot(t, qh_deg, 'k', lw=2)
    axs[0].set(ylabel='hip angle (deg)')

    qk_deg = [angle * 180 / math.pi for angle in qk]
    axs[1].plot(t, qk_deg, 'k', lw=2)
    axs[1].set(ylabel='knee angle (deg)')

    qa_deg = [angle * 180 / math.pi for angle in qa]
    axs[2].plot(t, qa_deg, 'k', lw=2)
    axs[2].set(xlabel='time (s)', ylabel='ankle angle (deg)')

    plt.show()


def plot_joint_torques(t, taua, tauk, tauh):
    fig = plt.figure(5)

    fig.clear()
    # fig.set_size_inches(6, 6)
    _, axs = plt.subplots(3, 1, num=5)

    # hip torque
    axs[0].plot(t, tauh, 'k', lw=2)
    axs[0].set(ylabel='hip torque (Nm)')

    axs[1].plot(t, tauk, 'k', lw=2)
    axs[1].set(ylabel='knee torque (Nm)')

    axs[2].plot(t, taua, 'k', lw=2)
    axs[2].set(xlabel='time (s)', ylabel='ankle torque (Nm)')

    plt.show()