import time
from config import human_model
from control import StanceController
from output import RbsAnimation, plot_joint_angles, plot_joint_torques, plot_grf, plot_figure_2
from trackers import ContactTracker

# INTEGRATION PARAMETERS
dt = 1e-4                  # [s] integration time step
tStop = 5              # [s] simulation stop time

print('\033[H\033[J')  # clear screen (equivalent to Matlab 'clc')

# create rigid body system
rbs = human_model()

# bodies stored in variables
trunk = rbs.body_list[0]
foot = rbs.body_list[3]

ball = rbs.contact_list[0]
ball_tracker = ContactTracker(ball)

# joints stored in variables
hip = rbs.joint_list[0]
knee = rbs.joint_list[1]
ankle = rbs.joint_list[2]

# stance_ctrl = StanceController(trunk.p, ankle.q, knee.q, hip.q)

qh, qk, qa = [hip.q], [knee.q], [ankle.q] # joint angles list, used for plotting
tauh, tauk, taua = [hip.tau], [knee.tau], [ankle.tau] # joint torques list, used for plotting

rbs_anim = RbsAnimation(update_time_step=0.04, rigid_body_system=rbs, adaptive=True, ratio=0.1)

# print('moving on')

start_time = time.time()

t = [0.0]

while t[-1] <= tStop:

    # animate rbs
    # print('going to updATE THE ANIMATION')
    rbs_anim.update_animation(t[-1], rigid_body_system=rbs)

    # append joint angles to a list for plotting
    qh.append(hip.q)
    qk.append(knee.q)
    qa.append(ankle.q)

    # apply controller is the foot is in stance
    if ball.contact is True:
        # tau_h, tau_k, tau_a = stance_ctrl.update(t[-1], dt, trunk.p, ankle.q, knee.q, hip.q)
        tau_h, tau_k, tau_a = 0.0, 0.0, 0.0
    else:
        tau_h, tau_k, tau_a = 0.0, 0.0, 0.0

    # print('tau_h: %4.2f, tau_k: %4.2f, tau_a: %4.2f\n' % (tau_h, tau_k, tau_a))

    # Store joint torques in a list for plotting
    tauh.append(hip.tau)
    tauk.append(knee.tau)
    taua.append(ankle.tau)

    # update contact point
    ball.update(dt, ground_height=0)
    ball_tracker.append()

    # print(tau_h)

    # Need to update the mtc as well as the activation here!!!!!

    # update forces and torques acting at all joints
    hip.update(dt, tau_h)
    knee.update(dt, tau_k)
    ankle.update(dt, tau_a)

    # integrate a single timestep
    for body in rbs.body_list:
        body.integrate(dt)
        # print('integrated')

    # print(t)
    t.append(t[-1] + dt)

print('Integration time: %f sec.' % (time.time() - start_time))

plot_joint_angles(t, qa, qk, qh)
plot_joint_torques(t, taua, tauk, tauh)

# plot_figure_2(t, x, z, p)

# ball_tracker.plot(t=t, fig_num='Ball contact point of foot segment')