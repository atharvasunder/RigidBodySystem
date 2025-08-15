import math


class Site():
    """
    Class for connection sites at rigid bodies. Interaction classes
    such as joints, ground contacts, and external actuators use 
    connection sites to define forces and torques acting on a body.
    """

    # constructor
    def __init__(self, name, x_b, z_b):
        self.name = name
        self.x_b = x_b  # coords in body frame
        self.z_b = z_b
        self.x_b2w = 0.0  # coords mapped to world frame
        self.z_b2w = 0.0

        self.fx = 0.0     # forces and moments acting at site
        self.fz = 0.0
        self.tau = 0.0

######################################################################
class Anchor():
    """
    Class for defining imovable objects in the plane. Thess objects can
    have sites attached with which other bodies can interact through
    joints and contact points.
    """

    # constructor
    def __init__(self, name, x, z, p):
        self.name = name

        self.x = x    # reference position and orientation
        self.z = z    # in inertial frame
        self.p = p

        self.sites = []  # list of connection sites attached to anchor

        self.geometry = []  # convex hull points for animation

    # create and add connection site to the anchor
    def add_site(self, name, x_b, z_b):
        self.sites.append(Site(name, x_b, z_b))
        self.update_site_coords()

    # map body coords of all sites into world frame coords
    def update_site_coords(self):
        cp, sp = math.cos(self.p), math.sin(self.p)

        for site in self.sites:  # <1>
            site.x_b2w = cp * site.x_b - sp * site.z_b
            site.z_b2w = sp * site.x_b + cp * site.z_b

    # body geometry position in the world frame
    def update_body_geometry(self):
        cp, sp = math.cos(self.p), math.sin(self.p)
        x_w = [self.x + cp * x - sp * z for x, z
               in zip(self.geometry[0], self.geometry[1])]
        z_w = [self.z + sp * x + cp * z for x, z
               in zip(self.geometry[0], self.geometry[1])]

        return x_w, z_w


######################################################################
class Body():
    """
    Class for rigid bodies in the plane. The rigid body is defined by 
    its total mass m at the coordinate origin, a moment of inertia In 
    about it, and connections sites at which other elements apply 
    forces and torques.

    In addition, a body instance stores auxiliary information such
    as the body's geometry used in animations.
    """

    g = 9.8067  # gravitational acceleration

    # construct 2-D rigid body
    def __init__(self, name, mass, moment_of_inertia,
                 x, z, p, vx, vz, vp):
        self.name = name

        self.x = x  # (x, z, p): CoM pos and body pitch
        self.z = z
        self.p = p

        self.vx = vx  # corresponding velocities
        self.vz = vz
        self.vp = vp

        self.m = mass
        self.In = moment_of_inertia  # moment of inertia about CoM

        self.sites = []  # list of connection sites

        self.geometry = []  # convex hull points for animation

    # create and add connection site to the body
    def add_site(self, name, x_b, z_b):
        self.sites.append(Site(name, x_b, z_b))

    # map body coords of all sites into world frame coords
    def update_site_coords(self):
        # print(self.p)
        cp, sp = math.cos(self.p), math.sin(self.p)

        for site in self.sites:  # <1>
            site.x_b2w = cp * site.x_b - sp * site.z_b
            site.z_b2w = sp * site.x_b + cp * site.z_b

    # integrate equations of motion
    def integrate(self, dt):
        # compute net accelerations
        net_fx, net_fz, net_tau = 0.0, 0.0, 0.0
        for site in self.sites:
            net_fx += site.fx
            net_fz += site.fz
            net_tau += site.tau \
                + site.x_b2w * site.fz - site.z_b2w * site.fx

        # perform one step of integration on dynamics using forward euler integration
        self.vx = self.vx + net_fx / self.m * dt
        self.x = self.x + self.vx * dt
        self.vz = self.vz + (net_fz / self.m - self.g) * dt  # accounts for gravity here
        self.z = self.z + self.vz * dt
        self.vp = self.vp + net_tau / self.In * dt
        self.p = self.p + self.vp * dt

        # print(net_tau)

        # update site coordinates
        self.update_site_coords()

    # get body geometry position in the world frame
    def update_body_geometry(self):
        cp, sp = math.cos(self.p), math.sin(self.p)
        x_w = [self.x + cp * x - sp * z for x, z
               in zip(self.geometry[0], self.geometry[1])]
        z_w = [self.z + sp * x + cp * z for x, z
               in zip(self.geometry[0], self.geometry[1])]

        return x_w, z_w


######################################################################

# <1>: The transformed coordinates are stored at the sites and later
# used in the integration method of the RgidBody2D class when
# computing the net torque acting on a body:
# tau_net = tau + x_wrld * fz - z_wrld * fx