class GroundContact():
    """
    Class describing ground contacts.
    """

    # constructor
    def __init__(self, name, base_body, base,
                 stiffness_x, max_vx, stiffness_z, max_vz,
                 mu_slide, v_transition, mu_stick):
        self.name = name
        self.base_body = base_body  # home body of contact site
        self.base = base            # contact site

        self.base_x = self.base_body.x + self.base.x_b2w  # contact site location in world frame
        self.base_z = self.base_body.z + self.base.z_b2w

        self.ground_height = 0.0  # ground height
        self.contact = False   # true if in ground contact
        self.kz = stiffness_z  # vertical ground interaction stiffness
        self.max_vz = max_vz   # and maximum relaxation velocity

        self.sliding_mode = True     # horizontal friction state, initialized true
        self.v_trans = v_transition  # transition to stiction <1>
        self.mu_slide = mu_slide     # sliding friction coefficient

        self.x_stick = 0.0        # horiz pos at start of stiction
        self.kx = stiffness_x     # horiz ground interaction stiffness
        self.max_vx = max_vx      # and max relaxation velocity
        self.mu_stick = mu_stick  # stiction coefficient

    # update ground contact
    def update(self, dt, ground_height):

        self.ground_height = ground_height  # not needed (for reporting only) 

        # compute current base location in world frame
        base_x = self.base_body.x + self.base.x_b2w
        base_z = self.base_body.z + self.base.z_b2w

        # check for contact
        if base_z < ground_height:
            self.contact = True

            # compute vertical ground reaction force
            dist_z = ground_height - base_z
            base_vz = (base_z - self.base_z) / dt

            self.base.fz = self.kz * dist_z * \
                (1 - base_vz / self.max_vz) * (base_vz < self.max_vz)

            # compute horizontal ground reaction force
            base_vx = (base_x - self.base_x) / dt

            if self.sliding_mode is True: # initialized as true,
                # compute as sliding mode friction, oppose direction of motion
                if base_vx > 0:
                    self.base.fx = - self.mu_slide * self.base.fz
                else:
                    self.base.fx = self.mu_slide * self.base.fz

                # check for friction mode transition
                if abs(base_vx) < self.v_trans:
                    self.sliding_mode = False
                    self.x_stick = base_x  # store x position when stiction engages
            else:
                # compute as stiction
                dist_x = base_x - self.x_stick
                self.base.fx = self.nonlin_stiction_model(dist_x, base_vx, self.kx, self.max_vx)
                #self.base.fx = self.lin_stiction_model(dist_x, base_vx, 8000, 100)

                # check for friction mode transition
                if abs(self.base.fx) > self.mu_stick * self.base.fz:
                    self.sliding_mode = True
        else:
            # check for transition out of contact
            if self.contact is True:
                self.contact = False
                self.base.fx, self.base.fz = 0.0, 0.0  # reset GRFs
                self.sliding_mode = True  # preset sliding mode

        # store location to compute base velocity again next time
        self.base_x, self.base_z = base_x, base_z

    # compute stiction force as linear spring damper
    @staticmethod
    def lin_stiction_model(dist_x, vx, k, b):
        return - k * dist_x - b * vx

    # compute stiction force as nonlinear spring damper
    @staticmethod
    def nonlin_stiction_model(dist, v, k, max_v):
        if dist >= 0:
            fx = - k * dist * (1 + v / max_v)  # * (v > - max_v)
        else:
            fx = - k * dist * (1 - v / max_v)  # * (v < max_v)
        return fx

######################################################################

# <1>: Velocity at which horizontal friction switches from sliding
# to stiction.