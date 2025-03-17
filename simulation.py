import os, time
import pybullet as pb
from pybullet_data import getDataPath
import numpy as np

class ErgoJrEnv(object):

    def load_urdf(self):
        fpath = os.path.dirname(os.path.abspath(__file__))
        pb.setAdditionalSearchPath(fpath)
        print(fpath)
        robot_id = pb.loadURDF(
            'poppy_ergo_jr.pybullet.urdf',
            basePosition = (0, 0, 0),
            baseOrientation = pb.getQuaternionFromEuler((0,0,0)),
            useFixedBase=True)
        return robot_id

    def __init__(self,
        control_mode=pb.POSITION_CONTROL,
        timestep=1/240,
        control_period=1,
        show=True,
    ):

        self.control_mode = control_mode
        self.timestep = timestep
        self.control_period = control_period
        self.show = show

        self.client_id = pb.connect(pb.GUI if show else pb.DIRECT)
        if show: pb.configureDebugVisualizer(pb.COV_ENABLE_SHADOWS, 0)
        pb.setTimeStep(timestep)
        pb.setGravity(0, 0, -9.81)
        pb.setAdditionalSearchPath(getDataPath())
        pb.loadURDF("plane.urdf")
        
        self.robot_id = self.load_urdf()
        self.num_joints = pb.getNumJoints(self.robot_id)
        self.joint_name, self.joint_index, self.joint_fixed = {}, {}, {}
        for i in range(self.num_joints):
            info = pb.getJointInfo(self.robot_id, i)
            name = info[1].decode('UTF-8')
            self.joint_name[i] = name
            self.joint_index[name] = i
            self.joint_fixed[i] = (info[2] == pb.JOINT_FIXED)
        
        self.initial_state_id = pb.saveState(self.client_id)

        # reasonable initial viewpoint for arm
        pb.resetDebugVisualizerCamera(
            1.2000000476837158, 56.799964904785156, -22.20000648498535,
            (-0.6051651835441589, 0.26229506731033325, -0.24448847770690918))

    def reset(self):
        # pb.resetSimulation()
        pb.restoreState(stateId = self.initial_state_id)
    
    def close(self):
        pb.disconnect()

    def get_base(self):
        loc, quat = pb.getBasePositionAndOrientation(self.robot_id)
        return (loc, quat)

    def step(self, action):
        pb.setJointMotorControlArray(
            self.robot_id,
            jointIndices = range(len(self.joint_index)),
            controlMode = self.control_mode,
            targetPositions = action,
            targetVelocities = [0]*len(action),
            positionGains = [.25]*len(action), # important for position accuracy
        )
        for _ in range(self.control_period):
            pb.stepSimulation()

    # get/set joint angles as np.array
    def get_position(self):
        states = pb.getJointStates(self.robot_id, range(len(self.joint_index)))
        return np.array([state[0] for state in states])    
    def set_position(self, position):
        for p, angle in enumerate(position):
            pb.resetJointState(self.robot_id, p, angle)

    # convert a pypot style dictionary {... name:angle ...} to joint angle array
    # if convert == True, convert from degrees to radians
    def angle_array(self, angle_dict, convert=True):
        angle_array = np.zeros(self.num_joints)
        for name, angle in angle_dict.items():
            angle_array[self.joint_index[name]] = angle
        if convert: angle_array *= np.pi / 180
        return angle_array
    # convert back to dict from array
    def angle_dict(self, angle_array, convert=True):
        return {
            name: angle_array[j] * (180/np.pi if convert else 1)
            for j, name in enumerate(self.joint_index)}

    # pypot-style command, goes to position in given duration
    # target is a joint angle array
    # speed is desired joint speed
    # if hang==True, wait for user enter at each timestep of motion
    def goto_position(self, target, speed=1., hang=False):

        current = self.get_position()
        distance = np.sum((target - current)**2)**.5
        duration = distance / speed

        num_steps = int(duration / (self.timestep * self.control_period) + 1)
        weights = np.linspace(0, 1, num_steps).reshape(-1,1)
        trajectory = weights * target + (1 - weights) * current

        positions = np.empty((num_steps, self.num_joints))
        for a, action in enumerate(trajectory):
            self.step(action)
            positions[a] = self.get_position()
            if hang: input('..')

        return positions

    # poppy_wrapper-style trajectory tracker
    def track_trajectory(self, trajectory, binsize=None, overshoot=None, ms_rpms = 0.165, hang=False):
        # trajectory = [..., (duration (sec), waypoint) ...]
        # waypoint[name] = angle (deg)
        # returns buffers[t,a] = angle a at timestep t in radians
        
        # pybul doesn't have fast array-version maxvel and pos ctrl is unrealistically fast
        # linearly interpolate waypoints to throttle speed

        buffers = []        
        for (duration, waypoint) in trajectory:
            current = self.get_position()
            target = self.angle_array(waypoint)

            num_steps = int(duration / (self.timestep * self.control_period) + 1)
            weights = np.linspace(0, 1, num_steps).reshape(-1,1)
            interp = weights * target + (1 - weights) * current

            positions = np.empty((num_steps, self.num_joints))
            for a, action in enumerate(interp):
                # self.step(action) # doesn't handle different control periods properly
                self.step_simple(action)
                positions[a] = self.get_position()
                if hang: input('..')

            buffers.append(positions)

        return np.concatenate(buffers, axis=0)

    def get_tip_positions(self):
        states = pb.getLinkStates(self.robot_id, [5, 7])
        return (states[0][0], states[1][0])
    
    def get_camera_image(self):
        width, height = 128, 128
        # width, height = 8, 8 # doesn't actually make much difference
        view = pb.computeViewMatrix(
            cameraEyePosition=(0,-.02,.02),
            cameraTargetPosition=(0,-.4,.02), # focal point
            cameraUpVector=(0,0,.5),
        )
        proj = pb.computeProjectionMatrixFOV(
            fov=135,
            aspect=height/width,
            nearVal=0.01,
            farVal=.4,
        )
        # rgba shape is (height, width, 4)
        _, _, rgba, _, _ = pb.getCameraImage(
            width, height, view, proj,
            flags = pb.ER_NO_SEGMENTATION_MASK) # not much speed difference
        rgba = np.array(rgba).reshape((height, width, 4))
        # rgba = np.empty((height, width, 4)) # much faster than pb.getCameraImage
        return rgba, view, proj

if __name__ == '__main__':

    import matplotlib.pyplot as pt

    env = ErgoJrEnv(pb.POSITION_CONTROL)

    target = 0.5*np.random.randn(env.num_joints)
    env.goto_position(target, speed=0.1)

    rgba, _, _ = env.get_camera_image()
    print(rgba.shape)
    pt.imshow(rgba)
    pt.show()

    # action = [0.]*env.num_joints
    # action[env.joint_index['m6']] = .5
    # while True:
    #     env.step(action)

    env.close()



