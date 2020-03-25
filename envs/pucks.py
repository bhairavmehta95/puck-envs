import os

import numpy as np
from gym import utils
from gym.envs.mujoco import mujoco_env
import xml.etree.ElementTree as et

import mujoco_py


class PucksEnv(mujoco_env.MujocoEnv, utils.EzPickle):
    def __init__(self, **kwargs):
        utils.EzPickle.__init__(self)
        self.reference_path = os.path.join(os.path.dirname(os.path.realpath(__file__)),
                                           'assets/pucks.xml')
        mujoco_env.MujocoEnv.__init__(self, self.reference_path, frame_skip=1)

        self.model.stat.extent = 10

    def _re_init(self, xml):
        self.model = mujoco_py.load_model_from_xml(xml)
        self.sim = mujoco_py.MjSim(self.model)
        self.data = self.sim.data
        self.init_qpos = self.data.qpos.ravel().copy()
        self.init_qvel = self.data.qvel.ravel().copy()
        observation, _reward, done, _info = self.step(np.zeros(self.model.nu))
        assert not done
        if self.viewer:
            self.viewer.update_sim(self.sim)

    # TODO: I'm making an assumption here that 3 places after the comma are good enough, are they?
    def _randomize_friction(self):
        frictionloss = self.dimensions[0].current_value

        for joint in self.object_joints:
            joint.set('frictionloss', '{:3f}'.format(frictionloss))

    def step(self, action):
        reward = 0.0
        self.do_simulation(action, self.frame_skip)
        ob = self._get_obs()
        done = False

        return ob, reward, done, {}
    def viewer_setup(self):
        coords = [.7, -.5, 0]
        for i in range(3):
            self.viewer.cam.lookat[i] = coords[i]
        self.viewer.cam.trackbodyid = -1
        self.viewer.cam.distance = 2

    def reset_model(self):
        qpos = self.init_qpos
        qvel = self.init_qvel

        self.set_state(qpos, qvel)
        return self._get_obs()

    def _get_obs(self):
        return np.concatenate([
            self.sim.data.qpos.flat,
            self.sim.data.qvel.flat,
        ])
