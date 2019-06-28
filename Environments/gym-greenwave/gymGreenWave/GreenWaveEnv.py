from .assets import set_sumo_home

import logging
import os
from time import sleep
import string
import random
import time

from shutil import copyfile

import numpy as np

import gym
from gym import spaces,error
from gym.utils import seeding

import traci

module_path = os.path.dirname(__file__)

class GreenWaveEnv(gym.Env):
    def __init__(self,oneway=True,uneven=False):

        self._seed = 31337
        self.GUI = GUI
        self.Play = Play
        self.PHASES = {
        }

        #how many SUMO seconds before next step (observation/action)
        self.OBSERVEDPERIOD = 10
        self.SUMOSTEP = 0.5

        #In this version the observation space is the set of sensors
        self.observation_space = spaces.Box(low=0, high=255, shape=(1,68), dtype=np.uint8)

        #Set action space as the set of possible phases
        self.action_space = spaces.Discrete(11)

        self.spec = {}

        #Generate an alphanumerical code for the run label (to run multiple simulations in parallel)
        self.runcode = "".join(random.choices(string.ascii_lowercase + string.digits, k=6))

        self.timestep = 0

        self._configure_environment()

    def seed(self,seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]


    def _configure_environment(self):
        if self.GUI:
            sumoBinary = set_sumo_home.sumoBinaryGUI
        else:
            sumoBinary = set_sumo_home.sumoBinary

        self.argslist = [sumoBinary, "-c", module_path+"/assets/tl.sumocfg",
                             "--collision.action", "remove",
            "--step-length", str(self.SUMOSTEP), "-S", "--time-to-teleport", "-1",
            "--collision.mingap-factor", "0",
            "--collision.check-junctions", "true", "--no-step-log"]

        if self.Play:
            #self.argslist.append("--game")
            self.argslist.append("--window-size")
            self.argslist.append("1000,1000")

        # if self.GUI:
        #     self.arglist.append("--gui-settings-file")
        #     self.arglist.append(module_path+"/assets/viewsettings.xml")

        traci.start(self.argslist,label=self.runcode)

        self.conn = traci.getConnection(self.runcode)

        time.sleep(5) # Wait for server to startup

    def __del__(self):
        self.conn.close()

    def closeconn(self):
        self.conn.close()

    def _selectPhase(self,target):
        target = self.PHASES[target]
        current_program = self.conn.trafficlight.getProgram(self.tlsID)
        if " to " in current_program:
            source = current_program.split(" to ")[1]
        else:
            #we are in another program like scat or 0... just change from N
            source = "G N"
            if source == target:
                source = "G S"
        if source == target and " to " in current_program:
            #ensure that the phase will not be changed automatically by the
            #program, by adding some time
            self.conn.trafficlight.setPhase(self.tlsID, 1)
            self.conn.trafficlight.setPhaseDuration(self.tlsID,60.0)
            return False
        else:
            transition_program = "from %s to %s" % (source,target)
            self.conn.trafficlight.setProgram(self.tlsID, transition_program)
            self.conn.trafficlight.setPhase(self.tlsID, 0)
            return True

    def _observeState(self):
        reward = 0.0
        obs = []
        measures = {}
        #TODO: build observation
        return obs,reward,measures

    def step(self, action):
        self._selectPhase(action)
        #get state and reward
        obs,reward,measures = self._observeState()
        return obs, reward, episode_over, measures

    def reset(self):
        self.timestep = 0
        #go back to the first step of the return
        if self.Play != None and self.Play != "action":
            self.conn.trafficlight.setProgram(self.tlsID, self.Play)
        if self.Play == "action" and self.GUI:
            self.conn.gui.screenshot(viewID='View #0',filename="/home/srizzo/phase_recording/%s_last_screen.png" % self.runcode)

        return self._observeState()[0]
