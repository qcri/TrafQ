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

        self.PHASES = {
        }

        #how many SUMO seconds before next step (observation/action)
        self.OBSERVEDPERIOD = 10
        self.SUMOSTEP = 1.0

        #In this version the observation space is the set of sensors
        self.observation_space = spaces.Box(low=0, high=255, shape=(1,24), dtype=np.uint8)

        self.detectorIDs = ["e2Detector_--gneE13_0_14",
        "e2Detector_--gneE14_0_17",
        "e2Detector_--gneE15_0_20",
        "e2Detector_--gneE16_0_23",
        "e2Detector_-gneE13_0_4",
        "e2Detector_-gneE14_0_7",
        "e2Detector_-gneE15_0_9",
        "e2Detector_-gneE16_0_11",
        "e2Detector_gneE13_0_5",
        "e2Detector_gneE14_0_6",
        "e2Detector_gneE15_0_8",
        "e2Detector_gneE16_0_10",
        "e2Detector_gneE20_0_0",
        "e2Detector_gneE21_0_1",
        "e2Detector_gneE21_0_13",
        "e2Detector_gneE22_0_16",
        "e2Detector_gneE22_0_2",
        "e2Detector_gneE23_0_19",
        "e2Detector_gneE23_0_3",
        "e2Detector_gneE24_0_22",
        "e2Detector_gneE35_0_12",
        "e2Detector_gneE38_0_15",
        "e2Detector_gneE39_0_22",
        "e2Detector_gneE41_0_23"]

        #Set action space as the set of possible phases
        self.action_space = spaces.Discrete(16)

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

        self.argslist = [sumoBinary, "-c", module_path+"/assets/greenWave.sumocfg",
                             "--collision.action", "teleport",
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
