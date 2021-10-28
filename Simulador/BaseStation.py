#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#import Simulation as sim
import random
import csv
import copy
import matplotlib.pyplot as plt
import random as np
import time
from enum import Enum
import numpy
import psutil
from scipy.stats import norm
import copy
import sys
import simpy

#Class RRH
#this class represents a RRH containing its possible processing nodes
class RRH(object):
	def __init__(self, aId, rrhs_matrix, env, service_time, cp):
		self.id = aId
		self.rrhs_matrix = rrhs_matrix
		self.env = env
		self.service_time = service_time
		self.cp = cp
		self.generationTime = 0.0
		self.waitingTime = 0.0

	#updates the generation time
	def updateGenTime(self, gen_time):
		self.generationTime = gen_time

	#updates the waiting time
	def updateWaitTime(self, wait_time):
		self.waitingTime = wait_time - self.generationTime


	def run(self):
		t = np.uniform((next_time -self.env.now)/4, next_time -self.env.now)
		yield self.env.timeout(t)
		self.cp.departs.put(self)

