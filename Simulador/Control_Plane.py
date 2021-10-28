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

class Control_Plane(object):
	def __init__(self, env, util, type):
		self.env = env
		self.requests = simpy.Store(self.env)
		self.departs = simpy.Store(self.env)
		self.action = self.env.process(self.run())
		self.ilpBatch = None
		self.check_load = simpy.Store(self.env)
		self.check_cloud_load = simpy.Store(self.env)
		
	#take requests and tries to allocate on a RRH
	def run(self):
		global total_aloc
		global total_nonaloc
		global no_allocated
		global count
		global actives
		global incremental_blocking
		global inc_block
		global batch_block
		global count_rrhs
		while True:
			#print(count_rrhs)
			r = yield self.requests.get()
			antenas = []
			antenas.append(r)
