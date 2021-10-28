#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#import Simulation as sim
import random
import csv
import copy
import random as np
import time
from enum import Enum
import numpy
#import psutil
from scipy.stats import norm
import copy
import sys
import Control_Plane as CP
import simpy
import Util as U

total_requested = []
traffics = []
traffic_quocient = 80
served_requests = 0
#count = 0
service_rate = 0.00001
queue_wait = 0
#total_wait = 0
#timestamp to change the load
change_time = 3600
#the next time
next_time = 3600
#inter arrival rate of the users requests
arrival_rate = 3600
#service time of a request
service_time = lambda x: np.uniform(0,100)
#total generated requests per timestamp
total_period_requests = 0
#to generate the traffic load of each timestamp
loads = []
#number of timestamps of load changing
stamps = 24
hours_range = range(1, stamps+1)
for i in range(stamps):
	x = norm.pdf(i, 12, 3)
	x *= traffic_quocient
	loads.append(x)
#first arrival rate of the simulation - to initiate the simulation
arrival_rate = loads[0]/change_time
distribution = lambda x: np.expovariate(arrival_rate)
loads.reverse()
stamps = len(loads)

#traffic generator - generates requests considering the distribution
class Traffic_Generator(object):
	def __init__(self, env, distribution, service, cp):
		self.env = env
		self.dist = distribution
		self.service = service
		self.cp = cp
		self.req_count = 0
		self.action = self.env.process(self.run())
		self.load_variation = self.env.process(self.change_load())

	#generation of requests
	def run(self):
		global total_period_requests
		global rrhs
		while True:
			yield self.env.timeout(self.dist(self))
			self.req_count += 1
			if rrhs:
				r = rrhs.pop()
				self.cp.requests.put(r)
				r.updateGenTime(self.env.now)
				r.enabled = True
				total_period_requests +=1
			else:
				pass

	#changing of load
	def change_load(self):
		while True:
			global traffics
			global arrival_rate
			global total_period_requests,queue_wait
			global next_time, service_rate, total_wait
			global sucs_reqs
			yield self.env.timeout(change_time)
			actual_stamp = self.env.now
			next_time = actual_stamp + change_time
			traffics.append(total_period_requests)
			arrival_rate = loads.pop()/change_time
			self.action = self.env.process(self.run())
			print("Arrival rate now is {} at {} and was generated {}".format(arrival_rate, self.env.now/3600, total_period_requests))
			total_requested.append(total_period_requests)
			total_period_requests = 0
			sucs_reqs = 0


if __name__ == "__main__":
	util = U.Util()
	env = simpy.Environment()
	cp = CP.Control_Plane(env, util, 'batch')
	rrhs = util.createRRHs(1000, env, service_time, cp)
	np.shuffle(rrhs)
	t = Traffic_Generator(env, distribution, service_time, cp)
	print("\Begin at "+str(env.now))
	env.run(until = 86401)
	#print(t)
	print("\End at "+str(env.now))

