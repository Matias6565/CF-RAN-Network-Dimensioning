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
import BaseStation as RRH

#Utility class
class Util(object):

	#create a list of RRHs with its own connected processing nodes
	def createRRHs(self, amount,env, service_time, cp):
		rrhs = []
		for i in range(amount):
			r = RRH.RRH(i, [1,0,0,0,0], env, service_time, cp)
			rrhs.append(r)
		self.setMatrix(rrhs)
		return rrhs

	#set the rrhs_matrix for each rrh created
	def setMatrix(self, rrhs):
		count = 1
		for r in rrhs:
			if count <= len(r.rrhs_matrix)-1:
				r.rrhs_matrix[count] = 1
				count += 1
			else:
				count = 1
				r.rrhs_matrix[count] = 1
				count += 1

