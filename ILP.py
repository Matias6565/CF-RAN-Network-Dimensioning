#!/usr/bin/env python
# -*- coding: utf-8 -*-

import math
import logging
#import cplex
import sys
import time
import random as rand
from docplex.mp.model import Model
import docplex.mp
from docplex.cp.model import *
import simpy
import functools
import random as np
import importlib
import csv
import numpy
import pandas as pd
import cplex

# Bloqueio para o restante sobrante. 1 req = 614.4
#total traffic resto da divisão por req = req bloq
# ==============================
__author__ = 'Matias Romário'
__email__ = "matiasrps@ufba.br/matiasromario@ieee.org"
__version__ = '1.0'
# ==============================

act_cloud = 0
act_fog = 0
act_lambda = 0

fog_delay = 0.0000980654 # 10km
cloud_delay = 0.000049033 # 10km 0,000033356
ONU_Activation_delay = 0.0000016 # 1,6 µs
LC_Activation_delay = 0.0000015 # 1,5 µs 0.000069283

# Taxa CPRI 
Band = 1000000 #Very big Num
#cpri = [614.4, 122.88, 460.8, 552.96, 0] # Valores do tráfego por split

cpri = [1966, 74, 119, 674.4] 


#Use case: One-way latency DL bandwidth UL bandwidth.Source: 159_functional_splits_and_use_cases_for_sc_virtualization.pdf
#Latency_Req = [0.000050967, 0.000450967, 0.000450967, 0.001450967] # 1 ->C-RAN; 2 -> PHY; 3-> Split MAC ; 4-> PDCP-RLC. 0.00025 - cl_delay
Latency_Req = [0.0001, 0.0025, 0.0005, 0.0003] # 1 ->C-RAN; 2 -> PHY; 3-> Split MAC ; 4-> PDCP-RLC. 0.00025 - cl_delay
Delay = [0.000057033, 0.000024516,0.000024516, 0.000024516, 0.000024516]#Atraso de transmissão de nó

cpri_rate = 614.4 # Taxa CPRI
wavelength_capacity = [10000.0, 10000.0, 10000.0, 10000.0, 10000.0, 10000.0, 10000.0, 10000.0, 10000.0, 10000.0, 10000.0,10000.0, 10000.0, 10000.0, 10000.0, 10000.0]
lambda_state = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
switchBandwidth = [10000.0,10000.0,10000.0,10000.0,10000.0,10000.0,10000.0,10000.0,10000.0,10000.0]

switch_state = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
rrhs_on_nodes = [0,0,0,0,0,0,0]
#Amount_onus = []
split_state = [0,0,0,0,0,0,0,0]


# rrhs
rrhs = range(0,1)
# total de nós #TODO resolver conflito ao aumentar os nós
nodes = range(0, 4)#4
#Total de Split TODO: precisa de ajuste
Split= range(0, 4) # São 4
# total de lambdas
lambdas = range(0, 10)
node_capacity = [60000, 10000, 10000, 10000]#cf-ran and split


# Custo dos Nós
nodeCost = [600.0, 300.0, 300.0, 300.0, 300.0, 300.0, 300.0, 300.0]
# Custo do line card
lc_cost = [20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0]
# Custo split
s_cost = [0.0, 20.0, 15.0, 10.0, 5.0]
#Custo por Onu ativada
Onu_cost = 7.7
RRH_cost = 20
nodeState = [0,0,0,0,0,0,0]

lambda_node = [
[1,1,1,1,1,1],
[1,1,1,1,1,1],
[1,1,1,1,1,1],
[1,1,1,1,1,1],
[1,1,1,1,1,1],
[1,1,1,1,1,1],
[1,1,1,1,1,1],
[1,1,1,1,1,1],
[1,1,1,1,1,1],
[1,1,1,1,1,1],
[1,1,1,1,1,1],
[1,1,1,1,1,1],
[1,1,1,1,1,1],
[1,1,1,1,1,1],
[1,1,1,1,1,1],
]


#Classe ILP
class ILP(object):

	# Iniciando
	def __init__(self, rrh, rrhs, nodes, lambdas, Split):
		self.rrh = rrh
		self.fog = []
		for i in rrh:
			self.fog.append(i.rrhs_matrix)
		self.rrhs = rrhs
		self.nodes = nodes
		self.lambdas = lambdas
		self.Split = Split

	def run(self):
		self.buildModel()
		sol = self.solveILP()
		return sol

	#Roteiro de execução
	def run_relaxed(self):
		self.buildModel_relaxed()
		sol = self.solveILP()
		return sol


	# Construção do modelo inteior
	def buildModel(self):
		self.mdl = Model()

		cpx = self.mdl.get_engine().get_cplex()
		#================ Variáveis de decisão ================
		self.x = self.mdl.binary_var_dict([(i,j,w,s) for i in self.rrhs for j in self.nodes for w in self.lambdas for s in self.Split], name = 'Rrh/Node/Lambda/Split: ')
		self.k = self.mdl.binary_var_dict([(w, j, s) for w in self.lambdas for j in self.nodes for s in self.Split], name = 'Lambda/Nonde/Split: ')
		self.y = self.mdl.binary_var_dict([(i,j) for i in self.rrhs for j in self.nodes], name = 'RRH/Node: ')
		self.xn = self.mdl.binary_var_dict([(j) for j in self.nodes], name = 'Node activated: ')
		self.z = self.mdl.binary_var_dict([(w, j) for w in self.lambdas for j in self.nodes], name = 'Lambda/Node: ')
		self.t = self.mdl.binary_var_dict([(i,j,s) for i in self.rrhs for j in self.nodes for s in self.Split], name = 'RRH/Node/Split: ')
		self.s = self.mdl.binary_var_dict([(i,s) for i in self.rrhs for s in self.Split], name = 'Rhh/Split: ')
		self.g = self.mdl.binary_var_dict([(i,j,w,s) for i in self.rrhs for j in self.nodes for w in self.lambdas for s in self.Split], name = 'Redirections: ')
		self.e = self.mdl.binary_var_dict([(j) for j in self.nodes], name = "Switch/Node")



	# ================ Constraints desenhadas ================
		self.mdl.add_constraints(self.mdl.sum(self.x[i,j,w,s] for j in self.nodes for w in self.lambdas for s in self.Split) == 2 for i in self.rrhs) # 2 VPONs
		self.mdl.add_constraints(self.mdl.sum(self.z[w,j] for j in self.nodes) <= 1 for w in self.lambdas) # Um nó por Lâmbda
		#self.mdl.add_constraints(self.mdl.sum(self.z[w,j] for w in self.lambdas)<= self.xn[j] for j in self.nodes)
		self.mdl.add_constraints(self.mdl.sum(self.s[i,s] for s in self.Split) == 1 for i in self.rrhs) # 1 Split iguais por rrh
		self.mdl.add_constraints(self.mdl.sum(self.y[i,j] for j in self.nodes) == 2 for i in self.rrhs)# serão dois nós por rrhs
		self.mdl.add_constraints(self.mdl.sum(self.y[i,j] for j in self.nodes[0:1]) == 1 for i in self.rrhs)# 1 obrigatoriamente na nuvem
		self.mdl.add_constraints(self.y[i,0] == 1 for i in self.rrhs) # invés do for
		self.mdl.add_constraints(self.t[i,j,s] <= self.mdl.sum(self.s[i,s]) for s in self.Split for j in self.nodes for i in self.rrhs)
		self.mdl.add_constraints(self.t[i,j,s] == self.mdl.sum(self.x[i,j,w,s] for w in self.lambdas) for w in self.lambdas for j in self.nodes for i in self.rrhs for s in self.Split)

		
		#Restrições de Capacidade de Banda - Restrições para quebra do CPRI - nó 0 é nuvem e nó 1 é fog
		self.mdl.add_constraints(self.mdl.sum(self.x[i,j,w,s] * cpri[s] for s in self.Split for i in self.rrhs for j in self.nodes[0:1]) <= wavelength_capacity[w] for w in self.lambdas) 
		self.mdl.add_constraints(self.mdl.sum(self.x[i,j,w,s] * (cpri[0] - cpri[s]) for s in self.Split for i in self.rrhs for j in self.nodes[1:]) <= wavelength_capacity[w] for w in self.lambdas)
		self.mdl.add_constraints(self.mdl.sum(self.x[i,j,w,s] * cpri[s] for s in self.Split for i in self.rrhs for w in self.lambdas) <= node_capacity[j] for j in self.nodes[0:1]) # Nó zero pega a parte da cpri com split
		self.mdl.add_constraints(self.mdl.sum(self.x[i,j,w,s] * (cpri[0] - cpri[s]) for s in self.Split for i in self.rrhs for w in self.lambdas) <= node_capacity[j] for j in self.nodes[1:])# nó de fog pega o remanescente


		self.mdl.add_constraints(Band*self.xn[j] >= self.mdl.sum(self.x[i,j,w,s] for s in self.Split for i in self.rrhs for w in self.lambdas) for j in self.nodes)
		self.mdl.add_constraints(self.xn[j] <= self.mdl.sum(self.x[i,j,w,s] for s in self.Split for i in self.rrhs for w in self.lambdas) for j in self.nodes)
		self.mdl.add_constraints(Band*self.z[w,j] >= self.mdl.sum(self.x[i,j,w,s] for s in self.Split for i in self.rrhs) for w in self.lambdas for j in self.nodes)
		self.mdl.add_constraints(self.z[w,j] <= self.mdl.sum(self.x[i,j,w,s] for s in self.Split for i in self.rrhs) for w in self.lambdas for j in self.nodes)
		self.mdl.add_constraints(Band*self.y[i,j] >= self.mdl.sum(self.x[i,j,w,s] for s in self.Split for w in self.lambdas) for i in self.rrhs for j in self.nodes)
		#self.mdl.add_constraints(self.z[w,j] <= lambda_node[w][j] for w in self.lambdas for j in self.nodes)

		#Switch node
		self.mdl.add_constraints(Band*self.e[j] >= self.mdl.sum(self.y[i,j] for i in self.rrhs) for j in self.nodes)
		self.mdl.add_constraints(self.e[j] <= self.mdl.sum(self.y[i,j] for i in self.rrhs)  for j in self.nodes)

    #Omited Latency Restrictions
  
  	def solveILP(self):
		self.mdl.minimize(self.mdl.sum(self.xn[j] * nodeCost[j] for j in self.nodes) + self.mdl.sum(self.z[w,j] * lc_cost[w] for w in self.lambdas for j in self.nodes) + self.mdl.sum(self.s[i,s] * s_cost[s] for s in self.Split for i in self.rrhs))

		#self.sol = self.mdl.solve(log_output=True) #- Mostra o relatório de gap
		self.mdl.parameters.lpmethod = 6
		self.sol = self.mdl.solve()
		self.sol.display()
		return self.sol
    
    #Omitted post processing functions
    
  def get_gaps(best_s, bound_s):
	  '''
	  Obtém os valores numéricos do intervalo entre o valor objetivo e o limite do objetivo.
	  Para uma função objetivo única, temos: gap = | value - bound | / max (Val | valor |)
	  TODO: Para várias funções objetivo: cada intervalo é o intervalo entre o valor correspondente e o limite. 
	  '''
	  a = max(best_s, bound_s)
	  b = min(best_s, bound_s)
	  solution = math.fabs(a - b)/math.fabs(a)#
	  return solution




u = Util()
antenas = u.newCreateRRHs(40)
np.shuffle(antenas)
ilp = ILP(antenas, range(len(antenas)), nodes, lambdas, Split)
solution = ilp.run()
solu = ilp.return_solution_values()
#ilp.print_var_values()
ilp.updateValues(solu)

print("---------------- ILP --------------------------")
print("Tempo: {}".format(solution.solve_details.time))
print("Energia_ideal: {}".format(solution.objective_value))
print("Energia: {}".format(u.getPowerConsumption()))
print("tráfego na fog atualizado: {}".format(ilp.Fog_Band(solu)))
print("tráfego na coud atualizado: {}".format(ilp.Cloud_Band(solu)))
#print("Atraso total: {}".format(ilp.Delay_total(solu)))
print("---------------- ILP --------------------------")
