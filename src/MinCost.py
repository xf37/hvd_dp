# -*- coding: utf-8 -*-
"""
Created on Mon Apr 23 13:35:08 2018

@author: Xin Feng
"""

import shapefile
import _pickle as cPickle
from shapely.geometry import Point, Polygon, LineString, MultiPoint, MultiPolygon
from collections import defaultdict
from gurobipy import *

#number = 776
number = 79340

path = "C:\\Users\\Xin Feng\\Dropbox\\Who&Xin\\Yuan_Xin\\HVD\\HVD_dp\\Xpress\\"


f = open(path +  "no_obs_location.txt", 'r')
lines = f.read().split('\n');
nobs_list = []
nobs_list_index = [0 for x in range(1,number)]
for i in range(len(lines)-1):
    line = lines[i].split('\t');                                                                                                                                        
    nobs_list.append(int(line[1]));
    nobs_list_index[int(line[1])] = int(line[0])-1


f = open(path +  "dms_location.txt", 'r')
lines = f.read().split('\n');
dms_list = []
dms_list_index = [0 for x in range(1,number)]
for i in range(len(lines)-1):
    line = lines[i].split('\t');                                                                                                                                        
    dms_list.append(int(line[1]));
    dms_list_index[int(line[1])] = int(line[0])-1
    

f = open(path +  "ems_location.txt", 'r')
lines = f.read().split('\n');
ems_list = []
for i in range(len(lines)-1):
    line = lines[i].split('\t');                                                                                                                                        
    ems_list.append(int(line[1]));


f = open(path +  "out_cost_wind_new.txt", 'r')
lines = f.read().split('\n');
phi_list = [[] for x in range(1,number)]
psi_list = [[] for y in range(1,number)]
cost_matrix = [[0 for x in range(1,number)] for y in range(1,number)] 


for i in range(len(lines)-1):
    line = lines[i].split('\t')                                                                                                                                       
    phi_list[int(line[2])].append(int(line[1]))
    psi_list[int(line[1])].append(int(line[2]))
    cost_matrix[int(line[2])][int(line[1])] = float(line[3])
    
####################################


###################################

# Create initial model
m = Model('MinCost')

# Create variables

X = []
for k in dms_list:
    print ("%d" %k)
    xk = []
    for i in dms_list:
        xki = []
        for j in nobs_list:
            xki.append(m.addVar(vtype=GRB.BINARY, name="X%d_%d_%d" %(k,i,j)))
        xk.append(xki)
    X.append(xk)
m.update()


# Set objective
m.setObjective(quicksum([quicksum([quicksum([X[k][i][nobs_list_index[j]]*cost_matrix[dms_list[i]][j] for j in phi_list[dms_list[i]]]) for i in range(len(dms_list))]) for k in range(len(dms_list))]),GRB.MINIMIZE)
#m.setObjective(quicksum([quicksum([p_y[i][j]*l_cost[j] for j in range(llt)]) for i in range(p)]), GRB.MINIMIZE)


# Add constrains
for k in range(len(dms_list)):
    print (k)
    m.addConstr(quicksum(X[k][k][nobs_list_index[j]] for j in phi_list[dms_list[k]]) == 1)    #constrain (7)
#        m.addConstr(quicksum(p_y[i][yi] for yi in point_table_in[start_pts[i]]) == 0)  
#    m.addConstr(quicksum( quicksum( X[k][nobs_list_index[i]][g]   for i in psi_list[ems_list[g]]) for g in range(len(ems_list)) )== 1)
    m.addConstr(quicksum(quicksum(X[k][dms_list_index[i]][nobs_list_index[ems_list[g]]] for i in psi_list[ems_list[g]]) for g in range(len(ems_list))) == 1)        #constrain (8)
#        m.addConstr(quicksum(p_y[i][yi] for yi in point_table_out[end_pts[i]]) == 0)      


#m.addConstr(quicksum(x[xj] for xj in range(lpt)) == 1)     #constrain (5)
for k in range(len(dms_list)):
    print (k)
    for i in range(len(dms_list)):
         if dms_list[i] != dms_list[k]:
             m.addConstr(quicksum(X[k][i][nobs_list_index[j]] for j in phi_list[dms_list[i]]) == quicksum(X[k][dms_list_index[j]][nobs_list_index[dms_list[i]]] for j in psi_list[dms_list[i]]))

                                   
#for i in range(p):
#    for xj in range(lpt):
#        if (xj not in start_pts) and (xj not in end_pts):
#            m.addConstr(quicksum(p_y[i][yi_in] for yi_in in point_table_in[xj]) == quicksum(p_y[i][yi_out] for yi_out in point_table_out[xj]) )      #constrain (8)


#for i in range(p):
#    for xj in range(lpt):
#        m.addConstr(quicksum(p_y[i][yi] for yi in point_table_in[xj]) >= x[xj])
#        m.addConstr(quicksum(p_y[i][yi] for yi in point_table_out[xj]) >= x[xj])   #constrain (9)

m.optimize()




####### print/record the optimal solution
for k in range(len(dms_list)):
    for g in range(len(ems_list)):
        for i in psi_list[ems_list[g]]:
            if X[k][dms_list_index[i]][nobs_list_index[ems_list[g]]].x == 1:
                print (dms_list[k], ems_list[g])

"""
for xj in range(lpt):
    if x[xj].x == 1:     #is selected as meet up point 
        print xj

output_line = []
for yi in range(llt):
    for i in range(p):
        if p_y[i][yi].x == 1:      #is selected as links 
             if yi >= llt/2:
                 output_line.append(yi-llt/2)
             else:
                 output_line.append(yi)
"""