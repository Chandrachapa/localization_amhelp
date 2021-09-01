#no error seq
#july 23
#last check: aug 30

from pyibex import *
from codac import *
import math
import random
import time
import numpy as np
import pandas as pd
from scipy.spatial import distance
from sklearn.metrics import mean_squared_error

# =================== 0. Import data ====================
df = pd. read_excel (r'5ktownv2.xlsx', sheet_name='priority')
rows = df.shape[0]
columns = df.shape[1]
random.seed(10)
print('rows',rows,df.loc[rows-1][13])
#rows =300
# =================== 0. Parameters, truth and data ====================
dt = 1#time interval of csp problem update
iteration_dt = 0.2#time for solving csp problem

# Truth (unknown pose)
all_x_truth=[]
all_x_est=[]
x_as_bench = []     #x as a benchmark node 
all_x_ids = []
all_emerg_ids = []
all_paths = []
path = []
benchmark_truth = []
benchmark_est = [] 
all_benchmark_est=[]
all_bench_ids =[]
all_theta_est = []
all_y_est =[]
all_eh = []

#urban outdoor LOS
#e_y = Interval(0.1*random.randint(-555,-100),0.1*random.randint(100,555))
#e_toa = Interval(random.randint(-500,-400),random.randint(400,500))
#e_t = Interval(random.randint(-30,-10),random.randint(10,30))
#e_g = Interval(0.1*random.randint(-500,-189),0.1*random.randint(189,500))

#urban outdoor NLOS
#e_y = Interval(0.1*random.randint(-107030,-14381),0.1*random.randint(14381,107030))
#e_toa = Interval(random.randint(-850,-700),random.randint(700,850))
#e_t = Interval(random.randint(-100,-30),random.randint(30,100))#aoa
#e_g = Interval(0.1*random.randint(-9970,-500),0.1*random.randint(500,9970))

#indoor NLOS
e_y = Interval(0.1*random.randint(-500,-4),0.1*random.randint(4,500))
e_toa = Interval(0.001*random.randint(-50000,-714),0.001*random.randint(714,50000))
e_t = Interval(0.01*random.randint(-5000,-125),0.01*random.randint(125,5000))#aoa
e_g = Interval(0.1*random.randint(-217,-20),0.1*random.randint(20,217))

all_area_est =[]
all_sol_est=[]
xgps_all={}
ygps_all={}
all_x_gps={}
all_gps_radius = {}
all_xr=[]
all_yr=[]
all_xrtruth=[]
all_yrtruth=[]
all_er_mean =[]
all_est_coor= []
all_pix_coor = []
specific_er_mean=[]
all_contract_time=[]
id_contract={}
past_bench_ids = {}
time_contract={}
time_arr = []
id_error={}

#2 minute time constraint{#consider this last
earlier_pathid = []
time_limit= 120
tdomain = Interval(120,float(df.loc[rows-1][13])) # [t0,tf]
#t = tdomain.lb()
t=120
t_ub=120
prev_t_obs = 0
dt=0
jj=1#index where data starts
earlier_pathid = []
t_ub=120
time_path = []
all_time_path=[]
id_update = {}
id_updategsp = {}
nodeid_locarea = {}
nodeid_locareagsp = {}
nodeid_b = {}
nodeid_y = {}
update_count = []
all_time = []
ite = 0
iter =0
iterr = 0
path = []
tg = 0
dist_arr = []
error_count={}
time_error ={}
id_count ={}#count number of errors recorded on each id 
toa=0
sol_ids = {}
abids = []
abmest = []
id_arr = []
nodearr =[]
emergarr=[]
# Truth (unknown pose)
all_x_truth=[]
all_x_est=[]
x_as_bench = []     #x as a benchmark node 
all_x_ids = []
all_emerg_ids = []
all_paths = []
path = []
benchmark_truth = []
benchmark_est = [] 
all_benchmark_est=[]
all_bench_ids =[]
all_theta_est = []
all_y_est =[]
all_eh = []
all_area_est =[]
all_sol_est=[]
xgps_all={}
ygps_all={}
all_x_gps={}
all_gps_radius = {}
all_xr=[]
all_yr=[]
all_xrtruth=[]
all_yrtruth=[]
all_er_mean =[]
all_est_coor= []
all_pix_coor = []
specific_er_mean=[]
all_contract_time=[]
id_contract={}
past_bench_ids = {}
time_contract={}
time_arr = []
id_error={}
id_time_arr =[]
#2 minute time constraint{#consider this last
earlier_pathid = []
time_limit= 120
tdomain = Interval(120,float(df.loc[rows-1][13])) # [t0,tf]
#t = tdomain.lb()
t=120
t_ub=120
prev_t_obs = 0
dt=0
jj=1#index where data starts
earlier_pathid = []
t_ub=120
time_path = []
all_time_path=[]
id_update = {}
id_updategsp = {}
nodeid_locarea = {}
nodeid_locareagsp = {}
nodeid_b = {}
nodeid_y = {}
id_time_error = {}
err_minmax = {}
id_time_cdt ={}
id_time_est_coord = {}
update_count = []
all_time = []
ite = 0
iter =0
iterr = 0
path = []
tg = 0
dist_arr = []
error_count={}
time_error ={}
id_count ={}#count number of errors recorded on each id 
toa=0
sol_ids = {}
abids = []
abmest = []
id_arr = []
time_complexity ={}
time_carr = []
node_heading={}
nodearr =[]
emergarr=[]
node_xtruth={}
node_benchids={}
node_y = {}
node_theta = {}
node_benchtruth= {}
sol = []
all_x_gpstheta = {}

#initial gps estimate of each benchmark node 
for i in range(1,rows):
    emerg_ids = df.loc[i][2] # emerg id
    x_ids = df.loc[i][3] # node id
    x1 = df.loc[i][4]
    x2 = df.loc[i][5]
    if xgps_all.get(x_ids) == None:
        gps_radius = 1994.737/(997.368**float(df.loc[i][11]))#worst : 1000, best : 2m
        #gps_radius = 180/(75**float(df.loc[i][11]))
        #gps_radius = 1/(1**float(df.loc[i][11]))
        angle = random.randrange(0,6)
        #print(np.cos(angle))
        #gps_radius=0
        x_gps = gps_radius*np.cos(angle)+x1
        y_gps = gps_radius*np.sin(angle)+x2
        xgps_all[x_ids] = x_gps  
        e_radius = Interval(0,0.1)#distance based error
        #y1_est = Interval(float(df.loc[j][9])+e_radius)     
        if ygps_all.get(x_ids) == None:
            ygps_all[x_ids] = y_gps 
    #print('angle',angle,np.cos(angle),np.sin(angle),gps_radius,x1,x2,x_gps,y_gps)
    e_g = Interval(random.uniform(-(50+gps_radius),-gps_radius),random.uniform(gps_radius,gps_radius+50))
    g1 = Interval(xgps_all[x_ids]+e_g)
    g2 = Interval(ygps_all[x_ids]+e_g)
    #print('angle',angle,np.cos(angle),np.sin(angle),gps_radius,x1,x2,x_gps,y_gps,g1,g2)
    eudist_xtruth = (x1,x2)
    eudist_est = (g1.mid(),g2.mid())
    #print('xtruth',x_ids,eudist_xtruth )
    eudist =  distance.euclidean(eudist_xtruth,eudist_est)
    #print('edist',eudist ,gps_radius)
    xy_gps =IntervalVector([g1,g2])#gps locations of nodes 
    #print('gps_radius',gps_radius)
    #all_gps_radius[x_ids]= Interval(0.01,gps_radius+0.01)
    all_gps_radius[x_ids]= Interval(0.01,eudist+0.01)
    if all_x_gps.get(x_ids) == None:
        all_x_gps[x_ids]=xy_gps#estimated locations of nodes  #estimated locations of nodes 
        x11 = all_x_gps[x_ids][0].mid()
        x22 = all_x_gps[x_ids][1].mid()
        if (all_x_gps[x_ids][0].is_empty()):
            print('yes empty',x_ids,all_x_gps[x_ids][0])
        #heading_gps = atan2(x22,x11)
        heading_gps = atan2(x2,x1)
        #if (heading_gps<0):
          #heading_gps = heading_gps+2*math.pi
        e_h = Interval(heading_gps).inflate(0.01)
        node_heading[x_ids] = e_h
        gps_theta =  math.atan2(x22-x2,x11-x1)
        #if (gps_theta<0):
          #gps_theta = gps_theta+2*math.pi
        all_x_gpstheta[x_ids] = gps_theta
