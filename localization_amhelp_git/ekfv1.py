from numpy.core.multiarray import vdot
#use apd code  based kalman code
#aug 26
#consideration of beacon uncertainities

from pyibex import *
from codac import *
import math
import random
import time
import numpy as np
import pandas as pd
from scipy.spatial import distance
from sklearn.metrics import mean_squared_error
import numpy as np
%matplotlib inline
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from IPython.display import YouTubeVideo
from scipy.stats import norm
from sympy import Symbol, Matrix
from sympy.interactive import printing
printing.init_printing()
from timeit import Timer
# =================== 0. Import data ====================
df = pd. read_excel (r'5ktownv1.xlsx', sheet_name='Input')
rows = df.shape[0]
columns = df.shape[1]
random.seed(10)

#print(rows,df.loc[rows-1](2))
#print('rows',rows,df.loc[rows-1][13])
# =================== 2. Parameters, input data ====================
#aug21
#range measures
#refer:https://github.com/balzer82/Kalman/blob/master/Extended-Kalman-Filter-CHCV.ipynb
#https://github.com/A2Amir/Extended-Kalman-Filter-for-Sensor-Fusion-Radar-and-Lidar

import numpy as np
%matplotlib inline
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from IPython.display import YouTubeVideo
from scipy.stats import norm
from sympy import Symbol, Matrix
from sympy.interactive import printing
printing.init_printing()
import math
from time import time
# =================== 0. Definitions ====================
def savestates(x, Z, P, K):
    x00.append(float(x[0]))
    x11.append(float(x[1]))
    x22.append(float(x[2]))

def initialization():
    m = 1
    dt = 0 # Time Step between Filter Steps
    dts = Symbol('\Delta t')

    u = 0.0
    I = np.eye(4)
    T = 1.0 # s measuremnt time
    m = int(10/1) # number of measurements

    px= 0.0 # x Position Start
    py= 0.0 # y Position Start
    pz= 1.0 # z Position Start

    vx = 0.0 # m/s Velocity at the beginning
    vy = 0.0 # m/s Velocity
    vz = 0.0 # m/s Velocity

    #c = 0.1 # Drag Resistance Coefficient
    #d = 0.9 # Damping
    Xr=[]
    Yr=[]
    Zr=[]

    # Preallocation for Plotting
    xt = []
    yt = []
    zt = []
    dr = []
    thetar = []
    dm = []
    thetam = []
    d = []
    theta = []
    
    # =================== 2. Initialization ====================

    Gs = np.matrix([dts**3/6, dts**2/2, dts])
    Gs*Gs.T
    sj = 0.1

    A = np.matrix([[1.0, 0.0, 0.0, 0],
                  [0.0, 1.0, 0.0, 0.0],
                  [0.0, 0.0, 1.0, 0.0],
                  [0.0, 0.0, 0.0, 1.0]])


    Q = np.matrix([[0,(dt**6)/36, 0, 0],
                  [0, (dt**6)/36, 0,0],
                  [0, 0, (dt**6)/36,0],
                  [0,(dt**5)/12, 0, 0]])*sj**2


    B = np.matrix([[0.0],
                  [0.0],
                  [0.0],
                  [0.0]])

    x = np.matrix([0.0, 0.0, 0.0, 0.0],dtype=float).T

    b1 = np.matrix([2.0, 1.0, 0.0, 0.0],dtype=float).T

    P = np.matrix([[1.0, 0.0, 0.0, 0],
                  [0.0, 1.0, 0.0, 0.0],
                  [0.0, 0.0, 0.0, 0.0],
                  [0.0, 0.0, 0.0, 0.0]],dtype=float)

    m =10#number of measurements
    sp = 1
    Xr = 2
    Yr = 3
    Zr = 4
    thetar = 0.2
    dr = 2
    return x,P,A,Q,B,u,I

# =================== 1. Parameters, input data ====================


# =================== 0. Parameters, truth and data ====================
dt = 1#time interval of csp problem update
iteration_dt = 2#time for solving csp problem

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
node_est_coord = {}
specific_er_mean=[]
all_contract_time=[]
id_time_arr = []
id_contract={}
past_bench_ids = {}
time_contract={}
time_arr = []
id_error={}
err_minmax = {}
x00= []
x11 = []
x22 = []
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
id_time_cdt = {}
id_time_error = {}
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
all_x_gpstheta={}

#indoor NLOS
e_y = Interval(0.1*random.randint(-500,-4),0.1*random.randint(4,500))
e_toa = Interval(0.001*random.randint(-50000,-714),0.001*random.randint(714,50000))
e_t = Interval(0.01*random.randint(-5000,-125),0.01*random.randint(125,5000))#aoa
e_g = Interval(0.1*random.randint(-217,-20),0.1*random.randint(20,217))

#indoor nlos
#nlos
sd_y = 14.038
sd_toa = 14.028
sd_t = 13.93
sd_g = 5.79

#urban outdoor NLOS
e_y = Interval(0.1*random.randint(-107030,-14381),0.1*random.randint(14381,107030))
e_toa = Interval(random.randint(-850,-700),random.randint(700,850))
e_t = Interval(random.randint(-100,-30),random.randint(30,100))#aoa
e_g = Interval(0.1*random.randint(-9970,-500),0.1*random.randint(500,9970))

#nlos
mu_y = 90.55
sd_y = 2638.997
mu_toa = -1040.965
sd_toa = 26652.47
mu_t = 231.096
sd_t = 27544.453
mu_g = -33.67062374
sd_g = 2681.283606

#urban outdoor LOS
#e_y = Interval(0.1*random.randint(-555,-100),0.1*random.randint(100,555))
#e_toa = Interval(random.randint(-500,-400),random.randint(400,500))
#e_t = Interval(random.randint(-30,-10),random.randint(10,30))
#e_g = Interval(0.1*random.randint(-500,-189),0.1*random.randint(189,500))

#testing
#e_toa = Interval(random.randint(-25,0),random.randint(0,25))
#e_y = Interval(random.randint(-25,-15),random.randint(15,25))
#e_g = Interval(1000*random.randint(-3,0),1000*random.randint(0,3))
#e_g = Interval(0,0)
#e_t = Interval(0.1*random.randint(-4,0),0.1*random.randint(0,4))

#no error
#e_toa = Interval(random.randint(0,0),random.randint(0,0))
#e_y = Interval(random.randint(0,0),random.randint(0,0))
#e_g = Interval(0,0)
#e_t = Interval(0.0*random.randint(0,0),0.0*random.randint(0,0))

#initial gps estimate of each benchmark node 
for i in range(1,rows):
    emerg_ids = df.loc[i][2] # emerg id
    x_ids = df.loc[i][3] # node id
    x1 = df.loc[i][4]
    x2 = df.loc[i][5]
    if xgps_all.get(x_ids) == None:
        angle = random.randrange(0,6)
        gps_radius=0
        x_gps = gps_radius*np.cos(angle)+x1
        y_gps = gps_radius*np.sin(angle)+x2
        xgps_all[x_ids] = x_gps  
        e_radius = Interval(0,0.1)#distance based error
        #y1_est = Interval(float(df.loc[j][9])+e_radius)     
        if ygps_all.get(x_ids) == None:
            ygps_all[x_ids] = y_gps 
    #print('angle',angle,np.cos(angle),np.sin(angle),gps_radius,x1,x2,x_gps,y_gps)
    #e_g = Interval(random.uniform(-(50+gps_radius),-gps_radius),random.uniform(gps_radius,gps_radius+50))
    #g1= xgps_all[x_ids]+np.random.normal(loc=1, scale=9970)
    g1 = xgps_all[x_ids]+ sd_g * (np.random.randn(1))
    g2 = ygps_all[x_ids]+ sd_g * (np.random.randn(1))
    #print('angle',angle,np.cos(angle),np.sin(angle),gps_radius,x1,x2,x_gps,y_gps,g1,g2)
    eudist_xtruth = (x1,x2)
    eudist_est = (g1,g2)
    #print('xtruth',x_ids,eudist_xtruth )
    eudist =  distance.euclidean(eudist_xtruth,eudist_est)
    #print('edist',eudist ,gps_radius)
    xy_gps =[g1,g2]#gps locations of nodes 
    #print('gps_radius',gps_radius)
    #all_gps_radius[x_ids]= Interval(0.01,gps_radius+0.01)
    all_gps_radius[x_ids]= eudist+ 0.01 * (np.random.randn(1))
    if all_x_gps.get(x_ids) == None:
        all_x_gps[x_ids]=xy_gps#estimated locations of nodes  #estimated locations of nodes 
        x11 = all_x_gps[x_ids][0]
        x22 = all_x_gps[x_ids][1]
        #heading_gps = atan2(x22,x11)
        heading_gps = atan2(x2,x1)
        #if (heading_gps<0):
          #heading_gps = heading_gps+2*math.pi
        e_h = heading_gps+ 0.01 * (np.random.randn(1))
        node_heading[x_ids] = e_h
        gps_theta =  math.atan2(x22-x2,x11-x1)
        #if (gps_theta<0):
          #gps_theta = gps_theta+2*math.pi
        all_x_gpstheta[x_ids] = gps_theta

# =========== Defining contractors to deal with equations ===========
# We use the predefined contractor ctc.dist, no need to build it
ctc_plus = CtcFunction(Function("a", "b", "c", "a+b-c")) # a+b=c
ctc_minus = CtcFunction(Function("a", "b", "c", "a-b-c")) # a-b=c

# =========== Contractor network ===========
cn = ContractorNetwork()
def is_integer(n):
    try:
        float(n)
    except ValueError:
        return False
    else:
        return float(n).is_integer()
# =================== 0. Definitions ====================
def loc_area(x1,y1,x2,y2):
    area= abs(x1-x2)*abs(y1-y2)
    return area 

def mean_square_error(act_arr,est_arr):#vectors: estimated x1 y1 
  dst   = 0 
  for i in range (0, len(act_arr)):
      a = act_arr[i]
      b = est_arr[i]
      dst += distance.euclidean(a,b)#convert to meters
  avg_error = math.sqrt(dst/len(act_arr))
  return avg_error

def collect_data(j,emergarr,all_x_gps):
    nodeid = df.loc[j][3]
    if (nodeid in nodearr):#nodearr has all unique nodes
        tuy=0
    else:    
        nodearr.append(nodeid)
    emerg_ids = df.loc[j][2] # emerg id
    if (emerg_ids in emergarr):#nodearr has all unique nodes
        tuy=0
    else:    
        emergarr.append(emerg_ids)
    x1 = df.loc[j][4]
    x2 = df.loc[j][5]
    x_truth= [df.loc[j][4],df.loc[j][5],math.atan2(x2,x1)] # (x,y,heading)#n1

    if (node_xtruth.get(nodeid)== None):#nodeid, truth location 
        node_xtruth[nodeid] = x_truth

    toa_availability = False 

    if (node_benchids.get(df.loc[j][3])== None):
        node_benchids[df.loc[j][3]] = str(df.loc[j][6])
    else:
        lst0 = node_benchids[df.loc[j][3]]
        lst = (df.loc[j][6])
        node_benchids[df.loc[j][3]] = str(lst0)+';'+str(lst)

    #benchmark locations
    benchmark1_truth =  [df.loc[j][7],df.loc[j][8]] #benchmark locations
    benchids = node_benchids[df.loc[j][3]].split(';')
    yy =benchids[len(benchids)-1] #last bench id
    #print(yy)
    #theta
    theta_truth = [df.loc[j][10]]#theta
    #theta1 = float(theta_truth[0])
    theta1 = atan2(benchmark1_truth[1]-x2,benchmark1_truth[0]-x1)
    theta_est1 = 0
    #if (theta1<0):
          #theta1 = theta1+2*math.pi
    #theta_est = 0 + 1 * (np.random.randn(1))#estimated theta of bench nodes   
    if (math.isnan(theta1)):
        print('###',nodeid)    
    if theta1!=float(6.28) and (float(theta_truth[0])!=6.28) and (theta1+3)<6.28:#incoverage benchark    
        theta_est1 =theta1 + sd_t * (np.random.randn(1))#estimated theta of bench nodes 
    elif (float(theta_truth[0])==6.28):
        theta_est1 = 0 + sd_t * (np.random.randn(1))#estimated theta of bench nodes 
    else:
        theta_est1 = 0 + sd_t * (np.random.randn(1))#estimated theta of bench nodes 
    if (yy=="gNB"):
        b1 = 410
        b2 = 150 
        toa = float(df.loc[j][13]) #time of arrival
        toa_availability = True 
    elif yy in all_x_gps:
        b1 = all_x_gps[yy][0]#estimated locations of bench nodes 
        b2 = all_x_gps[yy][1]#estimated locations of bench nodes 
        print('gps bench',b1,b2)
    else:
        bench1 = float(benchmark1_truth[0])
        bench2 = float(benchmark1_truth[1])
        b1 = bench1 + sd_g * (np.random.randn(1))#estimated locations of bench nodes 
        b2 = bench2 + sd_g * (np.random.randn(1))#estimated locations of bench nodes 
    benchmark_est = [b1,b2]
    eudist_xtruth = (benchmark1_truth[0],benchmark1_truth[1])
    eudist_est = (x1,x2)
    ydist =  distance.euclidean(eudist_xtruth,eudist_est)
    #y1_truth = float(df.loc[j][9])
    y1_truth = ydist
    #e_y = Interval(-float(df.loc[j][9])*0.01,float(df.loc[j][9])*0.01)#distance based error
    #y1_est = Interval(float(df.loc[j][9])+e_y)
    y1_est = y1_truth+ sd_y * (np.random.randn(1))
    y_est = [y1_est]
    theta_est = [theta_est1]
    if (toa_availability==True):
        benchmark_est =[b1,b2]
        #y3_est = Interval(float(df.loc[j][9])+e_toa)#toa range
        y3_est = y1_truth+ sd_toa * (np.random.randn(1))
        y_est = [y1_est,y3_est]
        theta_est = [theta_est1,theta_est1]#estimated theta of bench nodes 

    if (node_benchtruth.get(nodeid)== None):#nodeid, benchtruth location 
        node_benchtruth[nodeid] = (benchmark_est)
    else:
        lst0 = node_benchtruth[nodeid]
        lst = benchmark_est
        node_benchtruth[nodeid] = (lst0+lst)

    if (node_theta.get(nodeid)== None):#nodeid, theta wrt bench 
        node_theta[nodeid] = (theta_est) 
    else:
        lstt = node_theta[nodeid]
        lsttn = theta_est
        node_theta[nodeid] = (lstt+lsttn)
        #if (nodeid==13590):
            #print('#####',nodeid,lstt,lsttn)

    if (node_y.get(nodeid)== None):#nodeid, range wrt bench 
        node_y[nodeid] = (y_est)   
    else:
        lsty = node_y[nodeid]
        lstyn = y_est 
        node_y[nodeid] = (lsty+lstyn)
    #if (nodeid==int(3837)):
        #print('*&',nodeid, node_theta[nodeid],node_y[nodeid]) 
    return node_benchids,node_benchtruth,node_theta,node_y,node_heading,emergarr

def kalman(node_ids,node_benchids,node_benchtruth,node_theta,node_y,node_heading,timec,emergarr,all_x_gps,all_gps_radius,all_x_gpstheta):
    cn = ContractorNetwork()
    x00= []
    x11 =[]
    x_prev = []
    t0 = 0
    t1 = 0
    #print('t0',float(np.time()))
    
    for iter in node_ids:#for each node recorded till now
            import time
            t0 = time.perf_counter()
            #t0 =float(t.timeit())
            #print('t0',t0)
            id = iter
            b = node_benchtruth[iter]#data stored in the order of nbnode index
            d = node_y[iter]
            theta =  node_theta[iter]
            heading = node_heading[iter] 
            gps = all_x_gps[iter]
            gpstheta = all_x_gpstheta[iter]
            radius_gps = all_gps_radius[iter]
            #print('d',d[0])
            c = 1
            
            x,P,A,Q,B,u,I = initialization()
            #print('lend',len(d))
            x_prev = [x]
            if (node_est_coord.get(iter) is None):
                 x_prev[-1] = x
            else:
                 x_prev[-1]=  node_est_coord[iter]#x estimated in a previous iteration

            #GPS estimation            #GPS coordinated based x estimation
            z = []
            # Project the state ahead
            x = A*x + B*u

            # Project the error covariance ahead
            P = A*P*A.T + Q    

            # Time Update (Prediction)
            # ========================
                
            # Measurement Update (Correction)
            # ===============================
            z0 = np.array([gps[0], gps[1]],dtype=float)

            H  = np.matrix([[1.0, 0.0, 0.0, 0.0],
                    [0.0, 1.0, 0.0, 0.0]],dtype=float)
            rp =31.6**2  # Noise of Position Measurement#maximum gps error 1000m#rp = 31.6
            R = np.matrix([[rp, 0.0],
                          [0.0, rp]],dtype = float)
            S = H*P*H.T + R
                #print('5 S',S)
            inverses = np.linalg.pinv(S)
            # Compute the Kalman Gain
            K = (P*H.T)*inverses 
            #print('z0',z0.shape,'H shape',H.shape,'x shape',x.shape)
            yg = z0 - H*x
            #print('7.5 z',(z.T).shape, hxdash.shape)
            # Update the estimate via z
            x = x + (K*yg)
            #print('8 x',x)
            # Update the error covariance
            P = (I - (K*H))*P
            #print('x gps',x)
            x_prev[-1] = x

            index=0
            err_prev = [0]
            diff_err = 2
            err =100
            if (time_complexity.get(timec) ==None):
                time_complexity[timec] =1
                time_carr.append(float(timec))
            else:
                time_complexity[timec] = 1+time_complexity.get(timec)
            while (index<100 and diff_err>1 and err>10):
            #while (index<100 and err>10):
                x = x_prev[-1]
                if (time_complexity.get(timec) ==None):
                    time_complexity[timec] =len(d)
                    time_carr.append(float(timec))
                else:
                    time_complexity[timec] = len(d)+time_complexity.get(timec)
                for i in range (0,len(d)): # for each range and bearing measurement
                    z = []
                    # Project the state ahead
                    x = A*x + B*u

                    # Project the error covariance ahead
                    P = A*P*A.T + Q    

                    # Time Update (Prediction)
                    # ========================
                        
                    # Measurement Update (Correction)
                    # ===============================
                    #print('i',i,theta)
                    z0 = np.array([d[i], theta[i], d[i]],dtype=float)
                    if (len(d)==1):
                      z = np.array(z0)
                      #print('**',z.shape)
                    else:
                      z = z0.T#observation values  
                    #print((x[0][0]))
                    h1 = math.sqrt((b[0]-x[0])**2+(b[1]-x[1])**2)
                    h2 = math.atan2(b[1]-x[1],b[0]-x[0])
                    #print('tan',type(h1))

                    hxdash  = np.matrix([h1,h2,h1],dtype = float).T
                    rp = 31.6**2  # Noise of Position Measurement
                    R = np.matrix([[rp, 0.0,0],
                                  [0.0, rp,0],
                                  [0.0, 0,rp]],dtype = float)
                    #print(R, R.shape)
                    #print('1 z',z)
                    #print('2 hxdash',hxdash,h1,h2)

                    #print('3 y',y)

                    taninverse_num1  = -1/(b[0]-x[0])**2
                    taninverse_num2  = (b[1]-x[1])/(b[0]-x[0])
                    taninverse_denom = ((b[1]-x[1])/(b[0]-x[0]))**2

                    jh11 = (b[0]-x[0])/(h1)
                    jh12 = (b[1]-x[1])/(h1)
                    jh21 = taninverse_num1 /taninverse_denom
                    jh22 = taninverse_num2 /taninverse_denom

                    jH = np.matrix([[jh11, jh12, 0.0,0],
                                  [jh21, jh22, 0.0, 0.0],
                                  [jh11, jh12, 0.0, 0.0]],dtype=float)
                    #print('4 jH',jH)

                    S = jH*P*jH.T + R
                    #print('5 S',S)
                    inverses = np.linalg.pinv(S)
                    # Compute the Kalman Gain
                    K = (P*jH.T)*inverses 
                    #print('6 K',K)
                    #print('6.5 zT',(z.T).shape)
                    #print('7 hxdash',hxdash.shape,h1,h2)
                    # Innovation or Residual
                    if ((z.T).shape !=hxdash.shape):
                        z = z.T
                    yk = z.T - hxdash
                    #print('7.5 z',(z.T).shape, hxdash.shape)
                    # Update the estimate via z
                    x = x + (K*yk)
                    #print('8 x',x)
                    # Update the error covariance
                    P = (I - (K*jH))*P
                    #print('9 P',P)

                #print('10',x)
                err= mean_square_error(x,x_prev[-1])
                #print(index,'',err,' ',diff_err)
                #print(diff_err>1 or index<100 or err>10)
                x_prev[-1] = x
                if (index>5):
                    diff_err = abs(err - err_prev[-1])
                err_prev[-1] = err
                index+=1
                
            #print('end')#after while loop
            x00.append(float(x[0]))
            x11.append(float(x[1]))
            node_est_coord[iter] = x#after GPS and other observations(rssi,theta,toa)
            #print(type(xt))
            #Save states for Plotting
            #savestates(x, Z, P, K)
            t1 = time.perf_counter()
            #print('t1',t1)
            contraction_dt = t1-t0
            if (math.isnan(timec)==False and time_contract.get(timec)== None):
                  time_contract[timec] = contraction_dt#time taken to localize a device in this time
            elif (math.isnan(timec)==False):
                  time_contract[timec] = contraction_dt+ time_contract.get(timec)
            if iter in emergarr:#only emerg node ids
                  x1 = node_xtruth[iter][0]
                  x2 = node_xtruth[iter][1]
                  all_pix_coor.append([x1,x2])
                  all_est_coor.append([x[0],x[1]])
                  er = mean_square_error(all_pix_coor,all_est_coor)
                  #if (er>20):
                      #print(iter, [x1,x2],p[0].mid(),p[1].mid())
                  all_er_mean.append(0.1*er)#MSE in m^2 
                  key = str(iter)+';'+str(timec)
                  id_time_error[key] = 0.1*er
                  id_time_cdt[key] = contraction_dt
                  id_time_est_coord[key] = (x[0],x[1])
                  id_time_arr.append(key)
                  if (id_error.get(iter)== None):
                      id_error[iter] =0.1*er
                      id_count[iter]= 1
                      id_arr.append(iter)
                  else:
                      id_count[iter]= 1+id_count[iter]
                      id_error[iter] = (0.1*er+id_error.get(iter))/id_count[iter]
                  if (math.isnan(timec)==False and time_error.get(timec)== None):
                      time_error[timec] = 0.1*er#only emerg device
                      error_count[timec]= 1
                      time_arr.append(timec)#use to identify the time
                      err_minmax[timec] = str(0.1*er)
                  elif (math.isnan(timec)==False):
                      time_error[timec] = 0.1*er+time_error.get(timec)#only emerg device
                      error_count[timec]= 1+error_count.get(timec)  
                      err_minmax[timec] = err_minmax.get(timec)+';'+str(0.1*er)
                  if (id_contract.get(iter)== None):
                      id_contract[iter] = contraction_dt
                  else:
                      id_contract[iter] = contraction_dt+ id_contract.get(iter)        
    return sol_ids

# =========== Time constraints ===========
while float(df.loc[jj][13]) <= df.loc[rows-1][13]:#while now is below the upper bound of time domain
    time_error_count =0
    if float(df.loc[jj][13])<= t_ub:  
         nodeid = df.loc[jj][3]
         if (nodeid in nodearr):#nodearr has all unique nodes
              tuy=0
         else:    
              nodearr.append(nodeid)
         node_benchids,node_benchtruth,node_theta,node_y,node_heading,emergarr =collect_data(jj,emergarr,all_x_gps)
         #print('node_y',node_y)
         if (jj<rows):  
            jj+=1# jj: global node index
    else: 
         time_error_count =0
         #print('node_thetaa',node_theta)
         sol_with_ids = kalman(nodearr,node_benchids,node_benchtruth,node_theta,node_y,node_heading,df.loc[jj][13],emergarr,all_x_gps,all_gps_radius,all_x_gpstheta)
         while (float(df.loc[jj][13])> t_ub):    
            t_ub+=120  
         print('routinely update',jj) 
    if (jj==rows):
         break  

print(all_er_mean)

# =========== Export data ===========
import xlwt
list1 = id_contract
list2 = all_er_mean
book = xlwt.Workbook(encoding="utf-8")

i =1
sheet1 = book.add_sheet("Sheet 1")  
for n in range(0,len(id_error)):
    i = i+1#row number
    gg = id_arr[n]
    h  = id_error[int(gg)]
    sheet1.write(i, 0, int(gg))
    sheet1.write(i, 1, h)

k= 1
sheet2 = book.add_sheet("Sheet 2")    
for n in range(0,len(time_error)):
    k = k+1#row number
    gg = time_arr[n]
    h=time_contract[gg]
    sheet2.write(k, 0, gg)
    sheet2.write(k, 1, h)

kk= 1
sheet8 = book.add_sheet("Sheet 8")    
for n in range(0,len(id_contract)):
    kk = kk+1#row number
    gg = id_arr[n]
    h  = id_contract[gg]
    sheet8.write(kk, 0, gg)
    sheet8.write(kk, 1, h)

u = 1
sheet3 = book.add_sheet("Sheet 3")    
for n in range(0,len(time_error)):
    u = u+1#row number
    gg = time_arr[n]
    h=time_error[gg]
    m = error_count[gg]
    dg = h/m
    sheet3.write(u, 0, gg)
    sheet3.write(u, 1, h)
    sheet3.write(u, 2, dg)

y = 1
sheet4 = book.add_sheet("Sheet 4")    
for n in range(0,len(time_complexity)):
    y = y+1#row number
    gg = time_carr[n]
    h=time_complexity[gg]
    sheet4.write(y, 0, gg)
    sheet4.write(y, 1, h)

vu = 1
sheet6 = book.add_sheet("Sheet 6")    
for n in range(0,len(id_time_error)):
    vu = vu+1#row number
    gg = id_time_arr[n]
    h=id_time_error[gg]
    sheet6.write(vu, 0, gg)
    sheet6.write(vu, 1, h)

vuu = 1
sheet9 = book.add_sheet("Sheet 9")    
for n in range(0,len(id_time_cdt)):
    vuu = vuu+1#row number
    gg = id_time_arr[n]
    h=id_time_cdt[gg]
    sheet9.write(vuu, 0, gg)
    sheet9.write(vuu, 1, h)

ve = 1
sheet7 = book.add_sheet("Sheet 7")    
for n in range(0,len(err_minmax)):
    ve = ve+1#row number
    gg = time_arr[n]
    h=err_minmax[gg]
    #split h by ;
    x = h.split(';')
    m = error_count[gg]
    #nb rows = error count
    for pl in range(0,m):
        sheet7.write(pl, ve, x[pl])
    sheet7.write(m+1, ve, gg)

vt = 1
sheet10 = book.add_sheet("Sheet 10")    
for n in range(0,len(id_time_est_coord)):
    vt = vt+1#row number
    gg = id_time_arr[n]
    x = gg.split(';')
    h=id_time_est_coord[gg]
    sheet10.write(vt, 0, gg)
    sheet10.write(vt, 1,str(h))
    sheet10.write(vt, 2, str(node_xtruth[float(x[0])]))

book.save("ekf.xls")