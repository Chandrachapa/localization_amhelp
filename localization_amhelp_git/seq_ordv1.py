#no error seq
#july 23
#last check: sep 2

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
df = pd. read_excel (r'5ktownv4.xlsx', sheet_name='priority')
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
        #gps_radius = 1994.737/(997.368**float(df.loc[i][11]))#worst : 1000, best : 2m
        gps_radius = 180/(75**float(df.loc[i][11]))
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
  #print('apd',avg_error, len(act_arr))
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
    #if (theta1<0):
          #theta1 = theta1+2*math.pi
    if theta1!=float(6.28) and (float(theta_truth[0])!=6.28) and (theta1+3)<6.28:#incoverage benchark    
        theta_est = [Interval(theta1+e_t)]#estimated theta of bench nodes 
    elif (float(theta_truth[0])==6.28):
        theta_est =[Interval(0,6.28)]
    else:
        theta_est =[Interval(0,6.28)]#estimated theta of bench nodes

    if (yy=="gNB"):
        b1 = float(benchmark1_truth[0])
        b2 = float(benchmark1_truth[1])
        toa = float(df.loc[j][13]) #time of arrival
        toa_availability = True 
    elif yy in all_x_gps:
        b1 = all_x_gps[yy][0]#estimated locations of bench nodes 
        b2 = all_x_gps[yy][1]#estimated locations of bench nodes 
        print('gps bench',b1,b2)
    else:
        bench1 = float(benchmark1_truth[0])
        bench2 = float(benchmark1_truth[1])
        b1 = Interval(bench1+e_g)
        b2 = Interval(bench2+e_g)#estimated locations of bench nodes 
    benchmark_est = [IntervalVector([b1,b2])]
    eudist_xtruth = (benchmark1_truth[0],benchmark1_truth[1])
    eudist_est = (x1,x2)
    ydist =  distance.euclidean(eudist_xtruth,eudist_est)
    #y1_truth = float(df.loc[j][9])
    y1_truth = ydist
    #e_y = Interval(-float(df.loc[j][9])*0.01,float(df.loc[j][9])*0.01)#distance based error
    #y1_est = Interval(float(df.loc[j][9])+e_y)
    y1_est = Interval(y1_truth+e_y)
    y_est = [y1_est]
    if (toa_availability==True):
        benchmark_est = [IntervalVector([b1,b2]),IntervalVector([b1,b2])]
        #y3_est = Interval(float(df.loc[j][9])+e_toa)#toa range
        y3_est = Interval(y1_truth+e_toa)
        y_est = [y1_est,y1_est]
        theta_est = [Interval(theta1+e_t),Interval(theta1+e_t)]#estimated theta of bench nodes 

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

    if (node_y.get(nodeid)== None):#nodeid, range wrt bench 
        node_y[nodeid] = (y_est)   
    else:
        lsty = node_y[nodeid]
        lstyn = y_est 
        node_y[nodeid] = (lsty+lstyn)
    #if (nodeid==int(3837)):
        #print('*&',nodeid, node_theta[nodeid],node_y[nodeid]) 
    return node_benchids,node_benchtruth,node_theta,node_y,node_heading,emergarr

def update_solution(bids,best,sol_ids):  
    for iter in bids:
        benchids = bids[iter].split(';')
        yy =benchids[len(benchids)-1] #last bench id
        if (yy !='gNB'):
            print('1',yy,sol_ids.get(int(yy))!=None)
        if (yy =='gNB'):
            tof = 0
        elif sol_ids.get(int(yy))!=None:
            lst0 = best[yy]
            lst = [sol_ids.get(yy)]
            print('lst',lst0,' and',lst)
            best[yy] = (lst0+lst) 
            print('updated')
    return best

def forward_tracking(node_ids,node_benchids,node_benchtruth,node_theta,node_y,node_heading,time,emergarr,all_x_gps,all_gps_radius,all_x_gpstheta):
        cn = ContractorNetwork()
        for iter in node_ids:#for each node recorded till now
                id =iter
                b = node_benchtruth[iter]#data stored in the order of nbnode index
                #print((sol_ids) )
                if (len(sol_ids) != 0 ):
                    #print('benchid in sol id', sol_ids.get(iter)!=None)
                    benchids = node_benchids[iter].split(';')
                    yy =benchids[len(benchids)-1] #last bench id
                    if (yy !='gNB'):
                        tof = 0
                        #print('benchid in sol id', sol_ids.get(int(yy))!=None)
                        if (sol_ids.get(int(yy))!=None):
                            #print('solid',sol_ids[int(yy)])
                            lst = node_benchtruth[iter]
                            #print('before',lst)
                            #m = lst[-1]&sol_ids[int(yy)]&all_x_gps[int(yy)]
                            m = lst[-1]&sol_ids[int(yy)]
                            if (m[0].is_empty()):
                                uyi = 0
                            else:
                                lst[-1] = m
                                #print('seq intersect')
                            #print('after',lst)
                            #print('yy and iter',yy, iter)
                            node_benchtruth[iter] = (lst)
                            #print('done',node_benchtruth[iter])
                            if (time_complexity.get(time) ==None):
                                time_complexity[time] =1
                                time_carr.append(float(time))
                            else:
                                time_complexity[time] = 1+time_complexity.get(time)
                y = node_y[iter]
                theta =  node_theta[iter]
                heading = node_heading[iter] 
                gps = all_x_gps[iter]
                gpstheta = all_x_gpstheta[iter]
                radius_gps = all_gps_radius[iter]
                #print('**',radius_gps)
                if (radius_gps.is_empty()):
                    yyy= 0
                    #print('radius', radius_gps,gps,iter)
                else:
                    c = 1
                    if (time_complexity.get(time) ==None):
                        time_complexity[time] =len(y)
                        time_carr.append(float(time))
                    else:
                        time_complexity[time] = len(y)+time_complexity.get(time)
                    #print('iter',iter,y,b)
                    #r = cn.create_dom(IntervalVector(2))             
                    for i in range (0,len(y)): # for each range and bearing measurement
                          p = cn.create_dom(IntervalVector(2))     
                          d = cn.create_dom(IntervalVector(2))
                          alpha = cn.create_dom(Interval())
                          #heading = cn.create_dom(Interval())
                          #cn.add(ctc.dist, [p, gps, radius_gps])
                          #cn.add(ctc.polar, [d,radius_gps,gpstheta])  
                          cn.add(ctc.dist, [p, b[i], y[i]])# Distance constraint: relation between the state at t_i and the ith beacon position
                          cn.add(ctc_plus, [heading, alpha,theta[i]])#theta =heading+alpha
                          cn.add(ctc.polar, [d[0],d[1],y[i],theta[i]])
                          cn.add(ctc_minus, [b[i],p,d]) #d1 = b1-p1, d2 = b2-p2       
                    r = cn.create_dom(IntervalVector(2)) 
                    gd = cn.create_dom(IntervalVector(2)) 
                    cn.add(ctc.dist, [r, gps, radius_gps])
                    cn.add(ctc.polar, [gd[0],gd[1],radius_gps,gpstheta])  
                    contraction_dt = cn.contract_during(2)
                    cn.contract(True)
                    
                    if p.contains(node_xtruth[iter]):
                        toff=0
                        #print('yes')
                    else:
                        print('no',iter,'y',y)
                        print('b',b)
                        print('p',p)
                        print('p',r)
                        print('xtruth',node_xtruth[iter])
                    vv = p&r            
                    if (vv[0].is_empty()):
                        p = r
                    else:
                        p = vv#change p to intersection between gps estimate r and range based estimate p
                    sol_ids[iter] = p 
                    #print('p and r',iter,p,r,p&r)
                    if (vv[0].is_empty()):
                        tro = 0
                        print('empty',iter)
                        #print('radius_gps',node_xtruth[iter],gps,radius_gps)
                        #print('p and r',iter,p)
                        #print('p and r',iter,p,r,p&r)
                    sol.append(p)   
                    #err mean
                    if (math.isnan(p[0].mid())):  #empty vector occuring place
                        print(p)
                        #print('b',b,'y',y)
                        print('radius', radius_gps,gps,iter)
                        x1 = node_xtruth[iter][0]
                        x2 = node_xtruth[iter][1]
                        gps_radius = 1994.737/(997.368**random.randrange(0,1))#worst : 1000, best : 2m
                        angle = random.randrange(0, 6)
                        x_gps = gps_radius*np.cos(angle)+x1
                        y_gps = gps_radius*np.sin(angle)+x2
                        xgps_all[iter] = x_gps  
                        all_gps_radius[iter]=Interval(0,gps_radius)
                        ygps_all[iter] = y_gps 
                        g1 = Interval(xgps_all[iter]+e_g)
                        g2 = Interval(ygps_all[iter]+e_g)
                        xy_gps =IntervalVector([g1,g2])#gps locations of nodes 
                        all_x_gps[iter]=xy_gps#estimated locations of nodes  #estimated locations of nodes 
                    elif (p[0].is_empty()):
                        print('yea',node_xtruth[iter], all_x_gps[iter],id,all_gps_radius[iter])
                    else: 
                        if (math.isnan(time)==False and time_contract.get(time)== None):
                            time_contract[time] = contraction_dt#time taken to localize a device in this time
                        elif (math.isnan(time)==False):
                            time_contract[time] = contraction_dt+ time_contract.get(time)
                        if iter in emergarr:#only emerg node ids
                              #print('er mean yea')
                              x1 = node_xtruth[iter][0]
                              x2 = node_xtruth[iter][1]
                              all_pix_coor.append([x1,x2])
                              all_est_coor.append([p[0].mid(),p[1].mid()])
                              er = mean_square_error(all_pix_coor,all_est_coor)
                              #if (er>20):
                                  #print(iter, [x1,x2],p[0].mid(),p[1].mid())
                              all_er_mean.append(0.1*er)#MSE in m^2
                              key = str(iter)+';'+str(time)
                              id_time_error[key] = 0.1*er
                              id_time_arr.append(key)
                              id_time_cdt[key] = contraction_dt
                              id_time_est_coord[key] = (p[0],p[1])
                              if (id_error.get(iter)== None):
                                  id_error[iter] = 0.1*er
                                  id_count[iter] = 1
                                  id_arr.append(iter)
                              else:
                                  id_count[iter]= 1+id_count[iter]
                                  id_error[iter] = (0.1*er+id_error.get(iter))/id_count[iter]
                              if (math.isnan(time)==False and time_error.get(time)== None):
                                  time_error[time] = 0.1*er#only emerg device
                                  error_count[time]= 1
                                  time_arr.append(time)#use to identify the time
                                  err_minmax[time] = str(0.1*er)
                              elif (math.isnan(time)==False):
                                  time_error[time] = 0.1*er+time_error.get(time)#only emerg device
                                  error_count[time]= 1+error_count.get(time)
                                  err_minmax[time] = err_minmax.get(time)+';'+str(0.1*er)
                              if (id_contract.get(iter)== None):
                                  id_contract[iter] = contraction_dt
                              else:
                                  id_contract[iter] = contraction_dt+ id_contract.get(iter)          
        return sol_ids
		

while float(df.loc[jj][13]) <= df.loc[rows-1][13]:#while now is below the upper bound of time domain
    time_error_count =0
    if float(df.loc[jj][13])<= t_ub:  
         node_benchids,node_benchtruth,node_theta,node_y,node_heading,emergarr =collect_data(jj,emergarr,all_x_gps)
         #print('node_bids',nodearr)
         if (jj<rows):  
            jj+=1# jj: global node index
    else: 
         time_error_count =0
         sol_with_ids = forward_tracking(nodearr,node_benchids,node_benchtruth,node_theta,node_y,node_heading,df.loc[jj][13],emergarr,all_x_gps,all_gps_radius,all_x_gpstheta)
         while (float(df.loc[jj][13])> t_ub):    
            t_ub+=120  
         print('routinely update',jj)
         #node_benchids = {}#data refresh#change done
         #node_benchtruth = {}
         node_y = {}
         node_theta = {}
         node_ids = {}
         nodearr = [] 
    if (jj==rows):
         break  
    

print(all_er_mean)

# =========== Export data ===========
import xlwt
list1=id_contract
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

ve = 1
vee = 1
sheet7 = book.add_sheet("Sheet 7")    
for n in range(0,len(err_minmax)):
    ve = ve+1#row number
    gg = time_arr[n]
    h=err_minmax[gg]
    #split h by ;
    x = h.split(';')
    m = int(len(x))
    #nb rows = error count
    #print(ve,m)
    if (ve<256):
        for pl in range(0,m):
            #print(x[pl],pl)
            sheet7.write(pl, ve, x[pl])
        sheet7.write(m+1, ve, gg)
    else:
        vee=vee+1
        for pl in range(0,m):
            #print(x[pl],pl)
            sheet7.write(pl+100, vee, x[pl])
        sheet7.write(m+1+100, vee, gg)

kk= 1
sheet8 = book.add_sheet("Sheet 8")    
for n in range(0,len(id_contract)):
    kk = kk+1#row number
    gg = id_arr[n]
    h  = id_contract[gg]
    sheet8.write(kk, 0, int(gg))
    sheet8.write(kk, 1, h)

vuu = 1
sheet9 = book.add_sheet("Sheet 9")    
for n in range(0,len(id_time_cdt)):
    vuu = vuu+1#row number
    gg = id_time_arr[n]
    h=id_time_cdt[gg]
    sheet9.write(vuu, 0, gg)
    sheet9.write(vuu, 1, h)

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

book.save("seq_ord.xls")
