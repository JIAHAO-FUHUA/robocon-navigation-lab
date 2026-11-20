"""CPU teaching simulation: A*, footprint inflation, motion timing and cascaded PID.
Python 3.10+, standard library only. This is not a FAST-LIO or motor-hardware emulator.
"""
import argparse,csv,heapq,json,math,pathlib,random,time
from viewer import write_viewer

def clip(v,a,b):return max(a,min(b,v))
def wrap(a):return (a+math.pi)%(2*math.pi)-math.pi
def dist(a,b):return math.hypot(a[0]-b[0],a[1]-b[1])

class PID:
 def __init__(self,kp,ki,kd,limit,ilimit):
  self.kp,self.ki,self.kd,self.limit,self.ilimit=kp,ki,kd,limit,ilimit;self.integral=0.;self.last=None
 def step(self,error,dt):
  derivative=0. if self.last is None else (error-self.last)/dt
  proposed=clip(self.integral+error*dt,-self.ilimit,self.ilimit)
  raw=self.kp*error+self.ki*proposed+self.kd*derivative
  if abs(raw)<self.limit or raw*error<0:self.integral=proposed
  self.last=error
  return clip(self.kp*error+self.ki*self.integral+self.kd*derivative,-self.limit,self.limit)
 def reset(self):self.integral=0.;self.last=None

class Grid:
 def __init__(self,cfg,scenario):
  self.width,self.height=cfg['field_m'];self.res=cfg['grid_m'];self.obstacles=[list(x) for x in cfg['obstacles_m']]
  if scenario=='blocked':self.obstacles.append([4.6,0,5.4,self.height])
  if scenario=='replan':self.obstacles.append([6.9,4.3,7.9,6.5])
  self.radius=math.hypot(*[v/2 for v in cfg['robot_size_m']])+cfg['safety_margin_m'];self.nx=round(self.width/self.res)+1;self.ny=round(self.height/self.res)+1
 def valid(self,p,margin=None):
  x,y=p;r=self.radius if margin is None else margin
  if not(r<x<self.width-r and r<y<self.height-r):return False
  for a,b,c,d in self.obstacles:
   if math.hypot(max(a-x,0,x-c),max(b-y,0,y-d))<=r:return False
  return True
 def cell(self,p):return (round(p[0]/self.res),round(p[1]/self.res))
 def point(self,c):return (c[0]*self.res,c[1]*self.res)
 def line(self,a,b):
  n=max(1,math.ceil(dist(a,b)/(self.res/4)))
  return all(self.valid((a[0]+(b[0]-a[0])*i/n,a[1]+(b[1]-a[1])*i/n)) for i in range(n+1))
 def astar(self,start,goal):
  s,g=self.cell(start),self.cell(goal)
  if not self.valid(self.point(s)) or not self.valid(self.point(g)):return []
  q=[(0.,s)];cost={s:0.};parent={}
  while q:
   _,u=heapq.heappop(q)
   if u==g:
    cells=[u]
    while u!=s:u=parent[u];cells.append(u)
    pts=[self.point(c) for c in cells[::-1]];pts[0]=tuple(start);pts[-1]=tuple(goal)
    return pts
   for dx,dy in [(1,0),(-1,0),(0,1),(0,-1),(1,1),(1,-1),(-1,1),(-1,-1)]:
    v=(u[0]+dx,u[1]+dy)
    if not(0<=v[0]<self.nx and 0<=v[1]<self.ny) or not self.valid(self.point(v)):continue
    if dx and dy and (not self.valid(self.point((u[0]+dx,u[1]))) or not self.valid(self.point((u[0],u[1]+dy)))):continue
    n=cost[u]+math.hypot(dx,dy)*self.res
    if n<cost.get(v,float('inf')):cost[v]=n;parent[v]=u;heapq.heappush(q,(n+dist(self.point(v),self.point(g)),v))
  return []
 def simplify(self,pts):
  if not pts:return []
  out=[pts[0]];i=0
  while i<len(pts)-1:
   j=len(pts)-1
   while j>i+1 and not self.line(pts[i],pts[j]):j-=1
   out.append(pts[j]);i=j
  return out

def inverse(vx,vy,w,cfg):
 r=cfg['wheel_radius_m'];a=sum(cfg['wheel_span_m'])/2
 return [(vx-vy-a*w)/r,(vx+vy+a*w)/r,(vx+vy-a*w)/r,(vx-vy+a*w)/r]
def forward(ws,cfg):
 r=cfg['wheel_radius_m'];a=sum(cfg['wheel_span_m'])/2
 f0,f1,f2,f3=ws
 return r*(f0+f1+f2+f3)/4,r*(-f0+f1+f2-f3)/4,r*(-f0+f1-f2+f3)/(4*a)

def reference(length,t,vmax,acc):
 ta=min(vmax/acc,math.sqrt(length/acc));vp=acc*ta;tc=max(0,(length-acc*ta*ta)/vp);total=2*ta+tc
 if t<ta:s=.5*acc*t*t
 elif t<ta+tc:s=.5*acc*ta*ta+vp*(t-ta)
 elif t<total:s=length-.5*acc*(total-t)**2
 else:s=length
 return s,total

def run(cfg,scenario='baseline',seed=7):
 rng=random.Random(seed);grid=Grid(cfg,scenario);pose=list(cfg['start_pose']);goals=cfg['goals'];plans=[];all_path=[pose[:2]]
 for g in goals:
  raw=grid.astar(all_path[-1],g['pose'][:2]);simple=grid.simplify(raw)
  plans.append({'name':g['name'],'raw':raw,'path':simple,'goal':g['pose']})
  if not simple:return {'scenario':scenario,'status':'NO_PATH','field':cfg,'obstacles':grid.obstacles,'radius_m':grid.radius,'plans':plans,'frames':[],'goals':[],'collision_count':0,'path_length_m':0,'elapsed_s':0,'seed':seed}
  all_path.extend(simple[1:])
 pids=[PID(*cfg['position_pid']),PID(*cfg['position_pid']),PID(*cfg['yaw_pid'])]
 wheel_pids=[PID(*cfg['wheel_pid']) for _ in range(4)];ws=[0.]*4;cmd=[0.,0.,0.];frames=[];records=[]
 dt=.005;t=0.;measure=pose[:];measure_t=0.;last_ctl=-1.;fault=False;fault_time=None;maxwheel=0.;maxcurrent=0.;collisions=0
 def integrate(ref,target,stage,duration=None):
  nonlocal t,measure,measure_t,last_ctl,fault,fault_time,cmd,pose,maxwheel,maxcurrent,collisions
  settled=0.;begin=t
  while t-begin<60:
   loss=scenario=='pose_loss' and t>=8.
   if t-measure_t>=.05-1e-9 and not loss:
    noise=cfg['pose_noise_m'];measure=[pose[0]+rng.gauss(0,noise),pose[1]+rng.gauss(0,noise),wrap(pose[2]+rng.gauss(0,cfg['yaw_noise_rad']))];measure_t=t
   if t-measure_t>cfg['pose_timeout_s'] and not fault:fault=True;fault_time=t
   if t-last_ctl>=.02-1e-9:
    last_ctl=t
    if fault:cmd=[0.,0.,0.]
    else:
     vx=pids[0].step(ref[0]-measure[0],.02);vy=pids[1].step(ref[1]-measure[1],.02);omega=pids[2].step(wrap(ref[2]-measure[2]),.02)
     mag=math.hypot(vx,vy)
     if mag>cfg['max_speed_mps']:vx*=cfg['max_speed_mps']/mag;vy*=cfg['max_speed_mps']/mag
     # Limit changes in the translational velocity as a vector.
     delta=[vx-cmd[0],vy-cmd[1]];n=math.hypot(*delta);lim=cfg['max_acc_mps2']*.02
     if n>lim:delta=[d*lim/n for d in delta]
     cmd[0]+=delta[0];cmd[1]+=delta[1];cmd[2]+=clip(omega-cmd[2],-cfg['max_yaw_acc_rps2']*.02,cfg['max_yaw_acc_rps2']*.02)
   cy,sy=math.cos(measure[2]),math.sin(measure[2]);desired=inverse(cy*cmd[0]+sy*cmd[1],-sy*cmd[0]+cy*cmd[1],cmd[2],cfg)
   factor=max(1,max(abs(w) for w in desired)/cfg['max_wheel_radps']);desired=[w/factor for w in desired]
   currents=[]
   for j in range(4):
    current=wheel_pids[j].step(desired[j]-ws[j],dt);currents.append(current)
    ws[j]+=dt*(cfg['motor_gain']*current-cfg['wheel_drag']*ws[j])
   bvx,bvy,bw=forward(ws,cfg);cy,sy=math.cos(pose[2]),math.sin(pose[2]);pose[0]+=dt*(cy*bvx-sy*bvy);pose[1]+=dt*(sy*bvx+cy*bvy);pose[2]=wrap(pose[2]+dt*bw)
   maxwheel=max(maxwheel,max(abs(w) for w in ws));maxcurrent=max(maxcurrent,max(abs(i) for i in currents))
   if not grid.valid(pose[:2],math.hypot(*[v/2 for v in cfg['robot_size_m']])):collisions+=1;return False
   t+=dt
   if len(frames)==0 or t-frames[-1]['t']>=.05-1e-8:
    frames.append({'t':round(t,3),'x':pose[0],'y':pose[1],'yaw':pose[2],'vx':bvx,'vy':bvy,'w':bw,'ref':ref[:],'stage':stage,'state':'POSE_TIMEOUT' if fault else 'MOVING','pose_age':t-measure_t,'wheel':ws[:],'current':currents})
   if fault and t-fault_time>1.5:return False
   if duration is not None:return True
   e=dist(pose,target);a=abs(wrap(pose[2]-target[2]))
   if e<cfg['position_tolerance_m'] and a<cfg['yaw_tolerance_rad'] and math.hypot(bvx,bvy)<.035 and abs(bw)<.05:settled+=dt
   else:settled=0
   if settled>=cfg['settle_s']:return True
  return False
 ok=True
 for plan,g in zip(plans,goals):
  begin=t;pts=plan['path']
  for a,b in zip(pts,pts[1:]):
   length=dist(a,b);local=0.;startyaw=pose[2]
   _,period=reference(length,0,cfg['max_speed_mps']*.8,cfg['max_acc_mps2']*.7)
   while local<=period+1e-8:
    s,_=reference(length,local,cfg['max_speed_mps']*.8,cfg['max_acc_mps2']*.7);ratio=s/length
    ref=[a[0]+(b[0]-a[0])*ratio,a[1]+(b[1]-a[1])*ratio,wrap(startyaw+wrap(g['pose'][2]-startyaw)*ratio)]
    if not integrate(ref,g['pose'],g['name'],duration=dt):ok=False;break
    local+=dt
   if not ok:break
   target=[b[0],b[1],g['pose'][2]]
   if not integrate(target,target,g['name']):ok=False;break
  if not ok:break
  records.append({'name':g['name'],'time_s':round(t-begin,3),'position_error_m':dist(pose,g['pose']),'yaw_error_deg':abs(math.degrees(wrap(pose[2]-g['pose'][2]))),'final_pose':pose[:]})
 return {'scenario':scenario,'status':'POSE_TIMEOUT' if fault else 'COMPLETE' if ok else 'COLLISION' if collisions else 'TIMEOUT','field':cfg,'obstacles':grid.obstacles,'radius_m':grid.radius,'plans':plans,'frames':frames,'goals':records,'collision_count':collisions,'path_length_m':sum(dist(a,b) for a,b in zip(all_path,all_path[1:])),'elapsed_s':round(t,3),'max_wheel_radps':maxwheel,'max_current_a':maxcurrent,'fault_time_s':fault_time,'final_speed_mps':math.hypot(*forward(ws,cfg)[:2]),'seed':seed}

def main():
 ap=argparse.ArgumentParser();ap.add_argument('--scenario',choices=['baseline','replan','blocked','pose_loss'],default='baseline');ap.add_argument('--seed',type=int,default=7);ap.add_argument('--config',default=str(pathlib.Path(__file__).with_name('config.json')));ap.add_argument('--output',default='results');args=ap.parse_args()
 cfg=json.loads(pathlib.Path(args.config).read_text(encoding='utf-8'));data=run(cfg,args.scenario,args.seed);out=pathlib.Path(args.output).resolve();out.mkdir(parents=True,exist_ok=True)
 (out/'result.json').write_text(json.dumps(data,ensure_ascii=False,separators=(',',':')),encoding='utf-8');write_viewer(data,out/'viewer.html')
 with (out/'trajectory.csv').open('w',newline='',encoding='utf-8-sig') as f:
  w=csv.writer(f);w.writerow(['t_s','x_m','y_m','yaw_rad','vx_body_mps','vy_body_mps','omega_radps','state','pose_age_s'])
  for p in data['frames']:w.writerow([p[k] for k in ['t','x','y','yaw','vx','vy','w','state','pose_age']])
 summary={k:data[k] for k in ['scenario','status','elapsed_s','path_length_m','collision_count','goals']};print(json.dumps(summary,ensure_ascii=False,indent=2));print('Open '+str(out/'viewer.html'))
if __name__=='__main__':main()
