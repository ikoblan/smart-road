#!/usr/bin/env python3
"""Independent Python check of the mathematical model, NOT a Rust build/test.
Optional: Python 3.10+ and NumPy. The shipped application is entirely Rust.
Run: python3 validation/reference_check.py geometry|pairs|traffic|default
"""
import math
import sys
from dataclasses import dataclass
from functools import lru_cache
import numpy as np

DT=1/120; R=math.sqrt(5); REQUIRED=2*R+1; ENTRY=60+R-18
GATE=ENTRY-6.5; SPEED=[4.,10.,6.]; INNER=[3*math.pi/2,36.,21*math.pi/2]
EXIT=[ENTRY+x for x in INNER]; RELEASE=[x+6.5 for x in EXIT]; LENGTH=[x+ENTRY for x in EXIT]
MASK=(1<<64)-1

def pose(route,s):
    q,t=divmod(route,3); offset=[15,9,3][t]; s=max(0.,min(LENGTH[t],s))
    if s<=ENTRY: x,y=-offset,-60-R+s
    elif s<EXIT[t]:
        u=s-ENTRY
        if t==1: x,y=-offset,-18+u
        elif t==0: a=u/3; x,y=-18+3*math.cos(a),-18+3*math.sin(a)
        else: a=math.pi-u/21; x,y=18+21*math.cos(a),-18+21*math.sin(a)
    else:
        u=s-EXIT[t]
        if t==0: x,y=-18-u,-offset
        elif t==1: x,y=-offset,18+u
        else: x,y=18+u,offset
    return ((x,y),(-y,x),(-x,-y),(y,-x))[q]

def polyline(route,step=.5):
    t=route%3; points=[pose(route,GATE)]
    bounds=[GATE,ENTRY,EXIT[t],RELEASE[t]]
    for a,b in zip(bounds,bounds[1:]):
        n=math.ceil((b-a)/step)
        points.extend(pose(route,a+(b-a)*k/n) for k in range(1,n+1))
    return np.array(points)

def cross(a,b): return a[...,0]*b[...,1]-a[...,1]*b[...,0]
def point_seg(p,a,b):
    d=b-a; denom=np.sum(d*d,axis=-1)
    u=np.clip(np.sum((p-a)*d,axis=-1)/denom,0,1)
    return np.linalg.norm(p-a-u[...,None]*d,axis=-1)
def line_distance(p,q):
    a,b=p[:-1,None,:],p[1:,None,:]; c,d=q[None,:-1,:],q[None,1:,:]
    den=cross(b-a,d-c); nonzero=np.abs(den)>1e-12; safe=np.where(nonzero,den,1.)
    u=cross(c-a,d-c)/safe; v=cross(c-a,b-a)/safe
    if np.any(nonzero & (u>=0)&(u<=1)&(v>=0)&(v<=1)): return 0.
    return min(np.min(point_seg(a,c,d)),np.min(point_seg(b,c,d)),np.min(point_seg(c,a,b)),np.min(point_seg(d,a,b)))

@lru_cache(None)
def matrix():
    lines=[polyline(i) for i in range(12)]
    m=np.eye(12,dtype=bool)
    for i in range(12):
        for j in range(i+1,12): m[i,j]=m[j,i]=line_distance(lines[i],lines[j])<=REQUIRED+.1
    return m

def stopping(room): return 0. if room<=0 else 6*(math.sqrt(DT*DT+2*room/6)-DT)
@dataclass
class Car:
    id:int; route:int; s:float; v:float; detected:float
    admitted:bool=False; ticket:int=0; elapsed:float=0.

@lru_cache(None)
def pair_indices(n): return np.triu_indices(n,1)

class Sim:
    def __init__(self,seed=0):
        self.cars=[]; self.pending=[0]*12; self.cursor=[0]*4; self.nextspawn=[0]*4
        self.tick=0; self.id=1; self.ticket=1; self.completed=0; self.hard=0; self.minimum=math.inf
        self.peakreservations=0; self.maxv=None; self.minv=None; self.times=[]; self.seed=seed
        self.auto=False; self.nextauto=0; self.auto_interval=2.5; self.m=matrix()
    def rand(self,bound):
        self.seed=(self.seed+0x9e3779b97f4a7c15)&MASK
        z=self.seed; z=((z^(z>>30))*0xbf58476d1ce4e5b9)&MASK; z=((z^(z>>27))*0x94d049bb133111eb)&MASK
        return (z^(z>>31))%bound
    def observe(self,v):
        self.maxv=v if self.maxv is None else max(self.maxv,v)
        self.minv=v if self.minv is None else min(self.minv,v)
    def step(self):
        now=self.tick*DT
        if self.auto and self.tick>=self.nextauto:
            self.pending[3*self.rand(4)+self.rand(3)]+=1; self.nextauto=self.tick+round(self.auto_interval/DT)
        for a in range(4):
            if self.tick<self.nextspawn[a]: continue
            for off in range(3):
                turn=(self.cursor[a]+off)%3; route=3*a+turn
                if self.pending[route]==0: continue
                nearest=min((c.s for c in self.cars if c.route==route),default=math.inf)
                if nearest<8.1: continue
                speed=min(12,stopping(nearest-8.05)) if math.isfinite(nearest) else 12.
                self.cars.append(Car(self.id,route,0.,speed,now)); self.id+=1; self.pending[route]-=1
                self.cursor[a]=(turn+1)%3; self.nextspawn[a]=self.tick+30; self.observe(speed); break
        heads={}
        for c in self.cars:
            if not c.admitted and (c.route not in heads or c.s>heads[c.route].s): heads[c.route]=c
        ready=sorted((c for c in heads.values() if c.s>=GATE-10),key=lambda c:c.id)
        for c in ready:
            if not c.ticket: c.ticket=self.ticket; self.ticket+=1
        busy=[c.route for c in self.cars if c.admitted and c.s<RELEASE[c.route%3]]; oldest_blocked=None
        for c in sorted(ready,key=lambda c:c.ticket):
            unavailable=any(r!=c.route and self.m[c.route,r] for r in busy) or (oldest_blocked is not None and self.m[c.route,oldest_blocked])
            if unavailable:
                if oldest_blocked is None: oldest_blocked=c.route
            else: c.admitted=True; busy.append(c.route)
        self.peakreservations=max(self.peakreservations,len(busy))
        old=np.array([pose(c.route,c.s) for c in self.cars]); changes=[]
        for c in self.cars:
            t=c.route%3
            target=12. if c.s>=EXIT[t] else min(12.,math.sqrt(SPEED[t]**2+12*max(ENTRY-c.s-.05,0)))
            room=max(GATE-.05-c.s,0) if not c.admitted else math.inf
            for lead in self.cars:
                if lead.route==c.route and lead.s>c.s: room=min(room,max(lead.s-c.s-8.05,0))
            if math.isfinite(room): target=min(target,stopping(max(room-.05,0)))
            v=min(c.v+3.5*DT,target) if target>c.v else max(c.v-6*DT,target)
            ds=min(v*DT,max(room,0)); actual=ds/DT
            self.hard+=actual+6*DT+1e-8<c.v
            changes.append((ds,actual))
        for c,(ds,v) in zip(self.cars,changes):
            remain=LENGTH[c.route%3]-c.s
            c.s+=ds; c.v=v; self.observe(v)
            c.elapsed=now+DT-c.detected if ds<=0 or remain>ds else now+remain/ds*DT-c.detected
        n=len(self.cars)
        if n>=2:
            current=np.array([pose(c.route,c.s) for c in self.cars]); i,j=pair_indices(n)
            start=old[i]-old[j]; move=(current[i]-old[i])-(current[j]-old[j])
            den=np.sum(move*move,axis=1); den_safe=np.where(den>1e-20,den,1)
            t=np.where(den>1e-20,np.clip(-np.sum(start*move,axis=1)/den_safe,0,1),0)
            dist=np.linalg.norm(start+move*t[:,None],axis=1)
            ds=np.array([x[0] for x in changes]); gap=dist-2*R-(ds[i]**2+ds[j]**2)/24
            self.minimum=min(self.minimum,float(np.min(gap)))
            assert np.min(gap)>1, f"close call at {now:.6f}s: {np.min(gap):.9f}m"
        remaining=[]
        for c in self.cars:
            if c.s>=LENGTH[c.route%3]: self.completed+=1; self.times.append(c.elapsed)
            else: remaining.append(c)
        self.cars=remaining; self.tick+=1
    def drain(self,seconds=600):
        self.auto=False
        for _ in range(round(seconds/DT)):
            if not self.cars and not any(self.pending): return
            self.step()
        raise AssertionError('did not drain')

def geometry_checks():
    m=matrix(); assert np.array_equal(m,m.T); assert m[2,4]
    assert not any(m[i,j] for i in [0,3,6,9] for j in [0,3,6,9] if i!=j)
    for route in range(12):
        for boundary in [ENTRY,EXIT[route%3]]:
            assert math.dist(pose(route,boundary-1e-6),pose(route,boundary+1e-6))<3e-6
        # Numerically check the tightest same-route chord at 8 m arc headway.
        minimum=min(math.dist(pose(route,s),pose(route,s+8)) for s in np.linspace(0,LENGTH[route%3]-8,6000))
        assert minimum>REQUIRED
    for room in [0.,.001,.01,.1,1.,10.,100.]:
        v=stopping(room); assert v*DT+v*v/12<=room+1e-9
    # A denser, independent polyline check of every compatible pair.
    dense=[polyline(i,.10) for i in range(12)]
    min_compatible=math.inf
    for i in range(12):
        for j in range(i+1,12):
            if not m[i,j]:
                d=line_distance(dense[i],dense[j]); min_compatible=min(min_compatible,d)
                assert d>REQUIRED+.002
    print('REFERENCE GEOMETRY: PASS')
    print(f'Compatible unordered distinct-route pairs: {sum(not m[i,j] for i in range(12) for j in range(i+1,12))}/66')
    print(f'Minimum compatible-route centre clearance: {min_compatible:.6f} m')
    print(f'Required centre clearance: {REQUIRED:.6f} m')
    print('Conflict matrix (1 = conflict), N r/s/l; E r/s/l; S r/s/l; W r/s/l:')
    print('\n'.join(''.join('1' if x else '0' for x in row) for row in m))

def pairs_checks():
    minimum=math.inf; longest=0
    for a in range(12):
        for b in range(12):
            s=Sim(); s.pending[a]+=1; s.pending[b]+=1; s.drain(90)
            assert s.completed==2
            assert s.hard==0,(a,b,s.hard)
            minimum=min(minimum,s.minimum); longest=max(longest,s.tick*DT)
    s=Sim()
    for r in [0,3,6,9]: s.pending[r]+=1
    s.drain(); assert s.peakreservations==4
    print(f'REFERENCE PAIRS: PASS; 144/144 pairs drained; minimum gap {minimum:.6f} m; longest drain {longest:.3f} s; hard overrides 0')
    print('REFERENCE FOUR RIGHT TURNS: PASS; four concurrent reservations')

def traffic_checks():
    s=Sim(); s.pending=[3]*12; s.drain(360); assert s.completed==36; assert s.hard==0
    print(f'REFERENCE 36-VEHICLE BURST: PASS; drain time {s.tick*DT:.3f} s; minimum gap {s.minimum:.6f} m')
    for seed in [0,1,7,42,2026]:
        s=Sim(seed); s.auto=True; s.auto_interval=1.25
        for _ in range(round(120/DT)): s.step()
        requested=s.id-1+sum(s.pending); s.drain(600)
        assert s.completed==requested
        assert s.hard==0,(seed,s.hard)
        print(f'REFERENCE TRAFFIC seed={seed}: PASS; completed={s.completed}; min_gap={s.minimum:.6f} m; drain_time={s.tick*DT:.3f} s')

def default_checks():
    for seed in [0,1,7,42,2026]:
        s=Sim(seed); s.auto=True
        for _ in range(round(60/DT)): s.step()
        assert s.id-1+sum(s.pending)==24
        print(f'REFERENCE DEFAULT seed={seed}: completed={s.completed}; active={len(s.cars)}; pending={sum(s.pending)}; min_gap={s.minimum:.6f} m')
    print('REFERENCE DEFAULT RANDOM MODE: PASS; 24 requests per simulated minute')

if __name__=='__main__':
    mode=sys.argv[1] if len(sys.argv)>1 else 'geometry'
    {'geometry':geometry_checks,'pairs':pairs_checks,'traffic':traffic_checks,'default':default_checks}[mode]()
    print('This independently checks equations/model behavior. It does NOT compile or execute the Rust program.')
