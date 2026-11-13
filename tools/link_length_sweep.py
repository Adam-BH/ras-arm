import math, itertools
g=9.81; H=120
M=dict(elHw=0.08, elMot=0.28, j4=0.06, j5=0.06, grip=0.18)
CAP2=0.49*0.5*5.18*0.9; CAP3=0.49*0.5*5*0.95
def ik(r,zw,L1,L2):  # wrist target relative shoulder (x=r, y=zw); elbow-up; angles from vertical
    d=math.hypot(r,zw)
    if d>L1+L2-5 or d<abs(L1-L2)+5: return None
    c=(d*d-L1*L1-L2*L2)/(2*L1*L2); q3=math.acos(max(-1,min(1,c)))
    a=math.atan2(r,zw); b=math.atan2(L2*math.sin(q3),L1+L2*math.cos(q3))
    j2=a-b; return math.degrees(j2), math.degrees(q3)
def torques(L1,L2,L3,mp,j2,j3,ePos='j2'):
    D=math.radians
    u=(math.sin(D(j2)),math.cos(D(j2))); f=(math.sin(D(j2+j3)),math.cos(D(j2+j3)))
    E=(u[0]*L1,u[1]*L1); W=(E[0]+f[0]*L2,E[1]+f[1]*L2)
    mUA=1.0*L1/1000; mFA=0.83*L2/1000
    items=[(mUA,u[0]*L1/2,'ua'),(M['elHw'],E[0],'ua'),(M['elMot'],E[0] if ePos=='elbow' else 0,'ua'),
           (mFA,E[0]+f[0]*L2/2,'fa'),(M['j4'],E[0]+f[0]*30,'fa'),(M['j5'],W[0],'fa'),(M['grip'],W[0],'gr'),(mp,W[0],'gr')]
    t2=abs(sum(m*x for m,x,_ in items))*g/1000
    t3=abs(sum(m*(x-E[0]) for m,x,gr in items if gr!='ua'))*g/1000
    return t2,t3
def evaluate(L1,L2,L3=110,mp=0.05,ePos='j2',radii=(200,220,240),heights=(25,60,100,160)):
    worst2=worst3=0
    for r in radii:
        for zt in heights:   # tip height above table
            zw=zt+L3-H        # wrist height rel shoulder (gripper vertical)
            s=ik(r,zw,L1,L2)
            if s is None: return None
            j2,j3=s
            if j2<-20 or j2>110 or j3>150: return None
            t2,t3=torques(L1,L2,L3,mp,j2,j3,ePos)
            worst2=max(worst2,t2); worst3=max(worst3,t3)
    return worst2,worst3
res=[]
for L1 in range(100,301,10):
    for L2 in range(100,301,10):
        e=evaluate(L1,L2)
        if e: res.append((e[0]/CAP2,e[1]/CAP3,L1,L2,e[0],e[1]))
res.sort()
print("best by shoulder ratio (direct geared, no extra pulley):")
for r in res[:12]: print(f"L1={r[2]} L2={r[3]}  J2 {r[4]:.2f}Nm {r[0]*100:.0f}%  J3 {r[5]:.2f}Nm {r[1]*100:.0f}%")
for L in [(200,180),(150,200),(160,180),(140,210),(180,180),(120,220)]:
    e=evaluate(*L); print(L, e and f"J2 {e[0]:.2f} ({e[0]/CAP2*100:.0f}%)  J3 {e[1]:.2f} ({e[1]/CAP3*100:.0f}%)")
# extreme: 'full movement demo' reach 260, load 100g
print("stress r up to 260, load 100g:")
for L in [(res[0][2],res[0][3]),(150,200),(160,190)]:
    e=evaluate(*L,mp=0.1,radii=(180,220,260)); print(L, e and f"J2 {e[0]/CAP2*100:.0f}% J3 {e[1]/CAP3*100:.0f}%")

print("\n--- closer stations (r=150..190), direct shoulder ---")
best=[]
for L1 in range(100,261,10):
  for L2 in range(100,261,10):
    e=evaluate(L1,L2,radii=(150,170,190))
    if e: best.append((max(e[0]/CAP2,e[1]/CAP3),L1,L2,e[0]/CAP2,e[1]/CAP3))
best.sort(); 
for b in best[:6]: print(f"L1={b[1]} L2={b[2]} J2 {b[3]*100:.0f}% J3 {b[4]*100:.0f}%")

print("\n--- base inertia need vs accel (r=220 pose) ---")
def inertia(L1,L2,j2,j3,L3=110,mp=.05):
    D=math.radians;u=(math.sin(D(j2)),);f=(math.sin(D(j2+j3)),)
    E=u[0]*L1;W=E+f[0]*L2
    it=[(L1/1000,u[0]*L1/2),(M['elHw'],E),(0.83*L2/1000,E+f[0]*L2/2),(M['j4'],E+f[0]*30),(M['j5'],W),(M['grip'],W),(mp,W)]
    return sum(m*(x/1000)**2 for m,x in it)
for L in [(150,200),(160,190)]:
    j2,j3=ik(220,25+110-120,*L); I=inertia(*L,j2,j3)
    for a in (6.3,3.0,2.0): print(L,f"I={I:.4f} alpha={a}: need {I*a+0.05:.2f} Nm vs direct 0.245 -> {(I*a+0.05)/0.245*100:.0f}%")

CAP2b=CAP2*3*0.95
print("\n--- with shoulder belt x3 (kit 16T -> printed 48T), envelope r 150..260, tip 25..180, load 50g ---")
best=[]
for L1 in range(100,261,10):
  for L2 in range(100,261,10):
    e=evaluate(L1,L2,radii=(150,200,260),heights=(25,100,180))
    if e:
      e100=evaluate(L1,L2,mp=0.1,radii=(150,200,260),heights=(25,100,180))
      best.append((max(e[0]/CAP2b,e[1]/CAP3),L1,L2,e[0]/CAP2b,e[1]/CAP3,e100[1]/CAP3, e[0]/CAP2))
best.sort()
for b in best[:10]: print(f"L1={b[1]} L2={b[2]}  J2 {b[3]*100:.0f}% (direct would be {b[6]*100:.0f}%)  J3 {b[4]*100:.0f}%  (J3 @100g {b[5]*100:.0f}%)")
for L in [(150,200),(160,190),(170,180),(200,180)]:
    e=evaluate(*L,radii=(150,200,260),heights=(25,100,180)); print(L, e and f"J2 {e[0]/CAP2b*100:.0f}% J3 {e[1]/CAP3*100:.0f}%")
print("\n--- candidates table (shoulder x3, elbow x5) envelope r150-260 tip25-180 ---")
for L in [(170,150),(180,140),(190,140),(160,170),(200,180)]:
    e=evaluate(*L,radii=(150,200,260),heights=(25,100,180)); e2=evaluate(*L,mp=0.1,radii=(150,200,260),heights=(25,100,180))
    d=evaluate(*L,radii=(220,),heights=(25,60,100,160))
    print(L,f"J2 {e[0]/CAP2b*100:.0f}%/{e2[0]/CAP2b*100:.0f}%(100g)  J3 {e[1]/CAP3*100:.0f}%/{e2[1]/CAP3*100:.0f}%(100g) | task-only r=220 direct-shoulder {d[0]/CAP2*100:.0f}%" )
