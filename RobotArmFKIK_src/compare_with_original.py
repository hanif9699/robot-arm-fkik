import sys, random, json, io, os
sys.path.insert(0, sys.argv[1])
import numpy as np, RobotArmFKIK as rk
random.seed(5); out=[]
M=rk.Kinematics.RobotArmMechanism
KEEP=[M()]
out.append([rk.__doc__, rk.Kinematics.__doc__, M.__doc__, M.addSegment.__doc__, M.computeInverseKinematics.__doc__, M.computeForwardKinematics.__doc__, repr(KEEP[0])])
axes=["RotX","RotY","RotZ","TransX","TransY","TransZ","None"]
for t in range(300):
    r=M(); KEEP.append(r); n=random.randint(1,7)
    for _ in range(n):
        ax=random.choice(axes)
        if random.random()<.5: r.addSegment(ax)
        else: r.addSegment(ax, random.uniform(-20,20), random.uniform(-180,180), random.uniform(-20,20), random.uniform(-180,180))
    sys.stderr.write("t=%d n=%d %s\n"%(t,n,repr(r))); sys.stderr.flush()
    r.finaliseChain(); out.append(repr(r))
    dof=int(repr(r).split()[-1])
    q=[random.uniform(-200,200) for _ in range(dof)]
    out.append(r.computeForwardKinematics(q))
    if dof>=2:
        tgt=r.computeForwardKinematics([random.uniform(-90,90) for _ in range(dof)])
        o=np.zeros(dof); 
        # capture C-level stdout
        sys.stdout.flush(); rd,wr=os.pipe(); old=os.dup(1); os.dup2(wr,1)
        ok=r.computeInverseKinematics(tgt,[random.uniform(-90,90) for _ in range(dof)],o)
        sys.stdout.flush()
        try:
            import ctypes; ctypes.CDLL(None).fflush(None)
        except Exception: pass
        os.dup2(old,1); os.close(wr); txt=os.read(rd,100000).decode(); os.close(rd)
        out.append([ok,list(o),txt])
for bad in ["Rotx","RotQ","TransQ","foo",""]:
    try: KEEP.append(M()); KEEP[-1].addSegment(bad); out.append("noerr")
    except Exception as e: out.append([type(e).__name__,str(e)])
print(json.dumps(out))
