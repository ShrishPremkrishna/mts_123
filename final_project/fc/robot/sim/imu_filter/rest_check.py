import sys, numpy as np
sys.path.insert(0, sys.argv[2]); import fcimu
rows=[l.strip().split(',') for l in open(sys.argv[1]).readlines()[1:]]
acc=None; ev=[]  # build gyro-driven step list
qrv=None; out=[]
G=[];A=[];AN=[];DT=[];RV=[]; tprev=None; pending_acc=None
for r in rows:
    t=float(r[0]); rep=int(r[1]); v=[float(x) for x in r[4:] if x!='']
    if rep==1: pending_acc=v[:3]
    elif rep==5: qrv=v[:4]
    elif rep==7 and qrv is not None:
        g=np.array(v[:3])-np.array(v[3:6])
        DT.append(0 if tprev is None else t-tprev); tprev=t
        G.append(g); A.append(pending_acc if pending_acc else [0,0,0]); AN.append(1 if pending_acc else 0); pending_acc=None; RV.append(qrv)
G=np.array(G);A=np.array(A);RV=np.array(RV)
print('n',len(G),'gyro hz',1/np.median(np.diff(np.cumsum(DT))[1:]),'acc mean',A[np.array(AN)==1].mean(0),'|a|',np.linalg.norm(A[np.array(AN)==1],axis=1).mean())
# accel identical-run check
Aa=A[np.array(AN)==1]; same=(np.abs(np.diff(Aa,axis=0)).sum(1)==0); run=0;mx=0
for s in same: run=run+1 if s else 0; mx=max(mx,run)
print('accel longest identical repeats',mx)
f=fcimu.Filter(); q,rate,fl=f.run(np.array(DT),G,A,np.array(AN,np.int32))
def up(qx):
    x,y,z,w=qx.T; return np.stack([2*(x*z-w*y),2*(y*z+w*x),w*w-x*x-y*y+z*z],1)
u1=up(q);u2=up(RV); u2/=np.linalg.norm(u2,axis=1)[:,None]
d=np.degrees(np.arccos(np.clip((u1*u2).sum(1),-1,1)))
k=len(d)//10
print('filter vs RV gravity: after 1s median %.3f deg  max %.3f deg'%(np.median(d[k:]),d[k:].max()))
print('mount check: RV up in sensor', u2.mean(0), ' filter', u1.mean(0))
print(f.stats())
