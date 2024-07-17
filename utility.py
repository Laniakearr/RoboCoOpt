import math,types,os,random
import numpy as np

def clearEqName():
    eqIds={}
def getX(a):
    if type(a) is tuple:
        if len(a)==2:
            return (getX(a[0]),getX(a[1]))
        else:
            return (getX(a[0]),getX(a[1]),getX(a[2]))
    else: 
        assert type(a)==float or type(a)==np.float64 or type(a)==gurobipy.Var
        if type(a)==float or type(a)==np.float64:
            return a
        else:
            try: 
                return a.X
            except: 
                return a.Start
def getXLenSqr(a):
    if type(a) is tuple:
        if len(a)==2:
            return getX(a[0])*getX(a[0])+getX(a[1])*getX(a[1])
        else:
            return getX(a[0])*getX(a[0])+getX(a[1])*getX(a[1])+getX(a[2])*getX(a[2])
    else: return getX(a)*getX(a)
def getXLen(a):
    return math.sqrt(getXLenSqr(a))
def normalize(a):
    assert type(a) is tuple
    if len(a)==2:
        return (a[0]/getXLen(a),a[1]/getXLen(a))
    else:
        return (a[0]/getXLen(a),a[1]/getXLen(a),a[2]/getXLen(a))
def getLenSqr(a):
    if type(a) is tuple:
        if len(a)==2:
            return a[0]*a[0]+a[1]*a[1]
        else:
            return a[0]*a[0]+a[1]*a[1]+a[2]*a[2]
    else: return a*a
def add(a,b):
    if type(a) is tuple:
        if len(a)==2:
            return (a[0]+b[0],a[1]+b[1])
        else:
            return (a[0]+b[0],a[1]+b[1],a[2]+b[2])
    else: return a+b
def sub(a,b):
    if type(a) is tuple:
        if len(a)==2:
            return (a[0]-b[0],a[1]-b[1])
        else:
            return (a[0]-b[0],a[1]-b[1],a[2]-b[2])
    else: return a-b
def mul(a,b):
    if type(a) is tuple:
        if len(a)==2:
            return (a[0]*b,a[1]*b)
        else:
            return (a[0]*b,a[1]*b,a[2]*b)
    else: return a*b
def mulR(R,a):
    if len(a)==2:
        x=R[0][0]*a[0]+R[0][1]*a[1]
        y=R[1][0]*a[0]+R[1][1]*a[1]
        return (x,y)
    else:
        x=R[0][0]*a[0]+R[0][1]*a[1]+R[0][2]*a[2]
        y=R[1][0]*a[0]+R[1][1]*a[1]+R[1][2]*a[2]
        z=R[2][0]*a[0]+R[2][1]*a[1]+R[2][2]*a[2]
        return (x,y,z)
def transpose(R):
    return [(R[0][0],R[1][0]),(R[0][1],R[1][1])]
def perp(a):
    return (a[1],-a[0])
def dot(a,b):
    if type(a) is tuple:
        if len(a)==2:
            return a[0]*b[0]+a[1]*b[1]
        else:
            return a[0]*b[0]+a[1]*b[1]+a[2]*b[2]
    else: return a*b
def cross(a,b):
    assert len(a)==3
    return (a[1]*b[2]-a[2]*b[1],    \
            a[2]*b[0]-a[0]*b[2],    \
            a[0]*b[1]-a[1]*b[0])
def rot2D(theta):
    return [(math.cos(theta),-math.sin(theta)),
            (math.sin(theta), math.cos(theta))]
def clog(d,d0,coef):
    if d>d0:
        return 0.,0.
    valLog=math.log(d/d0)
    valLogC=valLog*(d-d0)
    relD=(d-d0)/d
    D=-(2*valLogC+(d-d0)*relD)*coef
    return -valLogC*(d-d0)*coef,D
def squareIndex(vars,coefs,C=1.):
    varsRet=[]
    coefsRet=[]
    for v1,c1 in zip(vars,coefs):
        for v2,c2 in zip(vars,coefs):
            varsRet.append((v1,v2))
            coefsRet.append(c1*c2*C)
    return varsRet,coefsRet
def areaIndex(a,b,C=1.):
    #[a[1],-a[0]]*[b[0],b[1]]=a[1]*b[0]-a[0]*b[1]
    return [(a[1],b[0]),(a[0],b[1])],[1.,-1.]

