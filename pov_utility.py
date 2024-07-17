try:
    import vapory as vp
except:
    print("No Povray Support!")
    exit()

def addPole(p,l0,l1,t,d,r,f,m):
    if m:
        pg=vp.Pigment('color',[0,1,0])
    elif f:
        pg=vp.Pigment('color',[1,0,0])
    else:
        pg=vp.Pigment('color',[0.1,0.1,0.1])
    fh=vp.Finish('phong',0.1)
    return vp.Cylinder((p[0],p[1],-l0*d  ),
                       (p[0],p[1],-l1*d-t*2),r,
                       fh,pg)
def addLink(p0,p1,l,t,d,r):
    pg=vp.Pigment('color',[0.5,0.5,0.5])
    fh=vp.Finish('phong',0.3)
    c0=vp.Cylinder( (p0[0],p0[1],-l*d  ),
                    (p0[0],p0[1],-l*d-t),w,
                    fh,pg)
    c1=vp.Cylinder( (p1[0],p1[1],-l*d  ),
                    (p1[0],p1[1],-l*d-t),w,
                    fh,pg)
    p01=sub(p1,p0)
    d01=getXLen(p01)
    angle=math.atan2(p01[1],p01[0])*180/math.pi
    b=vp.Box((0  ,-w,-l*d  ),
             (d01, w,-l*d-t),
             fh,pg,
             'rotate',(0,0,angle),
             'translate',(p0[0],p0[1],0))
    return vp.Union(b,c0,c1)
def addLink(p0,p1,l,t,d,r):
    pg=vp.Pigment('color',[0.5,0.5,0.5])
    fh=vp.Finish('phong',0.3)
    c= vp.Cylinder( (p0[0],p0[1],-l*d-t/2),
                    (p1[0],p1[1],-l*d-t/2),t/2,
                    fh,pg)
    return c
def addCurve(p0,p1,l,t,d,r,target):
    pg=vp.Pigment('color',[0,0,1] if not target else [1,1,0])#,"transmit",0.7)
    fh=vp.Finish('phong',0.3)
    c=vp.Cylinder((p0[0],p0[1],-l*d-t/2),
                  (p1[0],p1[1],-l*d-t/2),t/2,
                  fh,pg)
    s=vp.Sphere((p1[0],p1[1],-l*d-t/2),t/2,
                fh,pg)
    return vp.Union(c,s)
def addCurveS(p,l,t,d,r):
    pg=vp.Pigment('color',[0,1,0])#,"transmit",0.7)
    fh=vp.Finish('phong',0.3)
    s=vp.Sphere((p[0],p[1],-l*d-t/2),t/2,
                fh,pg)
    return s