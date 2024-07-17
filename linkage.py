import math
from pov_utility import *
from utility import *
try:
    import pygame
except:
    print("No GUI Support!")

class Linkage:
    def __init__(self,nrN,geom=None,t=1,linearMotor=False):
        self.U=[0]*(nrN+1)
        self.F=[0]*(nrN+1)
        self.C1=[-1]*(nrN+1)
        self.C2=[-1]*(nrN+1)
        self.len1=[0.0]*(nrN+1)
        self.len2=[0.0]*(nrN+1)
        #motor
        self.U[1]=self.F[1]=1
        self.rad_motor=[1.0,1.0] if linearMotor else 1.0
        self.ctr_motor=(0.0,0.0)
        self.linearMotor=linearMotor
        #position
        self.node_position=[(0.0,0.0)]*(nrN+1)
        self.curve=[]
        #bound
        self.B=10
        #geom
        if geom is None:
            return
        assert nrN==geom.K
        self.set_rad_motor(getXLen(geom.get_d1(1,t)))
        self.set_ctr_motor(geom.get_C())
        self.node_position[1]=geom.get_x(1,t)
        for i in range(2,geom.K+1):
            if getX(geom.get_U(i))<0.5:
                self.unuse_node(i)
            else:
                self.use_node(i)
                if getX(geom.get_F(i))>0.5:
                    self.set_node_fixed(i,geom.get_x(i,t))
                else:
                    self.set_node_C1(i,geom.get_C1(i),getXLen(geom.get_d1(i,t)))
                    self.set_node_C2(i,geom.get_C2(i),getXLen(geom.get_d2(i,t)))
                    self.node_position[i]=geom.get_x(i,t)
        #linear motor setup
        if linearMotor:
            T=len(geom.xit[0])
            xss=[geom.get_x(1,t)[0] for t in range(1,T)]
            yss=[geom.get_x(1,t)[1] for t in range(1,T)]
            self.set_rad_motor([(max(xss)-min(xss))/2.0,(max(yss)-min(yss))/2.0])
            self.set_ctr_motor([(max(xss)+min(xss))/2.0,(max(yss)+min(yss))/2.0])        
    def random_transform(self, scale=1.):
        import random
        R=rot2D(random.uniform(-math.pi,math.pi)*scale)
        t=[random.uniform(-self.B,self.B)*scale,
           random.uniform(-self.B,self.B)*scale]
        self.transform(R,t)
    def transform(self, R, t):
        if isinstance(R,float) or isinstance(R,int):
            R=rot2D(R*math.pi/180.0)
        self.ctr_motor=add(mulR(R,self.ctr_motor),t)
        for i in range(len(self.F)):
            if self.F[i]==1:
                self.node_position[i]=add(mulR(R,self.node_position[i]),t)
    def set_rad_motor(self, rad):
        self.rad_motor=rad
    def set_ctr_motor(self, ctr):
        self.ctr_motor=ctr
    def use_node(self, id):
        assert id>1
        self.U[id]=1
    def unuse_node(self, id):
        assert id>1
        self.U[id]=0
        self.F[id]=0
        self.C1[id]=-1
        self.C2[id]=-1
    def set_node_fixed(self, id, pos):
        self.use_node(id)
        self.F[id]=1
        self.C1[id]=self.C2[id]=-1
        self.len1[id]=0.0
        self.len2[id]=0.0
        self.node_position[id]=pos
    def set_node_movable(self, id):
        self.use_node(id)
        self.F[id]=0
    def set_node_C1(self, id, id1, len):
        assert id>1 and id1<id
        self.set_node_movable(id)
        self.C1[id]=id1
        self.len1[id]=len
    def set_node_C2(self, id, id2, len):
        assert id>1 and id2<id
        self.set_node_movable(id)
        self.C2[id]=id2
        self.len2[id]=len
    def forward_kinematic(self, theta):
        if theta>math.pi*2:
            self.curve=[]
        self.node_position_last=[n for n in self.node_position]
        #motor node
        if hasattr(self,"linearMotor") and self.linearMotor:
            x1=self.ctr_motor[0]+math.cos(theta)*self.rad_motor[0]
            y1=self.ctr_motor[1]+math.cos(theta)*self.rad_motor[1]
        else:
            x1=self.ctr_motor[0]+math.cos(theta)*self.rad_motor
            y1=self.ctr_motor[1]+math.sin(theta)*self.rad_motor
        self.node_position[1]=(x1,y1)
        #for all other nodes
        for i in range(len(self.node_position)):
            if self.U[i]==0 or self.F[i]==1:
                continue
            #assemble
            d1=self.node_position[self.C1[i]]
            d2=self.node_position[self.C2[i]]
            d12=sub(d1,d2)
            d12Sqr=dot(d12,d12)
            len1Sqr=self.len1[i]**2
            len2Sqr=self.len2[i]**2
            coefA=d12Sqr+len1Sqr-len2Sqr
            coefB=4*d12Sqr*len1Sqr-coefA**2
            if coefB<0.:
                return 0.,False
            coefB=math.sqrt(coefB)
            xi=d1[0]+(-d12[0]*coefA+d12[1]*coefB)/(2*d12Sqr)
            yi=d1[1]+(-d12[1]*coefA-d12[0]*coefB)/(2*d12Sqr)
            self.node_position[i]=(xi,yi)
        self.curve.append(self.node_position[-1])
        return math.fmod(theta,math.pi*2),True
    def check_validity(self, ntheta=360):
        self.node_position_batch=[np.vstack((np.ones(ntheta)*n[0],np.ones(ntheta)*n[1])) for n in self.node_position]
        theta=np.linspace(0,math.pi*2,ntheta)
        #motor node
        if hasattr(self,"linearMotor") and self.linearMotor:
            x1=self.ctr_motor[0]+np.cos(theta)*self.rad_motor[0]
            y1=self.ctr_motor[1]+np.cos(theta)*self.rad_motor[1]
        else:
            x1=self.ctr_motor[0]+np.cos(theta)*self.rad_motor
            y1=self.ctr_motor[1]+np.sin(theta)*self.rad_motor
        self.node_position_batch[1] = np.vstack((x1, y1))
        # for all other nodes
        for i in range(len(self.node_position)):
            if self.U[i] == 0 or self.F[i] == 1:
                continue
            # assemble
            d1 = self.node_position_batch[self.C1[i]]
            d2 = self.node_position_batch[self.C2[i]]
            d12 = d1 - d2
            d12Sqr = np.linalg.norm(d12, axis=0) ** 2
            len1Sqr = self.len1[i] ** 2
            len2Sqr = self.len2[i] ** 2
            coefA = d12Sqr + len1Sqr - len2Sqr
            coefB = 4 * d12Sqr * len1Sqr - coefA ** 2
            if np.any(coefB < 0.):
                return False
            coefB = np.sqrt(coefB)
            xi = d1[0] + (-d12[0] * coefA + d12[1] * coefB) / (2 * d12Sqr)
            yi = d1[1] + (-d12[1] * coefA - d12[0] * coefB) / (2 * d12Sqr)
            self.node_position_batch[i] = np.vstack((xi, yi))
        return True
    def transCoordXY(self, screen, x, y):
        BX=self.B
        BY=BX*screen.get_height()/screen.get_width()
        #xx,yy
        xx=int((x+BX)/(BX*2)*screen.get_width())
        yy=screen.get_height()-int((y+BY)/(BY*2)*screen.get_height())
        return (xx,yy)
    def transCoord(self, screen, p):
        return self.transCoordXY(screen, p[0], p[1])
    def invTransCoordXY(self, screen, x, y):
        BX=self.B
        BY=BX*screen.get_height()/screen.get_width()
        #xx,yy
        xx=float(x)/float(screen.get_width())*BX*2-BX
        yy=BY-float(y)/float(screen.get_height())*BY*2
        return (xx,yy)
    def invTransCoord(self, screen, p):
        return self.invTransCoordXY(screen, p[0], p[1])
    def render_font(self, screen, text, color, crd, line=None):
        font=pygame.font.SysFont("comicsansms",72)
        text=font.render(text,True,color)
        if line is not None:
            crd=(crd[0],crd[1]+line*font.get_height())
        screen.blit(text,crd)
    def render(self, screen, psz=24, lw=12, draw_font=True):
        #rigid rod
        pygame.draw.line(screen,[0,0,0],self.transCoord(screen,self.ctr_motor),self.transCoord(screen,self.node_position[1]),lw)
        for i in range(len(self.node_position)):
            if self.U[i]==0 or self.F[i]==1:
                continue
            p=self.node_position[i]
            l1=self.node_position[self.C1[i]]
            l2=self.node_position[self.C2[i]]
            pygame.draw.line(screen,[0,0,0],self.transCoord(screen,p),self.transCoord(screen,l1),lw)
            pygame.draw.line(screen,[0,0,0],self.transCoord(screen,p),self.transCoord(screen,l2),lw)
        #curve
        for i in range(1,len(self.curve)):
            p1=self.transCoord(screen,self.curve[i-1])
            p2=self.transCoord(screen,self.curve[i])
            pygame.draw.line(screen,[0,0,255],p1,p2,lw)
        #node
        pygame.draw.circle(screen,[255,0,0],self.transCoord(screen,self.ctr_motor),psz)
        for i in range(len(self.node_position)):
            if self.U[i]==0:
                continue
            if i==1:
                color=[0,255,0]
            elif self.F[i]==1:
                color=[255,0,0]
            else: 
                color=[0,0,0]
            p=self.node_position[i]
            pt=self.transCoord(screen,p)
            pygame.draw.circle(screen,color,pt,psz)
            if draw_font:
                self.render_font(screen,('n%d'%i),color,(pt[0],pt[1]-psz*5))
    def render_pov(self, path, screen=None, thick=0.25, dist=0.15, rad=0.30, zoom=1.5, drawLink=True):
        #assign level
        level=0
        level_min=[-1 for n in self.node_position]
        level_max=[-1 for n in self.node_position]
        for i in range(1,len(self.node_position)):
            if self.U[i]==1:
                level_min[i]=level_max[i]=level
                if self.F[i]==0:
                    level_max[self.C1[i]]=level
                    level_max[self.C2[i]]=level
                level=level+1
        #structure
        objects=[]
        if drawLink:
            objects.append(addPole( self.ctr_motor,0,0,
                                    thick,dist,rad,True,False))
            objects.append(addLink( self.node_position[1],self.ctr_motor,level_min[1],
                                    thick,dist,rad))
            for i in range(1,len(self.node_position)):
                if not level_min[i]==-1:
                    objects.append(addPole( self.node_position[i],level_min[i],level_max[i],
                                            thick,dist,rad,self.F[i]==1,i==1))
                    if self.F[i]==0:
                        objects.append(addLink( self.node_position[i],self.node_position[self.C1[i]],level_min[i],
                                                thick,dist,rad))
                        objects.append(addLink( self.node_position[i],self.node_position[self.C2[i]],level_min[i],
                                                thick,dist,rad))
        #curve
        for i in range(1,len(self.curve)-1):
            objects.append(addCurve(self.curve[i],self.curve[i+1],level_max[-1],
                                    thick,dist,rad,target=False))
        #target_curve
        if hasattr(self,'target_curve'):
            for i in range(len(self.target_curve)-1):
                objects.append(addCurve(self.target_curve[i],self.target_curve[i+1],level_max[-1],
                                        thick,dist,rad,target=True))
        if hasattr(self,'target_pts'):
            for i in range(len(self.target_pts)):
                objects.append(addCurveS(self.target_pts[i],level_max[-1],
                                         thick*1.5,dist,rad))
        #floor
        objects.append(vp.Background('color','White'))
        #objects.append(vp.Box((-self.B*100,-self.B*100,0),
        #                      ( self.B*100, self.B*100,0+thick),
        #                      vp.Finish('diffuse',0.9),
        #                      vp.Pigment('color',[1,1,1]),
        #                      'no_shadow'))
        #create scene
        loc=[0.0,0.0,-self.B*zoom]
        sun=vp.LightSource(loc,'color','White')
        objects.append(sun)
        s=vp.Scene(vp.Camera( 'orthographic',
                              'angle',90,
                              'location',loc,
                              'look_at', [0.0 , 0.0 , 0.0]),
                              objects=objects,included=['colors.inc'])
        #output
        p=os.path.dirname(os.path.abspath(path))
        if not os.path.isdir(p):
            os.mkdir(p)
        if path.endswith('.pov'):
            with open(path,'w') as f:
                f.write(str(s))
        elif screen is not None:
            s.render(path,width=screen.get_width(),height=screen.get_height())#,antialiasing=0.3)
            img=pygame.image.load(path)
            screen.blit(img,(0,0))
    @staticmethod
    def createJansen(B=13):
        link=Linkage(7)
        link.B=B
        link.set_rad_motor(1.5)
        link.set_ctr_motor((0.0,0.0))
        link.set_node_fixed(2,(-3.8,-0.78))
        link.set_node_C1(3,2,4.15)
        link.set_node_C2(3,1,5.0)
        link.set_node_C1(4,2,4.01)
        link.set_node_C2(4,3,5.58)
        link.set_node_C1(5,1,6.19)
        link.set_node_C2(5,2,3.93)
        link.set_node_C1(6,5,3.67)
        link.set_node_C2(6,4,3.94)
        link.set_node_C1(7,5,4.9)
        link.set_node_C2(7,6,6.57)
        return link
    @staticmethod
    def createSimple(B=13):
        link=Linkage(3)
        link.B=B
        link.set_rad_motor(1.5)
        link.set_ctr_motor((0.0,0.0))
        link.set_node_fixed(2,(3.8,-0.78))
        link.set_node_C1(3,2,4.15)
        link.set_node_C2(3,1,6.0)
        return link
    @staticmethod
    def createFromFile(name):
        import pickle
        with open(name,'rb') as f:
            results,_,_,_,_=pickle.load(f)
        return results[0]

def main_linkage(link):
    #param
    alpha=0.0
    aaCoef=4
    spd=0.1
    sim=True
    pov=False
    #visualize
    pygame.init()
    screen=pygame.display.set_mode((512,512))
    screen_hires=pygame.Surface((screen.get_width()*aaCoef,screen.get_height()*aaCoef))
    done=False
    fid=0
    while not done:
        for event in pygame.event.get():
            if event.type==pygame.QUIT:
                    done=True

            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_SPACE:
                    sim=not sim
                elif event.key == pygame.K_LCTRL:
                    pov=not pov
                elif event.key == pygame.K_1:
                    link.random_transform()
        screen_hires.fill([255,255,255])
        if sim:
            alpha,succ=link.forward_kinematic(alpha+spd)
        if pov:
            link.render_pov('frm.png',screen_hires)
        else: link.render(screen_hires)
        pygame.transform.smoothscale(screen_hires,screen.get_size(),screen)
        pygame.display.flip()
        fid=fid+1
        
if __name__=='__main__':
    link=Linkage.createSimple()
    link.check_validity()
    main_linkage(link)
