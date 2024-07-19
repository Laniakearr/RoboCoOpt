from Box2D import (b2DrawExtended,b2PolygonShape,b2CircleShape,b2FixtureDef,b2World,b2Vec2,b2AABB)
from time import time
from linkage import *
import shutil

class fwSettings:
    hz = 360.0
    velocityIterations = 500
    positionIterations = 500

    enableWarmStarting = True
    enableContinuous = True
    enableSubStepping = False
    
    motorOn = False
    motorSpeed = 0.1
    friction = 0.25
    torque = 8000
    
    densityBody = 1
    densityLeg = 1
    rad = 0.2
    res = 8
    rot = 0.0
    
    #povray 
    thickZ=1
    thick=0.75
    lightDist=16.
    cameraDist=30.
    cameraDistOrtho=50.

class PygameDraw(b2DrawExtended):
    """
    This debug draw class accepts callbacks from Box2D (which specifies what to
    draw) and handles all of the rendering.

    If you are writing your own game, you likely will not want to use debug
    drawing.  Debug drawing, as its name implies, is for debugging.
    """
    surface = None
    axisScale = 10.0

    def __init__(self, **kwargs):
        b2DrawExtended.__init__(self, **kwargs)
        self.flipX = False
        self.flipY = True
        self.convertVertices = True

    def DrawPoint(self, p, size, color):
        """
        Draw a single point at point p given a pixel size and color.
        """
        self.DrawCircle(p, size / self.zoom, color, drawwidth=0)

    def DrawAABB(self, aabb, color):
        """
        Draw a wireframe around the AABB with the given color.
        """
        points = [(aabb.lowerBound.x, aabb.lowerBound.y),
                  (aabb.upperBound.x, aabb.lowerBound.y),
                  (aabb.upperBound.x, aabb.upperBound.y),
                  (aabb.lowerBound.x, aabb.upperBound.y)]

        pygame.draw.aalines(self.surface, color, True, points)

    def DrawSegment(self, p1, p2, color):
        """
        Draw the line segment from p1-p2 with the specified color.
        """
        pygame.draw.aaline(self.surface, color.bytes, p1, p2)
        return

    def DrawTransform(self, xf):
        """
        Draw the transform xf on the screen
        """
        p1 = xf.position
        axisScale = 0.4
        p2 = self.to_screen(p1 + axisScale * xf.R.x_axis)
        p3 = self.to_screen(p1 + axisScale * xf.R.y_axis)
        p1 = self.to_screen(p1)

        pygame.draw.aaline(self.surface, (255,0,0), p1, p2)
        pygame.draw.aaline(self.surface, (0,255,0), p1, p3)

    def DrawCircle(self, center, radius, color, drawwidth=1):
        """
        Draw a wireframe circle given the center, radius, axis of orientation
        and color.
        """
        radius *= self.zoom
        if radius < 1:
            radius = 1
        else:
            radius = int(radius)

        pygame.draw.circle(self.surface, color.bytes, center, radius, drawwidth)

    def DrawSolidCircle(self, center, radius, axis, color):
        """
        Draw a solid circle given the center, radius, axis of orientation and
        color.
        """
        radius *= self.zoom
        if radius < 1:
            radius = 1
        else:
            radius = int(radius)

        self.draw_circle_alpha(self.surface, (color/2).bytes+[127], center, radius)
        pygame.draw.circle(self.surface, color.bytes, center, radius, 1)
        pygame.draw.aaline(self.surface, (255,0,0), center, (center[0] - radius * axis[0], center[1] + radius * axis[1]))

    def draw_circle_alpha(self, surface, color, center, radius):
        target_rect = pygame.Rect(center, (0, 0)).inflate((radius * 2, radius * 2))
        shape_surf = pygame.Surface(target_rect.size, pygame.SRCALPHA)
        pygame.draw.circle(shape_surf, color, (radius, radius), radius)
        surface.blit(shape_surf, target_rect)

    def DrawPolygon(self, vertices, color):
        """
        Draw a wireframe polygon given the screen vertices with the specified color.
        """
        if not vertices:
            return

        if len(vertices) == 2:
            pygame.draw.aaline(self.surface, color.bytes, vertices[0], vertices[1])
        else: 
            pygame.draw.polygon(self.surface, color.bytes, vertices, 1)

    def DrawSolidPolygon(self, vertices, color):
        """
        Draw a filled polygon given the screen vertices with the specified color.
        """
        if not vertices:
            return

        if len(vertices) == 2:
            pygame.draw.aaline(self.surface, color.bytes, vertices[0], vertices[1])
        else:
            self.draw_polygon_alpha(self.surface, (color/2).bytes + [127], vertices)
            pygame.draw.polygon(self.surface, color.bytes, vertices, 1)

    def draw_polygon_alpha(self, surface, color, points):
        lx, ly = zip(*points)
        min_x, min_y, max_x, max_y = min(lx), min(ly), max(lx), max(ly)
        target_rect = pygame.Rect(min_x, min_y, max_x - min_x, max_y - min_y)
        shape_surf = pygame.Surface(target_rect.size, pygame.SRCALPHA)
        pygame.draw.polygon(shape_surf, color, [(x - min_x, y - min_y) for x, y in points])
        surface.blit(shape_surf, target_rect)

class LinkagePhysics:
    def __init__(self, link, rot=0., settings=fwSettings()):
        self.settings=settings
        self.link=link
        self.R=rot2D(self.settings.rot*math.pi/180.0)
        
        self.world=b2World()
        self.renderer=PygameDraw()
        self.world.renderer=self.renderer
        self.renderer.flags={'drawShapes': True, 
                             'drawPairs': True, 
                             'drawJoints': True, 
                             'drawAABBs': False, 
                             'drawCOMs': True, 
                             'convertVertices': True}
    def create_floor(self, x=200, y=10, offy=0.):
        if hasattr(self,"ground"):
            self.world.DestroyBody(self.ground)
        self.ground=self.world.CreateStaticBody(
            shapes=[
                b2PolygonShape(vertices=[(-x  ,0+offy),(x  ,0+offy),(x  ,-y+offy),(-x  ,-y+offy)]),
                b2PolygonShape(vertices=[(-x-y,y+offy),(-x ,y+offy),(-x ,-y+offy),(-x-y,-y+offy)]),
                b2PolygonShape(vertices=[(x   ,y+offy),(x+y,y+offy),(x+y,-y+offy),(x   ,-y+offy)]),
            ])
        self.floor_offsety=offy
        self.floorx=x
        self.floory=y
    def create_torso(self, minBB, maxBB, offset=(0.,0.), fix_test=False):
        # The chassis
        if hasattr(self,"chassis"):
            self.world.DestroyBody(self.chassis)
        self.box=((maxBB[0]-minBB[0])/2,(maxBB[1]-minBB[1])/2)
        self.offset=offset
        chassis_fixture = b2FixtureDef(
            shape=b2PolygonShape(box=self.box),
            density=self.settings.densityBody,
            friction=self.settings.friction,
            groupIndex=-1)
        self.chassis = self.world.CreateDynamicBody(
            fixtures=chassis_fixture,
            position=self.offset)
        #fix robot to ground for testing
        if fix_test and hasattr(self,"ground"):
            self.motorJoint = self.world.CreateRevoluteJoint(
                bodyA=self.ground,
                bodyB=self.chassis,
                anchor=self.offset)
    def ensure_anchor_in_chassis(self, anchor):
        if not hasattr(self,'boxExt'):
            self.boxExt=self.box
        if abs(anchor[0])<self.boxExt[0] and abs(anchor[1])<self.boxExt[1]:
            return
        self.boxExt=(   max(abs(anchor[0]),self.boxExt[0]),
                        max(abs(anchor[1]),self.boxExt[1]))
        chassis_fixture = b2FixtureDef(
            shape=b2PolygonShape(box=self.boxExt),
            density=self.settings.densityBody,
            friction=self.settings.friction,
            groupIndex=-1)
        self.chassis.DestroyFixture(self.chassis.fixtures[0])
        self.chassis.CreateFixture(chassis_fixture)
    def create_legs(self, s, sep, rad_motor, mode, nrLeg):
        if not hasattr(self,"motorJoints"):
            self.motorJoints=[]
        # Chassis wheel front
        assert hasattr(self,"chassis")
        if rad_motor is None:
            rad_motor=self.link.rad_motor
        self.offsetCtr=b2Vec2(-sep,0)
        wheel_fixture = b2FixtureDef(
            shape=b2CircleShape(radius=rad_motor),
            density=self.settings.densityBody,
            friction=self.settings.friction,
            groupIndex=-1)
        self.wheel = self.world.CreateDynamicBody(
            fixtures=wheel_fixture,
            position=self.offsetCtr+self.offset)
        # add a joint between the chassis wheel and the chassis itself
        self.motorJoint = self.world.CreateRevoluteJoint(
            bodyA=self.wheel,
            bodyB=self.chassis,
            anchor=self.offsetCtr+self.offset,
            collideConnected=False,
            motorSpeed=self.settings.motorSpeed,
            maxMotorTorque=self.settings.torque,
            enableMotor=self.settings.motorOn)
        self.motorJoints.append(self.motorJoint)
        # add leg
        if mode=="symmetric_back_leg":
            s=1-s
            dtheta=math.pi*(1./nrLeg+1.)
            mode="symmetric_leg"
        elif mode=="reflected_back_leg":
            s=1-s
            dtheta=math.pi*(1./nrLeg+1.)
            mode="reflected_leg"
        else: dtheta=0.
        if mode=="symmetric_leg":
            offZP=self.settings.rad+self.settings.thickZ
            offZN=self.settings.rad+self.settings.thickZ
            for i in range(nrLeg):
                if i%2==0:
                    offZP=self.create_link_leg_as_rod(s, math.pi*i/nrLeg, offZP, zss=[1.])
                else:
                    offZN=self.create_link_leg_as_rod(s, math.pi*i/nrLeg, offZN, zss=[-1.])
        elif mode=="reflected_leg":
            offZ=self.settings.rad+self.settings.thickZ
            for i in range(nrLeg):
                offZ=self.create_link_leg_as_rod(s, math.pi*i/nrLeg, offZ, zss=[-1.,1.])
    def create_link_leg_as_rod(self, s, deg, offZ, zss):
        if not hasattr(self.wheel,"userData") or self.wheel.userData is None:
            self.wheel.userData=[]
        self.wheel.userData+=[(offZ*zSgn) for zSgn in zss]
        #offZ+=self.settings.rad*2
        #create bodies
        self.link.forward_kinematic(deg)
        self.anchor=[None for i in range(len(self.link.U))]
        self.offZ=[None for i in range(len(self.link.U))]
        self.C1s=[None for i in range(len(self.link.U))]
        self.C2s=[None for i in range(len(self.link.U))]
        self.rel=[None for i in range(len(self.link.U))]
        for i in range(1,len(self.link.U)):
            if self.link.U[i]==1:
                np=self.link.node_position[i]
                n0=self.link.ctr_motor
                self.rel[i]=mulR(self.R,sub(np,n0))
                if s>0.5:
                    self.rel[i]=(-self.rel[i][0],self.rel[i][1])
                if self.link.F[i]==1 and i>1:
                    self.anchor[i]=self.offsetCtr + self.offset + self.rel[i]
                    self.offZ[i]=0
                elif self.link.U[i]==1:
                    self.anchor[i]=self.offsetCtr + self.offset + self.rel[i]
                    self.offZ[i]=offZ
                    if i>1:
                        #leg C1
                        shape, ctr = self.create_capsule(self.anchor[i],self.anchor[self.link.C1[i]])
                        leg_fixture = b2FixtureDef(
                            shape=shape,
                            density=self.settings.densityLeg,
                            friction=self.settings.friction,
                            groupIndex=-1)
                        self.C1s[i]=self.world.CreateDynamicBody(
                            fixtures=leg_fixture,
                            position=ctr)
                        #set offZ
                        self.C1s[i].userData=[(offZ*zSgn,sub(self.anchor[self.link.C1[i]],ctr),self.offZ[self.link.C1[i]]*zSgn) for zSgn in zss]
                        #connect
                        if self.link.C1[i]==1:
                            self.world.CreateRevoluteJoint(
                                bodyA=self.C1s[i],
                                bodyB=self.wheel,
                                anchor=self.anchor[1])
                        elif self.link.F[self.link.C1[i]]==1:
                            self.world.CreateRevoluteJoint(
                                bodyA=self.C1s[i],
                                bodyB=self.chassis,
                                anchor=self.anchor[self.link.C1[i]])
                            self.ensure_anchor_in_chassis(self.anchor[self.link.C1[i]])
                        else:
                            self.world.CreateRevoluteJoint(
                                bodyA=self.C1s[i],
                                bodyB=self.C1s[self.link.C1[i]],
                                anchor=self.anchor[self.link.C1[i]])
                        #leg C2
                        shape, ctr = self.create_capsule(self.anchor[i],self.anchor[self.link.C2[i]])
                        leg_fixture = b2FixtureDef(
                            shape=shape,
                            density=self.settings.densityLeg,
                            friction=self.settings.friction,
                            groupIndex=-1)
                        self.C2s[i]=self.world.CreateDynamicBody(
                            fixtures=leg_fixture,
                            position=ctr)
                        #set offZ
                        self.C2s[i].userData=[(offZ*zSgn,sub(self.anchor[self.link.C2[i]],ctr),self.offZ[self.link.C2[i]]*zSgn) for zSgn in zss]
                        #connect
                        if self.link.C2[i]==1:
                            self.world.CreateRevoluteJoint(
                                bodyA=self.C2s[i],
                                bodyB=self.wheel,
                                anchor=self.anchor[1])
                        elif self.link.F[self.link.C2[i]]==1:
                            self.world.CreateRevoluteJoint(
                                bodyA=self.C2s[i],
                                bodyB=self.chassis,
                                anchor=self.anchor[self.link.C2[i]])
                            self.ensure_anchor_in_chassis(self.anchor[self.link.C2[i]])
                        else:
                            self.world.CreateRevoluteJoint(
                                bodyA=self.C2s[i],
                                bodyB=self.C2s[self.link.C2[i]],
                                anchor=self.anchor[self.link.C2[i]])
                        #constraint legs
                        self.world.CreateRevoluteJoint(
                            bodyA=self.C1s[i],
                            bodyB=self.C2s[i],
                            anchor=self.anchor[i])
                    offZ+=self.settings.rad*2
        return offZ
    def create_capsule(self, v1, v2):
        #create v1 vertices
        d12=v1-v2
        d12=normalize((d12[0],d12[1]))
        d12Orth=normalize((-d12[1],d12[0]))
        vertices=[]
        for i in range(self.settings.res):
            ang=math.pi*i/(self.settings.res-1)
            x=mul(d12Orth,math.cos(ang)*self.settings.rad)
            y=mul(d12,math.sin(ang)*self.settings.rad)
            vertices.append(b2Vec2(add(add(x,y),v1)))
        #create v2 vertices
        d21=v2-v1
        d21=normalize((d21[0],d21[1]))
        d21Orth=normalize((-d21[1],d21[0]))
        for i in range(self.settings.res):
            ang=math.pi*(self.settings.res-1-i)/(self.settings.res-1)
            x=mul(d21Orth,math.cos(ang)*self.settings.rad)
            y=mul(d21,math.sin(ang)*self.settings.rad)
            vertices.append(b2Vec2(add(add(x,y),v2)))
        #ctr
        ctr=(v1+v2)/2
        for i in range(len(vertices)):
            vertices[i]-=ctr
        return b2PolygonShape(vertices=vertices),ctr
    def center_view(self,screen):
        x=self.chassis.position.x*self.get_zoom()-screen.get_width()/2
        y=self.chassis.position.y*self.get_zoom()-screen.get_height()/2
        self.set_offset((x,y))
    def get_offset(self):
        return self.renderer.offset
    def set_offset(self,offset):
        self.renderer.offset=offset
    def get_zoom(self):
        return self.renderer.zoom
    def set_zoom(self,zoom):
        self.renderer.zoom=zoom
    def simulate(self, batch=60, y0=None, y_error_bound=None, a_error_bound=None):
        if self.settings.hz > 0.0:
            timeStep = 1.0 / self.settings.hz
        else: timeStep = 0.0
            
        self.world.warmStarting = self.settings.enableWarmStarting
        self.world.continuousPhysics = self.settings.enableContinuous
        self.world.subStepping = self.settings.enableSubStepping
        for joint in self.motorJoints:
            joint.motorEnabled=self.settings.motorOn

        diffY=0
        diffA=0
        for i in range(batch):
            self.world.Step(timeStep, self.settings.velocityIterations, self.settings.positionIterations)
            self.world.ClearForces()
            #diffY
            if y0 is not None:
                diffY=max(diffY,abs(self.chassis.position.y-y0))
                if y_error_bound is not None and diffY>y_error_bound:
                    return diffY,diffA
            #diffA
            diffA=max(diffA,abs(self.chassis.angle))
            if a_error_bound is not None and diffA>=a_error_bound:
                return diffY,diffA
        return diffY,diffA
    def eval_performance(self, seconds=10., y_error_bound=0.1, a_error_bound=math.pi*30./180.):
        self.load_state()
        x0=self.chassis.position.x
        y0=self.chassis.position.y
        self.settings.motorOn=True
        diffY,diffA = self.simulate(int(seconds*self.settings.hz), y0=y0, y_error_bound=y_error_bound, a_error_bound=a_error_bound)
        diffX = abs(self.chassis.position.x-x0)
        score = diffX if (diffY<y_error_bound and diffA<a_error_bound) else -1.
        #print('diffX=%f diffY=%f diffA=%f score=%f'%(diffX,diffY,diffA,score))
        return diffX
    def render(self, screen):
        self.renderer.screenSize=(screen.get_width(),screen.get_height())
        PygameDraw.surface=screen
        self.world.DrawDebugData()
    def render_pov(self, path, screen, areaLight=True, floorScale=0., floorColorCoef=0.6, orthoCam=False, loc=None):
        #povray
        objects=[]
        #floor
        if floorScale>0.:
            color0=vp.Pigment('rgb',[0.263*floorColorCoef,0.345*floorColorCoef,0.549*floorColorCoef])
        else: color0=vp.Pigment('rgb', [0.859*floorColorCoef,0.910*floorColorCoef,0.831*floorColorCoef])
        vps=vp.Box((-self.floorx,-self.floory+self.floor_offsety,-self.floorx),
                   ( self.floorx,             self.floor_offsety, self.floorx),
                   vp.Pigment(  'checker',color0,
                                vp.Pigment('rgb', [0.859*floorColorCoef,0.910*floorColorCoef,0.831*floorColorCoef]),
                                'scale',floorScale))
        if not orthoCam:
            objects.append(vps)
        #bodies
        for body in self.world.bodies:
            if body.mass==0.0:
                continue
            shape=body.fixtures[0].shape
            if isinstance(shape,b2PolygonShape) and len(shape.vertices)==4:
                #print("torso")
                fh=vp.Finish('phong', 0.3)
                pg=vp.Pigment('rgb',[153./255, 102./255, 51./255])
                vps=vp.Box((shape.vertices[0][0],shape.vertices[0][1],-self.settings.thickZ),
                           (shape.vertices[2][0],shape.vertices[2][1], self.settings.thickZ),
                           fh,pg,
                           'rotate',(0,0,body.angle*180.0/math.pi),
                           'translate',(body.position[0],body.position[1],0))
                objects.append(vps)
                ctr_pos=body.position
            if isinstance(shape,b2PolygonShape) and len(shape.vertices)>4:
                #print("leg")
                fh=vp.Finish('phong', 0.3)
                pg=vp.Pigment('rgb',[20./255, 20./255, 20./255])
                pgL=vp.Pigment('rgb',[0.761, 0.278, 0.220])
                assert len(shape.vertices)==self.settings.res*2
                dists=[]
                for i in range(self.settings.res*2):
                    if getXLen(sub(shape.vertices[i-1],shape.vertices[i]))>self.settings.rad*5:
                        dists.append(shape.vertices[i-1])
                        dists.append(shape.vertices[i  ])
                pos0=mul(add(dists[0],dists[3]),0.5)
                pos1=mul(add(dists[1],dists[2]),0.5)
                for offZ,rel,offZJ in body.userData:
                    vpc=vp.Cylinder((pos0[0],pos0[1],offZ),
                                    (pos1[0],pos1[1],offZ),self.settings.rad,
                                    fh,pg,
                                    'rotate',(0,0,body.angle*180.0/math.pi),
                                    'translate',(body.position[0],body.position[1],0))
                    vpc0=vp.Sphere((pos0[0],pos0[1],offZ),self.settings.rad,
                                   fh,pg,
                                   'rotate',(0,0,body.angle*180.0/math.pi),
                                   'translate',(body.position[0],body.position[1],0))
                    vpc1=vp.Sphere((pos1[0],pos1[1],offZ),self.settings.rad,
                                   fh,pg,
                                   'rotate',(0,0,body.angle*180.0/math.pi),
                                   'translate',(body.position[0],body.position[1],0))
                    vpcL=vp.Cylinder((rel[0],rel[1],offZ),
                                     (rel[0],rel[1],offZJ),self.settings.rad,
                                     fh,pgL,
                                     'rotate',(0,0,body.angle*180.0/math.pi),
                                     'translate',(body.position[0],body.position[1],0))
                    objects.append(vp.Union(vpc,vpc0,vpc1,vpcL))
            if isinstance(shape,b2CircleShape):
                #print("wheel")
                for offZ in body.userData:
                    fh=vp.Finish('phong', 0.3)
                    pg=vp.Pigment('rgb',[0.694,0.549,0.514])
                    pgL=vp.Pigment('rgb',[0.761, 0.278, 0.220])
                    assert shape.pos[0]==0.0 and shape.pos[1]==0.0
                    vps=vp.Cylinder((body.position[0],body.position[1], offZ-self.settings.rad),
                                    (body.position[0],body.position[1], offZ+self.settings.rad),shape.radius,fh,pg)
                    objects.append(vps)
                    vps=vp.Cylinder((body.position[0],body.position[1],-offZ-self.settings.rad),
                                    (body.position[0],body.position[1],-offZ+self.settings.rad),shape.radius,fh,pg)
                    objects.append(vps)
                vpcL=vp.Cylinder((body.position[0],body.position[1],min(body.userData)),
                                 (body.position[0],body.position[1],max(body.userData)),self.settings.rad,fh,pgL)
                objects.append(vpcL)
        #background
        objects.append(vp.Background('color','White'))
        #lights
        for i in range(4):
            dist=2*2
            ALR=50
            ALL=4*5
            xSgn=(1 if (i%2 )==0 else -1)*2.0
            zSgn=(1 if (i//2)==0 else -1)*2.0
            dir=[self.settings.lightDist*xSgn*dist,self.settings.lightDist*dist,self.settings.lightDist*zSgn*dist]
            ref=[0.,0.,0.]
            for d in range(3):
                if abs(dir[d])<abs(dir[(d+1)%3]) and abs(dir[d])<abs(dir[(d+2)%3]):
                    ref[d]=1.
            ALX=normalize(cross(dir,ref))
            ALY=normalize(cross(dir,ALX))
            params=[[ctr_pos[0]+dir[0],ctr_pos[1]+dir[1],dir[2]],'color','White']
            if areaLight:
                params+=['adaptive', 1, 'area_light', mul(ALX,ALL), mul(ALY,ALL), ALR, ALR, 'jitter']
            sun=vp.LightSource(*params)
            #print('%f %f'%(xSgn,zSgn))
            objects.append(sun)
        #camera
        if orthoCam:
            if loc is None:
                loc=[ctr_pos[0],ctr_pos[1],self.settings.cameraDistOrtho]
            cam=vp.Camera('orthographic','angle',60,'location',loc,'look_at', [ctr_pos[0], ctr_pos[1], 0.0])
        else:
            if loc is None:
                loc=[self.settings.cameraDist+ctr_pos[0],self.settings.cameraDist+ctr_pos[1],self.settings.cameraDist]
            cam=vp.Camera('angle',60,'location',loc,'look_at', [ctr_pos[0], ctr_pos[1], 0.0])
        s=vp.Scene(cam,objects=objects,included=['colors.inc'])
        #output
        p=os.path.dirname(os.path.abspath(path))
        if not os.path.isdir(p):
            os.mkdir(p)
        if path.endswith('.pov'):
            with open(path,'w') as f:
                f.write(str(s))
        elif screen is not None:
            s.render(path,width=screen.get_width(),height=screen.get_height(),antialiasing=0.3)
            img=pygame.image.load(path)
            screen.blit(img,(0,0))
        return loc
    def bounding_box(self):
        #self.simulate() #force update
        FLT_MAX=1e6
        aabb=b2AABB()
        aabb.lowerBound = b2Vec2( FLT_MAX, FLT_MAX)
        aabb.upperBound = b2Vec2(-FLT_MAX,-FLT_MAX)
        for body in self.world.bodies:
            aabb.Combine(LinkagePhysics.get_aabb(body))
        return aabb
    def save_state(self):
        self.state=[(b.position.x,b.position.y,b.angle) for b in self.world.bodies]
    def load_state(self):
        for s,b in zip(self.state,self.world.bodies):
            b.position.x=s[0]
            b.position.y=s[1]
            b.angle=s[2]
            b.linearVelocity.x=0
            b.linearVelocity.y=0
            b.angularVelocity=0
    @staticmethod
    def get_aabb(body):
        FLT_MAX=1e6
        aabb=b2AABB()
        aabb.lowerBound = b2Vec2( FLT_MAX, FLT_MAX)
        aabb.upperBound = b2Vec2(-FLT_MAX,-FLT_MAX)
        for fixture in body.fixtures:
            shape = fixture.shape
            if isinstance(shape,b2PolygonShape):
                assert shape.childCount==1
                for v in shape.vertices:
                    v=body.transform*v
                    r=b2Vec2(shape.radius, shape.radius)
                    aabb2=b2AABB()
                    aabb2.lowerBound=b2Vec2(v[0],v[1])-r
                    aabb2.upperBound=b2Vec2(v[0],v[1])+r
                    aabb.Combine(aabb2)
            elif isinstance(shape,b2CircleShape):
                v=body.transform*shape.pos
                r=b2Vec2(shape.radius, shape.radius)
                aabb2=b2AABB()
                aabb2.lowerBound=b2Vec2(v[0],v[1])-r
                aabb2.upperBound=b2Vec2(v[0],v[1])+r
                aabb.Combine(aabb2)
        return aabb

def main_linkage_physics(link, path='frms', recordTime=None):
    #param
    aaCoef=4
    batch=24
    sim=False if recordTime is None else True
    pov=False
    povAll=False
    orthoCam=False
    timeSpan=30
    ctr=False

    #visualize
    pygame.init()
    screen=pygame.display.set_mode((512,512))
    screen_hires=pygame.Surface((screen.get_width()*aaCoef,screen.get_height()*aaCoef))
    link.settings.motorOn=True
    link.load_state()   #reset state
    link.center_view(screen_hires)
    link.set_zoom(50)
    done=False
    offset=False
    loc=None
    fid=0
    frmId = 0
    print("SPACE    : simulate")
    print("LCTRL    : orthogonal render")
    print("1        : perspective render")
    print("2        : perspective, animation render")
    print("RCTRL    : switch motor")
    print("LSHIFT   : center camera")
    print("MOUSE1   : pan")
    while not done:
        #event
        for event in pygame.event.get():
            if event.type==pygame.QUIT:
                done=True
            elif event.type == pygame.KEYDOWN and event.key == pygame.K_SPACE:
                sim=not sim
            elif event.type == pygame.KEYDOWN and event.key == pygame.K_LCTRL:
                orthoCam=False
                pov=not pov
            elif event.type == pygame.KEYDOWN and event.key == pygame.K_1:
                orthoCam=True
                pov=not pov
            elif event.type == pygame.KEYDOWN and event.key == pygame.K_2:
                orthoCam=False
                povAll=not povAll
                frmId=0
            elif event.type == pygame.KEYDOWN and event.key == pygame.K_3:
                link.load_state()
            elif event.type == pygame.KEYDOWN and event.key == pygame.K_RCTRL:
                link.settings.motorOn=not link.settings.motorOn
            elif event.type == pygame.KEYDOWN and event.key == pygame.K_LSHIFT:
                ctr=not ctr
            elif event.type == pygame.MOUSEBUTTONDOWN:
                if event.button == 1:
                    offset=True
                    offset_pos=event.pos
                if event.button == 4:
                    link.set_zoom(link.get_zoom()+1.)
                elif event.button == 5:
                    link.set_zoom(max(link.get_zoom()-1.,1.))
            elif event.type == pygame.MOUSEMOTION and offset:
                delta=mul(sub(event.pos,offset_pos),aaCoef)
                link.set_offset(add(link.get_offset(),(-delta[0],delta[1])))
                offset_pos=event.pos
            elif event.type == pygame.MOUSEBUTTONUP:
                if event.button == 1:
                    offset=False
                    
        #frame
        screen_hires.fill([255,255,255])
        if ctr:
            link.center_view(screen_hires)
        if sim:
            link.simulate(batch=batch)
        if pov:
            link.render_pov('frm.png',screen_hires, orthoCam=orthoCam)
            pov=False
        if povAll:
            if not os.path.exists(path):
                os.mkdir(path)
            loc=link.render_pov(path+'/frm%d.png'%frmId,screen_hires, areaLight=False, floorScale=10., orthoCam=orthoCam, loc=loc)
            link.settings.motorOn=True
            frmId+=1
            sim=True
            if frmId>timeSpan*link.settings.hz/batch:
                povAll=False
        else:
            link.render(screen_hires)
            pygame.transform.smoothscale(screen_hires,screen.get_size(),screen)
            if recordTime is not None:
                assert isinstance(recordTime,float) or isinstance(recordTime,int)
                if frmId==0 and os.path.exists(path):
                    shutil.rmtree(path)
                if not os.path.exists(path):
                    os.mkdir(path)
                #save image
                pygame.image.save(screen,path+'/frm%d.png'%frmId)
                frmId+=1
                #terminate condition
                if frmId * batch / link.settings.hz > recordTime:
                    from gen_gif import img_to_gif
                    img_to_gif(path+'/frm%d.png', 'record.gif', screen.get_size(), 1000/batch)
                    break
        pygame.display.flip()
        fid=fid+1
        
def create_robot(link, tau=8000., spd=1., sep=5., mu=0.25, dr=1., dl=1., nleg=4):
    if isinstance(link,str):
        import pickle
        from optimizer_anneal import LinkageAnnealer
        opt=LinkageAnnealer()
        with open(link, 'rb') as handle:
            state=pickle.load(handle)
        opt.state=state
        link=opt.set_to_linkage()
        
    settings=fwSettings()
    settings.torque=tau
    settings.motorSpeed=spd
    settings.friction=mu
    settings.densityBody=dr
    settings.densityLeg=dl

    #ensure ctr_motor is zero
    link.ctr_motor=(0.,0.)
    link.rad_motor=abs(link.rad_motor)
    #check whether each state can be reached
    if not link.check_geometry_feasibility():
        return None
        
    robot=LinkagePhysics(link,settings=settings)
    robot.create_torso((-abs(sep),-1.),(abs(sep),1.))
    robot.create_legs(0., -sep, None, 'symmetric_leg', nleg)
    robot.create_legs(0.,  sep, None, 'symmetric_back_leg', nleg)
    bb=robot.bounding_box()
    robot.create_floor(offy=bb.lowerBound.y)
    robot.save_state()
    return robot
        
if __name__=='__main__':
    link=Linkage.createSimple()
    robot=create_robot('best.pickle', sep=5.)
    print("Walking distance over 10 seconds: %f"%robot.eval_performance(10.))
    main_linkage_physics(robot, recordTime=10)
