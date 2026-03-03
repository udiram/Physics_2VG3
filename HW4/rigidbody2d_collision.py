import math
from direct.showbase.ShowBase import ShowBase
from panda3d.core import *

class Particle(NodePath):
    def __init__(self, *args, **kwargs):
        NodePath.__init__(self, *args, **kwargs)
        self.vel = Vec3(0.,0.,0.)
        self.inverseMass = 1.0
        self.radius = 1.0
        self.collisionRadius = 1.8*self.radius

class RigidBody2D(Particle):
    def __init__(self, *args, **kwargs):
        Particle.__init__(self, *args, **kwargs)
        self.theta = 0.0
        self.omega = 0.0
        self.inverseMomentOfInertia = self.inverseMass/(2./3.*self.radius**2)
        self.collisionRadius = 1.8*self.radius
   
class SimpleScene(ShowBase):
    def __init__(self):
        super().__init__(self)

        self.g = 0.0
        self.eCoeffRestitution = 0.0
        self.simTime = 0.0
        self.collisionCount = 0
        self.reportedAt10 = False

        if (True):
            dlight = DirectionalLight('dlight')
            dlnp = render.attachNewNode(dlight)
            dlnp.setPos(10,0,10)
            dlnp.lookAt(0,25,0)
            render.setLight(dlnp)

        self.cube = loader.loadModel("./panda3d/Cube")
        self.cube.setScale(0.6)  
     
        #Initialize RigidBody2Ds
        scenario = 4
        self.RigidBody2Ds = []
        for i in range(2):
            p = RigidBody2D("p")  
            self.cube.instanceTo(p)

            #Adjust physical and rendering properties of particle here
            p.setPos(Vec3(-6+17.*i,25,i-0.5))
            p.vel = Vec3(2-i*6.,0., 0.)
            p.theta = 0.0
            p.omega = 0.0
            
            p.setScale(p.radius)
            p.setColor(Vec3(i,1-i,0))
            p.reparentTo(render)
            self.RigidBody2Ds.append(p)

        # Scenario setup (0-4)
        if scenario == 1:
            self.RigidBody2Ds[0].theta = math.radians(30.0)
        elif scenario == 2:
            self.RigidBody2Ds[0].theta = math.radians(60.0)
            self.RigidBody2Ds[1].theta = math.radians(30.0)
        elif scenario == 3:
            self.RigidBody2Ds[0].omega = -5.0
            self.RigidBody2Ds[1].vel = Vec3(0.,0.,0.)
            self.RigidBody2Ds[1].setPos(Vec3(-1.9,25,1.9))
        elif scenario == 4:
            self.RigidBody2Ds[0].omega = 3.0
            self.RigidBody2Ds[1].theta = math.radians(-10.0)

        # Apply initial rotations to render
        for p in self.RigidBody2Ds:
            p.setHpr(0., 0., p.theta*180/3.141592653589793238462)

        self.report_state("initial", self.simTime)
        self.gameTask = taskMgr.add(self.updateRigidBody2Ds, "updateRigidBody2Ds")

    def report_state(self, label, t):
        total_p = Vec3(0.,0.,0.)
        total_ke_bulk = 0.0
        total_ke_spin = 0.0
        total_l_bulk = 0.0
        total_l_spin = 0.0

        print("")
        print(f"=== {label} (t={t:.6f}s) ===")
        for i,p in enumerate(self.RigidBody2Ds):
            # Mass and moment of inertia
            m = 1.0 / p.inverseMass
            I = 1.0 / p.inverseMomentOfInertia
            r = p.getPos()
            v = p.vel

            # Linear momentum: p = m v
            p_lin = v * m
            # Bulk (translational) kinetic energy: 1/2 m v^2
            ke_bulk = 0.5 * m * v.length_squared()
            # Spin (rotational) kinetic energy: 1/2 I omega^2
            ke_spin = 0.5 * I * (p.omega**2)
            # Bulk angular momentum about origin (y-axis in x-z plane): L = r x p
            l_bulk = r.cross(p_lin).y
            # Spin angular momentum about COM (y-axis): L = I omega
            l_spin = I * p.omega

            total_p += p_lin
            total_ke_bulk += ke_bulk
            total_ke_spin += ke_spin
            total_l_bulk += l_bulk
            total_l_spin += l_spin

            print(f"body{i}_pos={r} body{i}_vel={v} body{i}_omega={p.omega:.6f}")
            print(f"body{i}_momentum={p_lin}")
            print(f"body{i}_ke_bulk={ke_bulk:.6f} body{i}_ke_spin={ke_spin:.6f}")
            print(f"body{i}_L_bulk_y={l_bulk:.6f} body{i}_L_spin_y={l_spin:.6f}")

        print(f"total_momentum={total_p}")
        print(f"total_ke_bulk={total_ke_bulk:.6f} total_ke_spin={total_ke_spin:.6f} total_ke={total_ke_bulk+total_ke_spin:.6f}")
        print(f"total_L_bulk_y={total_l_bulk:.6f} total_L_spin_y={total_l_spin:.6f} total_L_y={total_l_bulk+total_l_spin:.6f}")
        print("=== end ===")
        print("")

    def collideSquareSquare( self, x1, x2, radius1, radius2, angle1, angle2 ):
        square = [ Vec3(-1,0,-1), Vec3(-1,0,1), Vec3(1,0,1), Vec3(1,0,-1) ]

        m1 = Mat3().rotateMatNormaxis( angle1*180/3.14159265, Vec3(0,1,0) )
        m2 = Mat3().rotateMatNormaxis( angle2*180/3.14159265, Vec3(0,1,0) )

        square1=[]
        square2=[]

        for vertex in square:
            square1.append( x1 + m1.xform( vertex*radius1 ) )
            square2.append( x2 + m2.xform( vertex*radius2 ) )

        # 2D cross product (x,z) to support polygon clipping tests
        def cross2(a, b):
            return a.x*b.z - a.z*b.x

        # Build a potential separating axis from an edge normal
        def axis_from_edge(a, b):
            e = b - a
            axis = Vec3(-e.z, 0., e.x)
            if axis.length() == 0.0:
                return None
            axis.normalize()
            return axis

        # Project polygon vertices onto axis to get interval
        def project(axis, poly):
            d0 = axis.dot(poly[0])
            dmin = d0
            dmax = d0
            for v in poly[1:]:
                d = axis.dot(v)
                if d < dmin:
                    dmin = d
                elif d > dmax:
                    dmax = d
            return dmin, dmax

        # Sutherland–Hodgman polygon clipping (convex only)
        def clip_polygon(poly, clip):
            def inside(p, a, b):
                return cross2(b-a, p-a) <= 1e-6

            def intersect(s, e, a, b):
                r = e - s
                s1 = b - a
                denom = cross2(r, s1)
                if abs(denom) < 1e-8:
                    return e
                t = cross2(a - s, s1) / denom
                return s + r * t

            output = poly[:]
            for i in range(len(clip)):
                a = clip[i]
                b = clip[(i+1) % len(clip)]
                input_poly = output
                output = []
                if not input_poly:
                    break
                s = input_poly[-1]
                for e in input_poly:
                    if inside(e, a, b):
                        if not inside(s, a, b):
                            output.append(intersect(s, e, a, b))
                        output.append(e)
                    elif inside(s, a, b):
                        output.append(intersect(s, e, a, b))
                    s = e
            return output

        # Square-square collision using the Separating Axis Theorem (SAT)
        axes = []
        for i in range(4):
            axis = axis_from_edge(square1[i], square1[(i+1) % 4])
            if axis is not None:
                axes.append(axis)
        for i in range(4):
            axis = axis_from_edge(square2[i], square2[(i+1) % 4])
            if axis is not None:
                axes.append(axis)

        # Find smallest overlap and its axis (collision normal)
        min_overlap = None
        best_axis = None
        for axis in axes:
            min1, max1 = project(axis, square1)
            min2, max2 = project(axis, square2)
            overlap = min(max1, max2) - max(min1, min2)
            if overlap < 0.0:
                return (None, None, None, [], square1, square2)
            if (min_overlap is None) or (overlap < min_overlap):
                min_overlap = overlap
                best_axis = axis

        # Ensure normal points from square1 to square2
        if best_axis is None:
            return (None, None, None, [], square1, square2)
        if (x2 - x1).dot(best_axis) < 0.0:
            best_axis = -best_axis

        # Clip to find contacts; use centroid if multiple points
        contacts = clip_polygon(square1, square2)
        if not contacts:
            contacts = clip_polygon(square2, square1)
        if contacts:
            rp = Vec3(0.,0.,0.)
            for c in contacts:
                rp += c
            rp /= float(len(contacts))
        else:
            # Fallback: approximate contact along the normal
            rp = (x1 + x2) * 0.5 - best_axis * (0.5 * (min_overlap if min_overlap is not None else 0.0))

        return (min_overlap, rp, best_axis, contacts, square1, square2)

    def classifyContactFeature(self, contact, square1, square2):
        def dist_point_segment(p, a, b):
            ab = b - a
            denom = ab.dot(ab)
            if denom <= 1e-8:
                return (p - a).length()
            t = (p - a).dot(ab) / denom
            t = max(0.0, min(1.0, t))
            proj = a + ab * t
            return (p - proj).length()

        def min_vertex_edge_dist(p, square):
            min_v = None
            min_e = None
            for i in range(4):
                v = square[i]
                d_v = (p - v).length()
                if (min_v is None) or (d_v < min_v):
                    min_v = d_v
                a = square[i]
                b = square[(i+1) % 4]
                d_e = dist_point_segment(p, a, b)
                if (min_e is None) or (d_e < min_e):
                    min_e = d_e
            return min_v, min_e

        v1, e1 = min_vertex_edge_dist(contact, square1)
        v2, e2 = min_vertex_edge_dist(contact, square2)
        is_corner_1 = v1 <= e1
        is_corner_2 = v2 <= e2

        if is_corner_1 and not is_corner_2:
            return ("body0", "body1")
        if is_corner_2 and not is_corner_1:
            return ("body1", "body0")
        # Ambiguous: choose the one with stronger "corner-ness" (smaller v-e gap)
        gap1 = v1 - e1
        gap2 = v2 - e2
        if gap1 <= gap2:
            return ("body0", "body1")
        return ("body1", "body0")

    def updateRigidBody2Ds(self, task):
        # Get the time elapsed since the next frame. 
 
        #dt = globalClock.getDt()  
        #use precise timesteps
        dt = 1/60.
        self.simTime += dt
        if (not self.reportedAt10) and (self.simTime >= 10.0):
            print(f"total_collisions={self.collisionCount}")
            self.report_state("after_10s", self.simTime)
            self.reportedAt10 = True

        for p in self.RigidBody2Ds:
            a = Vec3(0.,0.,-self.g)  # simple a here
            vold = p.vel
            p.vel = p.vel + a*dt  
            rold = p.getPos() 
            r = rold + p.vel * dt
            p.setPos(r)

            p.theta += p.omega * dt
            p.setHpr( 0., 0., p.theta*180/3.141592653589793238462  )

        #check other RigidBody2Ds in pairs (exclude self)
        for i,pi in enumerate(self.RigidBody2Ds):
            for pj in self.RigidBody2Ds[i+1:]:
                #collision detection
                xi = pi.getPos()
                xj = pj.getPos()

                rij = xj-xi
                #check 1: possible overlap
                if (rij.length() < pi.collisionRadius + pj.collisionRadius):
                    print("Possible contact")

                    #check 2: find collision point (if one exists)
                    d,xclose,normal,contacts,sq1,sq2 = self.collideSquareSquare( xi, xj, pi.radius, pj.radius, pi.theta, pj.theta )
                    if (normal is not None):
                        print("Contact found")

                        ui = pi.vel
                        uj = pj.vel
                        rip = xclose - xi
                        rjp = xclose - xj
                        # Full relative velocity at the contact point (linear + angular)
                        vip = ui + Vec3(0.,pi.omega,0.).cross(rip)
                        vjp = uj + Vec3(0.,pj.omega,0.).cross(rjp)
                        uij = vjp - vip
                        #check 3: must be approaching
                        if (uij.dot(normal) < 0):
                            print("Collision detected!")
                            self.collisionCount += 1

                            #resolve collision
                            nij = normal
                            rp = xclose
                            rip = rp - xi
                            rjp = rp - xj
                            ripcn = rip.cross(nij)
                            rjpcn = rjp.cross(nij)
                            # Momentum impulse with rotational effects (effective mass includes rotation)
                            denom = (pi.inverseMass + pj.inverseMass +
                                     ripcn.dot(ripcn)*pi.inverseMomentOfInertia +
                                     rjpcn.dot(rjpcn)*pj.inverseMomentOfInertia)
                            # Impulse magnitude along normal
                            dp = nij*(1+self.eCoeffRestitution)/denom*uij.dot(nij)
                            pi.vel += dp*pi.inverseMass
                            pj.vel -= dp*pj.inverseMass 
                            pi.omega += (rip.cross(dp)).y*pi.inverseMomentOfInertia
                            pj.omega -= (rjp.cross(dp)).y*pj.inverseMomentOfInertia
                            print(f"impulse={dp}")

                            # Reporting for Q2
                            is_multi = (len(contacts) >= 2)
                            contact_kind = "multi-contact (averaged)" if is_multi else "single contact"
                            corner_body, face_body = self.classifyContactFeature(xclose, sq1, sq2)
                            print(f"time={self.simTime:.4f}")
                            print(f"contact_point={xclose}")
                            print(f"normal={normal}")
                            print(f"contact_type={contact_kind}")
                            if is_multi:
                                print(f"raw_contacts={contacts}")
                                avg = Vec3(0.,0.,0.)
                                for c in contacts:
                                    avg += c
                                avg /= float(len(contacts))
                                print(f"averaged_contact={avg}")
                            print(f"corner_from={corner_body}, face_from={face_body}")


        return task.cont
    
scene=SimpleScene()
scene.run()
