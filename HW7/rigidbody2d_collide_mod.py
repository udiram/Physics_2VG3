from panda3d.core import *
from math import *
from rigidbody2d_markers_mod import *

#-------- collision detection 

def collidePointLine( scene, xp, x1, x2 ):
    """
    Detect projection of point xp onto line from x1 to x2
    arguments: scene (not used), Vec3 xp x1 x2
    returns: projection Vec3 location (collision point) or None if not on line
    """
    x12 = x2-x1
    t = (xp-x1).dot(x12)/(x12).dot(x12)
    if (t < 0):
        if (t>-0.001): return x1
        else: return None
    elif (t>1):
        if (t<1.001): return x2
        else: return None
    else:
        return x1*(1-t)+x2*t

def closestPointOnSegment(xp, x1, x2):
    """
    Closest point on the line segment x1-x2 to point xp.
    """
    x12 = x2 - x1
    denom = x12.dot(x12)
    if denom == 0:
        return x1
    t = (xp - x1).dot(x12) / denom
    t = max(0.0, min(1.0, t))
    return x1 * (1 - t) + x2 * t
        
def collidePointPolygon( scene, xp, vertices, xcenp, xcenv ):
    """
    Detect collision point between point (vertex) and polydon
    arguments: scene (for diagnostics), Vec3 xp, list of Vec3 vertices of polygon
    Vec3 xcenp centre of collider, Vec3 xcenv centre of vertices (polygon)
    returns: type (collision depth (scalar), Vec3 collision point, Vec3 collision normal)
    or (None,None,None) if no collision 
    """
    nvertices = len(vertices)
    depthbest = 1e37
    xcontactbest = None
    normalbest = None
    dist_xp_xcenp = (xp-xcenp).length()
    for i in range(nvertices):
        x1 = vertices[i]
        x2 = vertices[(i+1)%nvertices]
        xcontact = collidePointLine( scene, xp, x1, x2 )
        if (xcontact == None): continue
        if ((xcontact-xcenp).length() > dist_xp_xcenp): continue

        normal = ((x1+x2)*0.5-xcenv).normalized()
        depth = (xcontact-xp).dot(normal)
        depthxcenv = (xcontact-xcenv).dot(normal)

        if (depth >= 0 and depth < depthxcenv and depth < depthbest):
            depthbest=depth
            xcontactbest=xcontact
            normalbest=normal

    return (depthbest,xcontactbest,normalbest)
     
def collideSquareSquare( scene, x1, x2, radius1, radius2, angle1, angle2 ):
    """
    Detect collision between two squares in world space
    arguments: scene (diagnostic), Vec3 x1 centre of square1, Vec3 cenre of square2
    scalar radius1, radius2   half side length of squares 1, 2
    scalar angle1l, angle2    orientation of each square in radians
    returns list of contacts with each contact a tuple (depth,contact,normal) 
    [see collidePointPolygon]
    """
    square = [Vec3(-1.0,0.0,-1.0),Vec3(-1.0,0.0,1.0),Vec3(1.0,0.0,1.0),Vec3(1.0,0.0,-1.0)]
    
    m1 = Mat3().rotateMatNormaxis( angle1*180/3.141592653589793238462, Vec3(0,1,0) )
    m2 = Mat3().rotateMatNormaxis( angle2*180/3.141592653589793238462, Vec3(0,1,0) )
    square1 = []
    square2 = []
    for i in range(4):
        square1.append(x1 + m1.xform(square[i]*radius1))
        square2.append(x2 + m2.xform(square[i]*radius2))

    contacts = []
    for i in range(4):
        dcl,xcl,normal = collidePointPolygon( scene, square1[i], square2, x1, x2 )
        if (xcl != None and (xcl-x1).length()<(square1[i]-x1).length()*1.01):
            markerAdd(scene, xcl,"collision") 
            contacts.append( (dcl,xcl,-normal) )
        dcl,xcl,normal = collidePointPolygon( scene, square2[i], square1, x2, x1 ) 
        if (xcl != None and (xcl-x2).length()<(square2[i]-x2).length()*1.01): 
            markerAdd(scene, xcl,"collision")
            contacts.append( (dcl,xcl,normal) )   

    return contacts

def getShapeVertices( scene, shape ):
    """
    Get vertices in model space
    arguments: scene (diagnostic), shape string
    returns list of vertices in model space for give shape (unscaled)
    """
    if (shape == "square"):
        vertices = [Vec3(-1.0,0.0,-1.0),Vec3(-1.0,0.0,1.0),Vec3(1.0,0.0,1.0),Vec3(1.0,0.0,-1.0)]
        return vertices
    elif (shape == "nonagon"):
        v = []
        for i in range(9):
            theta = i*2*pi/9
            v.append( Vec3( cos(theta),0.,sin(theta)) )
        return v
    elif (shape == "cylinder"):
        v = []
        for i in range(360):
            theta = i*2*pi/360
            v.append( Vec3( cos(theta),0.,sin(theta)) )
        return v
        
    return None

def transformedShapeVertices( scene, shape, radius, x, angle ):
    """
    Get vertices in world space
    arguments: scene (diagnostic), shape string, radius, position, angle
    returns list of vertices in world space for give shape (scaled,rotated,translated)
    """
    m = Mat3().rotateMatNormaxis( angle*180/3.141592653589793238462, Vec3(0,1,0) )
    vmodel = getShapeVertices(scene, shape)
    vertices = []
    for v in vmodel:
        vertices.append(x + m.xform(v*radius))

    return vertices

def collideCylinderPolygon( scene, pCyl, pPoly, sign ):
    """
    Get contacts for Cylinder-Polygon collision
    arguments: scene (diagnostic), pCyl (cylinder), pPoly (polygon), sign for normals
    returns contact if any
    """
    xc = pCyl.getPos()
    xp = pPoly.getPos()
    vp = transformedShapeVertices( scene, pPoly.shape, pPoly.radius, xp, pPoly.theta)

    nvertices = len(vp)
    best_contact = None
    for i in range(nvertices):
        x1 = vp[i]
        x2 = vp[(i+1)%nvertices]
        xcl = closestPointOnSegment(xc, x1, x2)
        rcc = xcl - xc
        dcen = rcc.length()
        if dcen >= pCyl.radius:
            continue

        if dcen > 1e-12:
            normal = -(rcc.normalized()) * sign
        else:
            edge = x2 - x1
            normal = Vec3(-edge.z, 0.0, edge.x).normalized() * sign
        depth = pCyl.radius - dcen
        candidate = (depth, xcl, normal)
        if best_contact is None or depth > best_contact[0]:
            best_contact = candidate

    if best_contact is None:
        return []

    markerAdd(scene, best_contact[1], "collision")
    return [best_contact]

def collideCylinders( scene, p1, p2 ):
    """
    Get contacts for Cylinder-Cylinder collision
    arguments: scene (diagnostic), p1 (cylinder), p2 (cylinder)
    returns contact if any
    """
    x1 = p1.getPos()
    x2 = p2.getPos()

    r12 = x2-x1
    depth = p1.radius + p2.radius - r12.length()
    if (depth<0): return []

    normal = r12.normalized()
    xcontact = (x1*p1.radius + x2*p2.radius)/(p1.radius+p2.radius)
    markerAdd(scene, xcontact,"collision")
    return [ (depth,xcontact,normal) ]

def collideShapes( scene, p1, p2 ):
    """
    Get contacts for generic shape-shape collision
    arguments: scene (diagnostic), p1 (shape), p2 (shape)
    returns contacts if any
    """
    if (p1.shape == "cylinder"):
        if (p2.shape == "cylinder"): return collideCylinders( scene, p1, p2 )
        return collideCylinderPolygon(scene, p1, p2, -1 )
    if (p2.shape == "cylinder"):
        return collideCylinderPolygon(scene, p2, p1, 1 )

    x1 = p1.getPos()
    x2 = p2.getPos()
    v1 = transformedShapeVertices( scene, p1.shape, p1.radius, x1, p1.theta)
    v2 = transformedShapeVertices( scene, p2.shape, p2.radius, x2, p2.theta)

    #for v in v1:
    #    markerAdd(scene, v, "corner")

    contacts=[]
    for v in v1:
        dcl,xcl,normal = collidePointPolygon( scene, v, v2, x1, x2 )
        if (xcl != None and (xcl-x1).length() < (v-x1).length()*1.01):
            markerAdd(scene, xcl, "collision")
            contacts.append( (dcl, xcl, -normal) )

    for v in v2:
        dcl,xcl,normal = collidePointPolygon( scene, v, v1, x2, x1 )
        if (xcl != None and (xcl-x2).length() < (v-x2).length()*1.01):
            markerAdd(scene, xcl, "collision")
            contacts.append( (dcl, xcl, normal) )                    

    return contacts

def collideContactList(scene):
    """
    Get collision contacts list for current state of rigidbodies from scene
    returns detailed collision data if any
    """
    #check RigidBody2Ds in pairs (exclude self-self or repeats)
    contacts = []
    for i,pi in enumerate(scene.RigidBody2Ds):
        for pj in scene.RigidBody2Ds[i+1:]:
            if (pi.name in pj.noCollideList or pj.name in pi.noCollideList): continue
            #collision detection
            xi = pi.getPos()
            xj = pj.getPos()

            rij = xj-xi
            #check 1: inside 3d sphere that encloses bodies
            if (abs(rij.length()) < pi.collisionRadius + pj.collisionRadius):
                #Find closest points of contact
                contacts_new = collideShapes( scene, pi, pj )
                for contact in contacts_new:
                    dclose,xclose,nij = contact
                    dxi = (xclose-xi)
                    dxj = (xclose-xj)
                    ui = pi.vel + Vec3(0.,pi.omega,0.).cross(dxi)
                    uj = pj.vel + Vec3(0.,pj.omega,0.).cross(dxj)
                    uij = uj-ui
                    unormal = uij.dot(nij)
                    vnormal = -scene.eCoeffRestitution*unormal  #target v.n value + vbias
                    #torque constants  
                    rni = dxi.cross(nij)  
                    rnj = dxj.cross(nij)  
                    iMFac = 1/(pi.inverseMass+pj.inverseMass + \
                                rni.dot(rni)*pi.inverseMomentOfInertia + rnj.dot(rnj)*pj.inverseMomentOfInertia)
                    dpScalar = 0.  #initial impulse
                    dpFric = Vec3(0,0,0)
                    contacts.append( [ dpScalar, dpFric, pi, pj, dclose, xclose, nij, uij, vnormal, dxi, dxj, iMFac ] )
    return contacts       

