from math import cos, pi, sin

from panda3d.core import *

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


def getShapeVertices(scene, shape):
    if shape == "square":
        return [Vec3(-1.0,0.0,-1.0), Vec3(-1.0,0.0,1.0), Vec3(1.0,0.0,1.0), Vec3(1.0,0.0,-1.0)]
    if shape == "nonagon":
        vertices = []
        for i in range(9):
            theta = i * 2.0 * pi / 9.0
            vertices.append(Vec3(cos(theta), 0.0, sin(theta)))
        return vertices
    raise ValueError(f"Unsupported polygon shape: {shape}")


def transformedShapeVertices(scene, shape, radius, x, angle):
    if shape == "cylinder":
        raise ValueError("Cylinder vertices are not used directly; use collideCylinderPolygon.")

    m = Mat3().rotateMatNormaxis(angle * 180.0 / pi, Vec3(0,1,0))
    vertices = []
    for v in getShapeVertices(scene, shape):
        vertices.append(x + m.xform(v * radius))
    return vertices


def collidePolygons(scene, p1, p2):
    x1 = p1.getPos()
    x2 = p2.getPos()
    shape1 = getattr(p1, "shape", "square")
    shape2 = getattr(p2, "shape", "square")
    vertices1 = transformedShapeVertices(scene, shape1, p1.radius, x1, p1.theta)
    vertices2 = transformedShapeVertices(scene, shape2, p2.radius, x2, p2.theta)

    contacts = []
    for vertex in vertices1:
        dcl, xcl, normal = collidePointPolygon(scene, vertex, vertices2, x1, x2)
        if xcl is not None and (xcl - x1).length() < (vertex - x1).length() * 1.01:
            markerAdd(scene, xcl, "collision")
            contacts.append((dcl, xcl, -normal))
    for vertex in vertices2:
        dcl, xcl, normal = collidePointPolygon(scene, vertex, vertices1, x2, x1)
        if xcl is not None and (xcl - x2).length() < (vertex - x2).length() * 1.01:
            markerAdd(scene, xcl, "collision")
            contacts.append((dcl, xcl, normal))

    return contacts


def closestPointOnSegment(xp, x1, x2):
    x12 = x2 - x1
    denom = x12.dot(x12)
    if denom <= 1.0e-12:
        return x1
    t = (xp - x1).dot(x12) / denom
    t = max(0.0, min(1.0, t))
    return x1 * (1.0 - t) + x2 * t


def collideCylinderPolygon(scene, cylinder, polygon):
    xcyl = cylinder.getPos()
    xpoly = polygon.getPos()
    vertices = transformedShapeVertices(scene, polygon.shape, polygon.radius, xpoly, polygon.theta)

    depthbest = -1.0
    xcontactbest = None
    normalbest = None
    nvertices = len(vertices)
    for i in range(nvertices):
        x1 = vertices[i]
        x2 = vertices[(i + 1) % nvertices]
        xcontact = closestPointOnSegment(xcyl, x1, x2)
        delta = xcyl - xcontact
        dist = delta.length()
        if dist > cylinder.radius * 1.01:
            continue

        if dist > 1.0e-8:
            outward = delta / dist
        else:
            outward = ((x1 + x2) * 0.5 - xpoly).normalized()
        depth = cylinder.radius - dist
        if depth > depthbest:
            depthbest = depth
            xcontactbest = xcontact
            normalbest = -outward

    if xcontactbest is None:
        return []

    markerAdd(scene, xcontactbest, "collision")
    return [(depthbest, xcontactbest, normalbest)]


def collideCylinderCylinder(scene, p1, p2):
    x1 = p1.getPos()
    x2 = p2.getPos()
    dx = x2 - x1
    dist = dx.length()
    radius_sum = p1.radius + p2.radius
    if dist >= radius_sum:
        return []

    if dist > 1.0e-8:
        normal = dx / dist
    else:
        normal = Vec3(1.0, 0.0, 0.0)
    depth = radius_sum - dist
    xcontact = x1 + normal * p1.radius
    markerAdd(scene, xcontact, "collision")
    return [(depth, xcontact, normal)]


def collideShapes(scene, p1, p2):
    shape1 = getattr(p1, "shape", "square")
    shape2 = getattr(p2, "shape", "square")

    if shape1 == "square" and shape2 == "square":
        return collideSquareSquare(scene, p1.getPos(), p2.getPos(), p1.radius, p2.radius, p1.theta, p2.theta)
    if shape1 == "cylinder" and shape2 == "cylinder":
        return collideCylinderCylinder(scene, p1, p2)
    if shape1 == "cylinder":
        return collideCylinderPolygon(scene, p1, p2)
    if shape2 == "cylinder":
        contacts = collideCylinderPolygon(scene, p2, p1)
        return [(depth, xclose, -normal) for depth, xclose, normal in contacts]
    return collidePolygons(scene, p1, p2)
     
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

class Marker(NodePath):
    """
    Marker class
    inherits from NodePath
    """
    def __init__(self, *args, **kwargs):
        NodePath.__init__(self, *args, **kwargs)
        self.iFrame = 0

def markerInit(scene, modelfile):
    """
    Initialize diagnostic markers for a scene
    arguments: scene object (panda3d),  model file (egg format)
    """
    scene.markermodel = loader.loadModel(modelfile)
    scene.markermodel.setDepthTest(False); 
    scene.markermodel.setScale(0.05)  
    scene.markers = []

def markerClean(scene):
    """
    Remove all expired markers from a scene (if markers initialized)
    arguments: scene object (panda3d)
    """
    if (hasattr(scene, 'markers')):
        oldMarkers = scene.markers
        scene.markers = []
        for marker in oldMarkers:
            if (marker.iFrame < scene.iFrame-1):
                marker.removeNode()
            else:
                scene.markers.append(marker)

def markerAdd(scene,x,type):
    """
    Add a marker to a scene (if markers initialized)
    arguments: scene object (panda3d), Vec3 x (world space location), string type marker type
    """
    if (hasattr(scene, 'markers')):
        marker = Marker(type)
        if (type == "corner"):
            marker.setColor(Vec3(1,1,1))   
            marker.setPos(Vec3(x.x,x.y,x.z)) 
            marker.setScale(0.5)
        elif (type == "closest"):
            marker.setColor(Vec3(1,0,0))
            marker.setPos(Vec3(x.x,x.y,x.z)) 
            marker.setScale(0.5)
        else: #collision (default)
            marker.setColor(Vec3(0,1,0))
            marker.setPos(Vec3(x.x,x.y,x.z)) 
        marker.iFrame = scene.iFrame                          
                            
        marker.reparentTo(render)
        scene.markermodel.instanceTo(marker)
        scene.markers.append(marker)
