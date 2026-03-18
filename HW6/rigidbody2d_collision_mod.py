from panda3d.core import *
from math import *

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

def getShapeVertices( scene, shape ):
    if (shape == "square"):
        return [Vec3(-1.0,0.0,-1.0),Vec3(-1.0,0.0,1.0),Vec3(1.0,0.0,1.0),Vec3(1.0,0.0,-1.0)]
    if (shape == "nonagon"):
        v = []
        for i in range(9):
            theta = i*2*pi/9.
            v.append( Vec3( cos(theta), 0., sin(theta) ) )
        return v
    if (shape == "cylinder"):
        v = []
        for i in range(360):
            theta = i*2*pi/360.
            v.append( Vec3( cos(theta), 0., sin(theta) ) )
        return v
    return []

def transformedShapeVertices( scene, shape, radius, x, angle ):
    # This should only be used for polygons
    m = Mat3().rotateMatNormaxis( angle*180/3.141592653589793238462, Vec3(0,1,0) )
    vmodel = getShapeVertices(scene, shape)
    vertices = []
    for v in vmodel:
        vertices.append(x + m.xform(v*radius))
    return vertices

def collideCylinderPolygon( scene, pCyl, pPoly, sign ):
    contacts = []
    xcenc = pCyl.getPos()
    xcenv = pPoly.getPos()
    vertices = transformedShapeVertices(scene, pPoly.shape, pPoly.radius, xcenv, pPoly.theta )
    nvertices = len(vertices)

    xcontacts = []
    # Case 1: Cylinder pokes into edge
    for i in range(nvertices):
        x1 = vertices[i]
        x2 = vertices[(i+1)%nvertices]
        xcontact = collidePointLine( scene, xcenc, x1, x2 )
        xcontacts.append(xcontact)
        if (xcontact == None): continue
        if ((xcontact-xcenc).length() > pCyl.radius): continue
        markerAdd(scene,xcenc,"corner")
        normal = ((x1+x2)*0.5-xcenv).normalized()
        depth = pCyl.radius + (xcontact-xcenc).dot(normal)
        if (depth >= 0):
            markerAdd(scene,xcontact,"collision")
            contacts.append( (depth, xcontact, normal*sign) )

    # Case 2: Corner of polygon pokes into cylinder
    for i in range(nvertices):
        xv = vertices[i]
        if (xcontacts[i]!=None or xcontacts[(i+nvertices-1)%nvertices]!=None): 
            continue
        rcc = xv-xcenc
        if (rcc.length() > pCyl.radius): continue
        normal = rcc.normalized()
        depth = pCyl.radius-rcc.length()
        markerAdd(scene,xv,"collision")
        contacts.append( (depth, xv, normal) )

    if (len(contacts) > 1):
        contacts = contacts[:1]
    return contacts

def collideShapes( scene, p1, p2 ):
    if (p1.shape == "cylinder"):
        contacts = collideCylinderPolygon( scene, p1, p2, -1 )
    elif (p2.shape == "cylinder"):
        contacts = collideCylinderPolygon( scene, p2, p1, 1 )
    else:
        x1 = p1.getPos()
        x2 = p2.getPos()
        v1 = transformedShapeVertices(scene, p1.shape, p1.radius, x1, p1.theta )
        v2 = transformedShapeVertices(scene, p2.shape, p2.radius, x2, p2.theta )
        for v in v1:
            markerAdd(scene, v, "corner" )
        contacts = []
        for v in v1:
            dcl,xcl,normal = collidePointPolygon( scene, v, v2, x1, x2 )
            if (xcl != None and (xcl-x1).length()<(v-x1).length()*1.01):
                markerAdd(scene,xcl,"collision")
                contacts.append( (dcl,xcl,-normal) )
        for v in v2:
            dcl,xcl,normal = collidePointPolygon( scene, v, v1, x2, x1 )
            if (xcl != None and (xcl-x2).length()<(v-x2).length()*1.01):
                markerAdd(scene, xcl,"collision")
                contacts.append( (dcl,xcl,normal) )

    # Ensure normals point from p1 to p2
    x1 = p1.getPos()
    x2 = p2.getPos()
    x12 = x2 - x1
    fixed = []
    for (dcl, xcl, normal) in contacts:
        if (normal.dot(x12) < 0):
            normal = -normal
        fixed.append((dcl, xcl, normal))
    return fixed
     
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

