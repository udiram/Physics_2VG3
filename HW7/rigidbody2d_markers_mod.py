from panda3d.core import *

#-------- markers for collision/contact diagnostics

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
    scene.iMarker = 0
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
        if (type == "hinge"):
            scene.iMarker+=1
            marker.setColor(Vec3(((scene.iMarker)&2)/2,(scene.iMarker)&1,1))   
            marker.setPos(Vec3(x.x,x.y,x.z)) 
            marker.setScale(1.0)
        elif (type == "corner"):
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

