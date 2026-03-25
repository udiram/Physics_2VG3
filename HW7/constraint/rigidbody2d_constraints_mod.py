from panda3d.core import *
import math
from math import *
from rigidbody2d_markers_mod import *

def constraintContactList(scene):
    """
    Get constraint contacts list in world space for current state of rigidbodies from scene constraintList
    returns detailed constraint contact data in world space if any
    """
    constraints = []
    for constraintModel in scene.constraintList:
        #drmodeli includes scaling, just not rotation and translation to world space
        constraintType, pi, pj, drmodeli, drmodelj = constraintModel
        mi = Mat3().rotateMatNormaxis( pi.theta*180/math.pi, Vec3(0,1,0) )
        vi = pi.getPos() + mi.xform(drmodeli)
        mj = Mat3().rotateMatNormaxis( pj.theta*180/math.pi, Vec3(0,1,0) )
        vj = pj.getPos() + mj.xform(drmodelj)

        if (constraintType=="hinge"):
            constraints.append( constraintAddHinge( scene, pi,pj,vi,vj) )
        else:
            assert( constraintType == "known type" )
    return constraints

def constraintAddHinge( scene, pi, pj, xci, xcj ):
    """
    Generate constraint contact data and add diagnostic markers
    arguments scene, pi, pj (bodies), xci, xcj (hinge location in world space)
    returns detailed constraint contact data for hinge
    """
    xi = pi.getPos()
    xj = pj.getPos()
    markerAdd( scene, xci, "hinge" )
    markerAdd( scene, xcj, "hinge" )
    dxi = (xci-xi)
    dxj = (xcj-xj)
    dp = Vec3(0,0,0)
    return [ dp, None, pi, pj, xci, xcj, xi, xj ]    #none in second arg indicates hinge

def constraintAdd(scene, constraintType, pi, pj, drmodeli, drmodelj ):
    """
    Add constraint to list stored in scene
    arguments scene, constraintType string, pi, pj (bodies), drmodeli, drmodelj (hinge location in model space)
    returns (none)
    """
    scene.constraintList.append( ( constraintType, pi, pj, drmodeli, drmodelj ) )

