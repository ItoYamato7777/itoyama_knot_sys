#import os
#import sys

import math


"""
import org.omg.CosNaming
import jp.go.aist.hrp.simulator

url='file:///opt/grx/%s/model/main.wrl'%(modelName)

# setup path
sys.path.append('/opt/grx/share/OpenHRP-3.1/java/openhrpstubskel.jar')
EXTRA_JAR_PATH='/opt/grx/HIRONX/share/hrpsys/jar/'
for f in os.listdir(EXTRA_JAR_PATH):
  if f.endswith(".jar") and not sys.path.count(EXTRA_JAR_PATH+f):
    sys.path.insert(0, EXTRA_JAR_PATH+f)
    print 'bodyinfo.py: '+f+' is added.'
"""
modelName ='HIRONX'

timeToInitialPose = 10.0 # [sec]
initialPose = [
   [ 0,    0,    0],   
   [-0.6,  0, -100,  15.2,  9.4,  3.2],
   [ 0.6,  0, -100, -15.2,  9.4, -3.2],
   [], 	 # [ 0,    0,    0,   0],
   [],   # [ 0,    0,    0,   0],
]

timeToOffPose = 10.0 # [sec]
offPose = [ 
   [0,    0,    0],   
   [25, -139, -157,  45,  0, 0], 
   [-25, -139, -157, -45,  0, 0],
   [], 	 # [ 0,    0,    0,   0],
   [],   # [ 0,    0,    0,   0],
]

rhandOpen  = [90.0, -90.0, -90.0, 90.0]
rhandClose = [ 0.0,   0.0,   0.0,  0.0]
lhandOpen  = [90.0, -90.0, -90.0, 90.0]
lhandClose = [ 0.0,   0.0,   0.0,  0.0]

testPatternName = 'Hello Action' # just displayed on dialog
testPattern = [
    [[ [  0,  0,-10], [ -0.6,  0,-100, 15.2,  9.4, 3.2], [ 0.6,  0,-100,-15.2,  9.4, -3.2],
       [ 0,  0,  0,  0], [  0,  0,  0,  0] ], 3],
    [[ [ 10, 20,-10], [-10.6,  0,-100, 15.2,  9.4, 3.2], [-9.4,  0,-100,-15.2,  9.4, -3.2],
       [45,-45,-45, 45], [ 45,-45,-45, 45] ], 3],
    [[ [-10,-20,-10], [  9.4,  0,-100, 15.2,  9.4, 3.2], [10.6,  0,-100,-15.2,  9.4, -3.2],
       [90,-90,-90, 90], [ 90,-90,-90, 90] ], 3],
    [[ [ 10, 20,-10], [-34,  -50,-136,-110, -25,  12],   [34,  -50,-136,110.0,-25.0,-12.0],
       [45,-45,-45, 45], [ 45,-45,-45, 45] ], 3],
    [[ [  0,  0,  0], [ -0.6,  0,-100, 15.2,  9.4, 3.2], [ 0.6,  0,-100,-15.2,  9.4, -3.2],
       [ 0,  0,  0,  0], [  0,  0,  0,  0] ], 3],
]

testPattern2Name = 'Hello Action(No Hands)' # just displayed on dialog
testPattern2 = [
    [[ [  0,  0,-10], [ -0.6,  0,-100, 15.2,  9.4, 3.2], [ 0.6,  0,-100,-15.2,  9.4, -3.2],
       [], [] ], 3],
    [[ [ 10, 20,-10], [-10.6,  0,-100, 15.2,  9.4, 3.2], [-9.4,  0,-100,-15.2,  9.4, -3.2],
       [], [] ], 3],
    [[ [-10,-20,-10], [  9.4,  0,-100, 15.2,  9.4, 3.2], [10.6,  0,-100,-15.2,  9.4, -3.2],
       [], [] ], 3],
    [[ [ 10, 20,-10], [-34,  -50,-136,-110, -25,  12],   [34,  -50,-136,110.0,-25.0,-12.0],
       [], [] ], 3],
    [[ [  0,  0,  0], [ -0.6,  0,-100, 15.2,  9.4, 3.2], [ 0.6,  0,-100,-15.2,  9.4, -3.2],
       [], [] ], 3],
]


def anglesFromDistance(gripDist):
    
    safetyMargin = 3

    l1 = 33
    l2 = 41.9
    l3 = 30
    l4 = l2 - safetyMargin*2
    l5 = 19

    #if gripDist < 0.0 or gripDist > (l1+l4)*2:
    if gripDist < 0.0 or gripDist > (l2+l5 - safetyMargin)*2:
        return None
    
    xPos   = gripDist/2.0 + safetyMargin
    #print 'xPos =', xPos
    a2Pos  = xPos - l5
    #print 'a2Pos =', a2Pos
    a1radH = math.acos(a2Pos/l2)
    #print 'a1radH = ', a1radH
    a1rad  = math.pi/2.0 - a1radH
    a2rad  = -a1rad
    #dEnd   = l2 * math.cos(a1rad) + l3

    return a1rad, a2rad, -a1rad, -a2rad #, dEnd


# this is for compatibility between different robots
def deg2radPose(pose):
    ret = []
    for p in pose:
      ret.append([jv*math.pi/180.0 for jv in p])
    return ret

"""
# get bodyInfo from modelloader
def init(nameContext):
  global url, linkInfo, dof
  obj = nameContext.resolve([org.omg.CosNaming.NameComponent('ModelLoader', '')])
  mdlldr = jp.go.aist.hrp.simulator.ModelLoaderHelper.narrow(obj)
  bodyInfo = mdlldr.getBodyInfo(url)
  linkInfo = bodyInfo.links()
  dof = 0
  for l in linkInfo:
      if l.jointId >= 0:
          dof += 1

def jointId(jointName):
  for l in linkInfo:
    if jointName == l.name:
      return l.jointId

def jointName(jointId):
  for l in linkInfo:
    if jointId == jid:
      return l.name
"""
