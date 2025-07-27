from functools import reduce
import operator
from ocp_vscode import show, show_object, reset_show, set_port, set_defaults, get_defaults
set_port(4943)
import time
from statistics import mode,mean
from math import floor,ceil,radians,sin,cos,pi
import math
from build123d import *
from os import getcwd

startTime =time.perf_counter_ns()

bitList=\
    ["PH0000","PH000","PH00","PH0","PH1","PH2","SL1.0","SL1.5","SL2.0","SL2.5","SL3.0","SL3.5","SL4.0",\
    "T2","T3","T4","T5","T6","T7","T8","T9","T10","T15","T20","P2","P5","P6","H0.7","H0.9","H1. 3","H1.5",\
    "H2.0","H2.5","H3.0","H4.0","Y0.6","Y1.0","Y2.0","Y2.5","Y3.0","SQ0","SQ1.0","SQ2.0","U2.0","U2.6",\
    "U3.0","tri2.0","tri2.3","tri2.5","tri3.0","sim0.8","w1.5","extPH0","extPH2","extSL2.0","extH2.0"]
printWallThickness=.62;printLayerThickness=.42
holeDia=4.35;bitHoleRad=holeDia/2;holeWallThickness=printWallThickness*3;holeWallRad=bitHoleRad+holeWallThickness; driverWallThickness=printWallThickness*5;boxWallThickness=3;
holeDepth=22;bottomThickness=printWallThickness*5; xBitSpacing=4;yBitSpacing=xBitSpacing;textHoleSep=2+holeWallThickness+bitHoleRad;xBitCount=7;
yBitCount=8;hexMaxRad=4;boxZDist=holeDepth+bottomThickness; bigHoleChamferLength=.6;smallHoleChamferLength=.2;smallestHoleChamferLength=.18;boxFilletRad=1.2; fontSize=10; fontSize=6;
maxXdist=180;hexSlotRad=4;xPadding=(20+hexSlotRad)*2.6; yPadding=xPadding;boxWallHeight=90;
driverLength=174;drierStickout=20;driverRad=8;boxEdgeChamferLength=.7;driverYOffset=10+holeWallThickness*1.3
rowChangeCount=0; lidBoltHoleSep=15; lidBoltXOffset=8; lidBoltRad=1.8; 
def reportPerf(startTime):
    endTime=time.perf_counter_ns()
    secs=(endTime-startTime)/1_000_000_000
    mins=secs//60
    secsRem=secs-mins*60
    print(f'{mins}m {secsRem}s')
    return endTime

def makeHexRectExtArrOnFace(face:Face,hexRad:float,keepOut=None):
    faceBB=face.bounding_box()
    hexOffset=hexRad*2+holeWallThickness*2
#    how can we make this more intelight small vs large and always guess which should be y and x in the GridLocations
    largeDimHexCount=floor(faceBB.size.Z/hexOffset)-1
    smallDimHexCount=floor(faceBB.size.Y/hexOffset)-1
    if faceBB.size.Y==0:
        #why are X and Z reversed from what you think they'd be here X looks like the larger dim
        largeDimHexCount=floor(faceBB.size.Z/hexOffset)-1
        smallDimHexCount=floor(faceBB.size.X/hexOffset)-1
    offAxisAngle=-30
    hexPolyFace=Plane(face)
    # hexPolyFace=hexPolyFace.rotated((0,offAxisAngle,0))
    hexPolys=[
             hexPolyFace* loc* Rot(0,offAxisAngle,0)* RegularPolygon(radius=hexRad, side_count=6, align=Align.CENTER)
            for loc in GridLocations(hexOffset,hexOffset,largeDimHexCount,smallDimHexCount,align=Align.CENTER)
        ]
    hexPolys= Sketch(Compound(hexPolys).wrapped)
    if keepOut is not None:
        keepOut=scale(keepOut,1.2)
        hexPolys=hexPolys-keepOut
        hpfFaces=hexPolys.faces()
        hpfFaceArea=floor(mode([x.area for x in hpfFaces]))
        hpf=[f for f in hpfFaces.faces() if\
            floor(f.area)==hpfFaceArea]
        hexPolys=hpf
    hexPolys=extrude(hexPolys, boxWallThickness*2, target=bitHolder,until=Until.FIRST,both=True)
    return hexPolys

#BoundBox class useless
def makeTopAndSides(bb:BoundBox):
    topAndSidesOrigin=bb.center()
    topAndSidesOrigin.Y-=driverYOffset-4;
    topAndSidesOrigin.X-=5;
    topAndSides= Pos(topAndSidesOrigin)* Box(bb.size.X+xPadding,bb.size.Y+yPadding,boxWallHeight,align=(Align.CENTER,Align.CENTER,Align.MAX))
    tasSize=topAndSides.bounding_box().size
    wallCutBox=Pos((topAndSides.edges() << Axis.Z<< Axis.Y)[0].center())*  Box(\
        tasSize.X-boxWallThickness*2,
        tasSize.Y-boxWallThickness,
        tasSize.Z-hexSlotRad-boxWallThickness,
        align=(Align.CENTER,Align.MIN,Align.MIN))

    topCutBase=(wallCutBox.faces() >> Axis.Z)[0].center()
    topCutBase.Y+=driverYOffset
    topCutDepth=(topAndSides.faces() >> Axis.Z)[0].center().Z-topCutBase.Z-boxWallThickness
    topCutBox=Pos(topCutBase)*  Box(\
        tasSize.X-xPadding,
        tasSize.Y-yPadding+7,
        topCutDepth,
        align=(Align.CENTER,Align.CENTER,Align.MIN))
    yCutFace=(topCutBox.faces()>Axis.Y)[0]
    topCutBox+=(extrude(yCutFace,hexSlotRad*.6))
    xCutFace=(topCutBox.faces()<Axis.X)[0]
    topCutBox+=extrude(xCutFace,hexSlotRad*1.6,both=True)
    topAndSides=topAndSides-[wallCutBox,topCutBox]
    return topAndSides

def makeTopHexGrid(lowerLeft:Circle|None,topRight:Circle|None):
    # Maybe could calcuate this better
    lowerLeftMaxX=lowerLeft.bounding_box().max.X+holeWallThickness+hexMaxRad*1.5
    lowerLeftMinY=lowerLeft.bounding_box().min.Y
    holeOffset=hexMaxRad*2+holeWallThickness
    topRightMinY=topRight.bounding_box().min.Y-hexMaxRad
    topRightMaxX=topRight.bounding_box().max.X
    xHexCount=abs(topRightMaxX-lowerLeftMaxX)/holeOffset
    yHexCount=abs(topRightMinY-lowerLeftMinY)/holeOffset-1
    hexPolys=[
         loc * RegularPolygon(radius=hexMaxRad, side_count=6, align=Align.MIN)
         for loc in GridLocations(holeOffset,holeOffset*.9,ceil(xHexCount),ceil(yHexCount),align=(Align.MIN, Align.MIN))
    ]
    hexPolySketch=Sketch()+hexPolys
    hexPolySketch=Pos(lowerLeftMaxX,lowerLeftMinY)*hexPolySketch
    return hexPolySketch

def textFamily(textStr, neighbor:Circle|None|RegularPolygon=None):
    text=Text(textStr,fontSize,font_path="C:/Windows/Fonts/ariali.ttf",align=(Align.MIN,Align.MAX),font_style=FontStyle.BOLD)
    tbb=text.bounding_box()
    holePos= Pos(tbb.center().X,tbb.min.Y-textHoleSep)
    holeWallCircle=holePos*Circle(holeWallRad,align=Align.CENTER)
    tempSketch=Sketch()+[holePos*Circle(bitHoleRad,align=Align.CENTER),text]
    tempSketch.label=bit
    tempSketch.__doc__=bit
    return tempSketch,holeWallCircle

sideHexSketchAreas=[]
smallHoleChamferEdges=[]
x=y=0
prev=None
firstInRow=None
rowChangeHoleWall=None
textAndHolesList=[]
holeWallsList=[]
bigHoleChamferList=[]
showList=[]
isFirstBit=False
holeWallCircle=None
row1Start=None
row2Start=None
for bit in bitList:
    cur,holeWallCircle=textFamily(bit,holeWallCircle)
    if firstInRow is None:
        firstInRow=cur
        row1Start=cur
        isFirstBit=True
    else:
        pbb=prev.bounding_box()
        if pbb.max.X+xBitSpacing+cur.bounding_box().size.X>=maxXdist:
            rowChangeCount+=1
            cur,holeWallCircle=textFamily(bit,)
            if rowChangeCount==1:
                row2Start=cur
            fbb=firstInRow.bounding_box()
            x=fbb.min.X
            y=fbb.min.Y-yBitSpacing/2
            firstInRow=cur
            rowChangeHoleWall=prev
        else:
            x=pbb.max.X+xBitSpacing
            y=pbb.max.Y
        cur.position=(x,y,0)
        isFirstBit=False

    textAndHolesList.append(cur)
    holeWallCircle=Pos(x,y)*holeWallCircle
    holeWallsList.append(holeWallCircle)
    prev=cur
topHexGrid=makeTopHexGrid(holeWallCircle,rowChangeHoleWall)
holeWallsSketch=Sketch(label='holeWalls')+holeWallsList
textAndHolesSketch=Sketch(label='textAndHoles')+textAndHolesList
#hack to get conmbined bound box
combinedTopBB=bounding_box(textAndHolesSketch,holeWallsSketch).bounding_box()
topAndSides=makeTopAndSides(combinedTopBB)
topAndSidesNoChamfer=topAndSides
holeWallsExtrude=extrude(holeWallsSketch,-(holeDepth+holeWallThickness*2))
textAndHolesExtrude=extrude(textAndHolesSketch,-holeDepth)

bitHolder=Part()+[topAndSides,holeWallsExtrude]
bitHolder-=textAndHolesExtrude
bitHolderTopHexEdges=bitHolder.edges()
bitHolder-=extrude(topHexGrid,-holeDepth)
# showList.append(bitHolder)
smallHoleChamferEdges+=(bitHolder.edges()-bitHolderTopHexEdges)
#probably could calculate the sweep dims and xPadding better kinda fudged it undtill it looked good visualy
hexPathDimsFace=(topAndSides.faces()|Axis.Z)[-2]
hexPathDimsXEdges=(hexPathDimsFace.edges()>Axis.X)[:2]
hexPathDimsYEdges=(hexPathDimsFace.edges()>Axis.Y)[:2]
hexSweepCenter=hexPathDimsFace.center()
hexSweepCenter.Z=(topAndSides.faces()|Axis.Z)[0].center().Z
hexSweepCenter.X+=6
hexSweepCenter.Y+=driverYOffset
hexSlotPath=Pos(hexSweepCenter)*\
    Rectangle(mean([y.length for y in hexPathDimsYEdges]),mean([x.length for x in hexPathDimsXEdges]))
hexSlotProfile=Pos((hexSlotPath.vertices()>>Axis.X<<Axis.Y)[0].center())*Rotation((90,0,0))*RegularPolygon(radius=hexSlotRad,side_count=6)
hexSlotProfile=chamfer(hexSlotProfile.vertices(),bigHoleChamferLength)
hexSlotSweep=sweep(hexSlotProfile,hexSlotPath.edges(),transition=Transition.RIGHT )
bitHolder-=hexSlotSweep
showList.append(hexSlotPath)
showList.append(hexSlotProfile)
driverHolderBase=(bitHolder.faces()<Axis.X)[1]
sideHexGridEdges=bitHolder.edges()
preSideHexGridFaces=bitHolder.faces()
#if we use topAndSides post chamfer the BB is wrong, Grab vertices before chamfering
driverHolderHelperVertices=[e.vertices().sort_by(Axis.X)[-1] for e in topAndSides.faces().filter_by(Axis.Z).sort_by(Axis.Z)[1].edges().group_by(Axis.Y)[:2]]
driverThicknessSketch=Sketch()+driverHolderBase.location_at(1,1)*\
    Rectangle(55,abs(driverHolderHelperVertices[0].Y-driverHolderHelperVertices[1].Y),align=(Align.MIN,Align.MAX))
#
sideHexSketches=[makeHexRectExtArrOnFace((topAndSidesNoChamfer.faces()>Axis.X)[1],hexMaxRad-3),makeHexRectExtArrOnFace(driverHolderBase,hexMaxRad-2,driverThicknessSketch),makeHexRectExtArrOnFace((topAndSidesNoChamfer.faces()<Axis.Y)[1],hexMaxRad-1)]
bitHolder-=sideHexSketches
for x in sorted([x for x in bitHolder.faces() \
    if x.geom_type==GeomType.PLANE],key=lambda x: x.area, reverse=True)[:9]:
    smallHoleChamferEdges+=x.edges()-sideHexGridEdges
topBitHoleEdges=[x for x in bitHolder.edges() if x.geom_type==GeomType.CIRCLE  and x.length==2*pi*bitHoleRad]
showList.append([ x for x in bitHolder.edges() if x in topAndSides.edges()])
bitHolder=chamfer([ x for x in bitHolder.edges() if x in topAndSides.edges() or x in topBitHoleEdges],boxEdgeChamferLength) # done
bitHolder=chamfer([ x for x in bitHolder.edges() if x in smallHoleChamferEdges],smallHoleChamferLength)
driverThicknessEx=extrude(driverThicknessSketch,driverLength-drierStickout+holeWallThickness)
driverHoleSketch=driverThicknessSketch.location_at(.5,.5) *Circle(driverRad*2)
driverHoleEx=extrude(driverHoleSketch,driverLength,both=True)

driverThicknessEx=chamfer(driverThicknessEx.edges(),boxEdgeChamferLength)-driverHoleEx,
bitHolder+=driverThicknessEx
bitHolder-=driverHoleEx

bhbb=bitHolder.bounding_box()
lidPolyline=Plane.YZ*Pos(0,0,bhbb.min.X)*Polyline([
        (bhbb.min.Y,bhbb.max.Z),
        (bhbb.max.Y,bhbb.max.Z),
        ])
bhlf=(bitHolder.faces()>>Axis.Z)[0]
#the soorting is backwards from what I expect
lidStartDefiningEdges= (bhlf.edges()>Axis.Y)[0:2]
lidStartLipWidth=abs(reduce(operator.sub,[v.center().Y for v in lidStartDefiningEdges]))
print("lidLipWidth",lidStartLipWidth)
lidStartLip=Pos(bhbb.center().X,bhbb.min.Y,bhbb.max.Z)*Box(bhbb.size.X,lidStartLipWidth,boxWallThickness,align=(Align.CENTER,Align.MIN,Align.MIN))

lidEndDefiningEdges= (bhlf.edges()>Axis.Y)[-2:]
lidEndLipWidth=abs(reduce(operator.sub,[v.center().Y for v in lidEndDefiningEdges]))
print("lidLipWidth",lidEndLipWidth)
lidEndLip=Pos(bhbb.center().X,bhbb.max.Y,bhbb.max.Z)*Box(bhbb.size.X,lidEndLipWidth,boxWallThickness,align=(Align.CENTER,Align.MAX,Align.MIN))
lidCenter=Pos(bhbb.center().X,bhbb.center().Y,bhbb.max.Z)*\
    Box(bhbb.size.X,bhbb.size.Y-lidStartLipWidth-lidEndLipWidth+2,24+boxWallThickness,align=(Align.CENTER,Align.CENTER,Align.MIN))
#the soorting is backwards from what I expect
lidCenter=offset(lidCenter,-boxWallThickness,(lidCenter.faces()>Axis.Z)[0])
lid=Part()+[lidStartLip,lidCenter,lidEndLip]
lid=chamfer(lid.edges(),bigHoleChamferLength)
lidBoltHoleVertex=((bhlf.edges()<Axis.Y)[0].vertices()>>Axis.X)[0]
lidBoltHoleCutterSketch=Pos(lidBoltHoleVertex.X-lidBoltXOffset,lidBoltHoleVertex.Y-lidStartLipWidth*.5,lidBoltHoleVertex.Z)*\
    Sketch([p*Circle(lidBoltRad,align=(Align.CENTER,Align.CENTER)) for p in GridLocations(lidBoltHoleSep,lidBoltHoleSep,5,1,align=(Align.MAX,Align.CENTER))])
#the soorting is backwards from what I expect
lidBoltHoleCutterSketch+=mirror(mirror(lidBoltHoleCutterSketch,Plane((bitHolder.faces()>Axis.X)[0]).offset(-bhbb.size.X/2)),\
    Plane((bitHolder.faces()<Axis.Y)[0]).offset(-bhbb.size.Y/2))
lidBoltHoleCutter=extrude(lidBoltHoleCutterSketch,boxWallThickness*10,both=True)
lidBoltHoleEdges=lid.edges()
bitHolderLidBoltHoleEdges=bitHolder.edges()
lid-=lidBoltHoleCutter
bitHolder-=lidBoltHoleCutter
lidBoltHoleEdges=lid.edges()-lidBoltHoleEdges
lid=chamfer([ x for x in lid.edges() if x in lidBoltHoleEdges],smallestHoleChamferLength)
zUnitCutter=Plane.XY.offset(-52)
# bitHolderZChopped=split(bitHolder,zUnitCutter)
bitHolderZChopped=bitHolder.split(zUnitCutter)
showList.append(bitHolder)
showList.append(lid)
startTime=reportPerf(startTime)
# xUnitCutter=Plane((bitHolder.faces()<Axis.X)[0]).offset(-48.89)
# yUnitCutter=Plane((bitHolder.faces()>Axis.Y)[0]).offset(-88)
# bitHolderXChopped=bitHolderZChopped.split(xUnitCutter)
# bitHolderYChopped=bitHolderXChopped.split(yUnitCutter,keep=Keep.BOTTOM)
# lidXChopped=lid.split(xUnitCutter)
# lidYChopped=lidXChopped.split(yUnitCutter,keep=Keep.BOTTOM)
# export_step(bitHolder,"bitHolder.step")
# export_step(lid,"lid.step")
# export_step(bitHolderYChopped,"bitHolderUnitTest.step")
# export_step(lidYChopped,"lidUnitTest.step")
show(*showList)
print(abs(driverHolderHelperVertices[0].Y-driverHolderHelperVertices[1].Y),) #36.218