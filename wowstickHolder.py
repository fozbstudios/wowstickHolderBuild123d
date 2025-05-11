from ocp_vscode import show, show_object, reset_show, set_port, set_defaults, get_defaults
set_port(3939)
import time
from statistics import mode,mean
from math import floor,ceil,radians,sin,cos,pi
from build123d import *
startTime =time.perf_counter_ns()


bitList=\
    ["PH0000","PH000","PH00","PH0","PH1","PH2","SL1.0","SL1.5","SL2.0","SL2.5","SL3.0","SL3.5","SL4.0",\
    "T2","T3","T4","T5","T6","T7","T8","T9","T10","T15","T20","P2","P5","P6","H0.7","H0.9","H1. 3","H1.5",\
    "H2.0","H2.5","H3.0","H4.0","Y0.6","Y1.0","Y2.0","Y2.5","Y3.0","SQ0","SQ1.0","SQ2.0","U2.0","U2.6",\
    "U3.0","tri2.0","tri2.3","tri2.5","tri3.0","sim0.8","w1.5","extPH0","extPH2","extSL2.0","extH2.0"]
holeDia=8.3;bitHoleRad=holeDia/2;holeWallThickness=3;holeWallRad=bitHoleRad+holeWallThickness; driverWallThickness=holeWallThickness*2;boxWallThickness=3;
holeDepth=8;bottomThickness=5; xBitSpacing=4;yBitSpacing=xBitSpacing;textHoleSep=2+holeWallThickness+bitHoleRad;xBitCount=7;
yBitCount=8;hexMaxRad=5;boxZDist=holeDepth+bottomThickness; bigHoleChamferLength=.6;smallHoleChamferLength=.25;boxFilletRad=1.2; textHeight=6; fontSize=6;
maxXdist=210;hexSlotRad=4;xPadding=(20+hexSlotRad)*2; yPadding=xPadding;boxWallHeight=90;driverLength=165;driverRad=8;boxEdgeChamferLength=.7;
rowChangeCount=0
def reportPerf(startTime):
    endTime=time.perf_counter_ns()
    secs=(endTime-startTime)/1_000_000_000
    mins=secs//60
    secsRem=secs-mins*60
    print(f'{mins}m {secsRem}s')
    return endTime

def makeHexRectExtArrOnFace(face:Face,hexRad:float,keepOut=None):
    faceBB=face.bounding_box()
  
    hexOffset=hexRad*2+holeWallThickness
#    how can we make this more intelight small vs large and always guess which should be y and x in the GridLocations
    largeDimHexCount=floor(faceBB.size.Z/hexOffset)-1
    smallDimHexCount=floor(faceBB.size.Y/hexOffset)-1
    if faceBB.size.Y==0:
        #why are X and Z reversed from what you think they'd be here X looks like the larger dim
        largeDimHexCount=floor(faceBB.size.Z/hexOffset)-1
        smallDimHexCount=floor(faceBB.size.X/hexOffset)-1
    offAxisAngle=-30
    hexPolys=None
    if keepOut is None:
        hexPolys=[
            loc* Rot(0,offAxisAngle,0) * RegularPolygon(radius=hexRad, side_count=6, align=Align.CENTER)
            for loc in GridLocations(hexOffset,hexOffset,largeDimHexCount,smallDimHexCount,align=Align.CENTER)
        ]
        hexPolys=Sketch()+hexPolys
        
    else:
        print("yooooo")
        hexPolys=[
        loc * RegularPolygon(radius=hexRad, side_count=6, align=Align.CENTER)
        # loc * Rot(0,offAxisAngle,0)* RegularPolygon(radius=hexRad, side_count=6, align=Align.CENTER)
        for loc in GridLocations(hexOffset,hexOffset,largeDimHexCount,smallDimHexCount,align=Align.CENTER)
        ]
        hexPolys=Sketch()+hexPolys

        hexPolys=hexPolys-keepOut
        hpfFaces=hexPolys.faces()
        hpfFaceArea=floor(mode([x.area for x in hpfFaces]))
        hpf=[f for f in hpfFaces.faces() if\
            floor(f.area)==hpfFaceArea]
        hexPolys=hpf
        show_object(hpf,name="hpf")
        
    hexPolyFace=Plane(face)
    hexPolys=[hexPolyFace*s for s in hexPolys]
    # hexPolys=Plane(face)hexPolys)
    # hexPolys=Plane(face)*Rot(0,180,0)*hexPolys
    hexPolys=extrude(hexPolys, boxWallThickness*2, target=bitHolder,until=Until.FIRST,both=True)
    return hexPolys

#BoundBox class useless
def makeTopAndSides(bb:BoundBox):
    topAndSides= Pos(bb.center())* Box(bb.size.X+xPadding,bb.size.Y+yPadding,boxWallHeight,align=(Align.CENTER,Align.CENTER,Align.MAX))
    tasSize=topAndSides.bounding_box().size
    wallCutBox=Pos((topAndSides.edges() << Axis.Z<< Axis.Y)[0].center())*  Box(\
        tasSize.X-boxWallThickness*2,
        tasSize.Y-boxWallThickness,
        tasSize.Z-hexSlotRad-boxWallThickness,
        align=(Align.CENTER,Align.MIN,Align.MIN))

    topCutBase=(wallCutBox.faces() >> Axis.Z)[0].center()
    topCutDepth=(topAndSides.faces() >> Axis.Z)[0].center().Z-topCutBase.Z-boxWallThickness
    topCutBox=Pos(topCutBase)*  Box(\
        tasSize.X-xPadding,
        tasSize.Y-yPadding,
        topCutDepth,
        align=(Align.CENTER,Align.CENTER,Align.MIN))
    yCutFace=(topCutBox.faces()>Axis.Y)[0]
    topCutBox+=(extrude(yCutFace,hexSlotRad*.6))
    xCutFace=(topCutBox.faces()<Axis.X)[0]
    topCutBox+=extrude(xCutFace,hexSlotRad*1.6,both=True)

    topAndSides=topAndSides-[wallCutBox,topCutBox]
    return topAndSides
def in2d(e:Edge, bb:BoundBox,bbn:Vector):
    if bbn!=e.normal():
        print(f"normal e: {e.normal()} bbn: {bbn}")
        return False
    v=e.vertices()[0]
   
    return any([BoundBox.find_outside_box_2d(bb,v.bounding_box())!=None for v in e.vertices()])
def makeTopHexGrid(lowerLeft:Circle|None,topRight:Circle|None):
    # Maybe could calcuate this better
    lowerLeftMaxX=lowerLeft.bounding_box().max.X+holeWallThickness+hexMaxRad
    lowerLeftMinY=lowerLeft.bounding_box().min.Y
    holeOffset=hexMaxRad*2+holeWallThickness
    topRightMinY=topRight.bounding_box().min.Y-hexMaxRad
    topRightMaxX=topRight.bounding_box().max.X
    xHexCount=abs(topRightMaxX-lowerLeftMaxX)/holeOffset
    yHexCount=abs(topRightMinY-lowerLeftMinY)/holeOffset
    hexPolys=[
         loc * RegularPolygon(radius=hexMaxRad, side_count=6, align=Align.MIN)
         for loc in GridLocations(holeOffset,holeOffset,ceil(xHexCount),ceil(yHexCount),align=(Align.MIN, Align.MIN))
    ]
    hexPolySketch=Sketch()+hexPolys
    hexPolySketch=Pos(lowerLeftMaxX,lowerLeftMinY)*hexPolySketch
    # hexPolySketch=hexPolySketch+hexPolys
    return hexPolySketch
def textFamily(textStr,textBottom=False, neighbor:Circle|None|RegularPolygon=None):
    text=Text(textStr,textHeight,font_path="C:/Windows/Fonts/ariali.ttf",align=(Align.MIN,Align.MIN if textBottom else Align.MAX))
    tbb=text.bounding_box()
    holePos=Pos(tbb.center().X,tbb.max.Y+textHoleSep) if textBottom else Pos(tbb.center().X,tbb.min.Y-textHoleSep)
    holeWallCircle=holePos*Circle(holeWallRad,align=Align.CENTER)

    tempSketch=Sketch()+[holePos*Circle(bitHoleRad,align=Align.CENTER),text]

    tempSketch.label=bit
    tempSketch.__doc__=bit
    return tempSketch,holeWallCircle

smallHoleChamferEdges=[]
x=y=0
prev=None
firstInRow=None
rowChangeHoleWall=None
textAndHolesList=[]
holeWallsList=[]
bigHoleChamferList=[]
isFirstBit=False
holeWallCircle=None
row1Start=None
row2Start=None
for bit in bitList:
    cur,holeWallCircle=textFamily(bit,rowChangeCount==0,holeWallCircle)
    if firstInRow is None:
        firstInRow=cur
        row1Start=cur
        isFirstBit=True
    else:
        pbb=prev.bounding_box()
        if pbb.max.X+xBitSpacing+cur.bounding_box().size.X>=maxXdist:
            rowChangeCount+=1
            if rowChangeCount==1:
                cur,holeWallCircle=textFamily(bit,textBottom=rowChangeCount==0)
                row2Start=cur
            fbb=firstInRow.bounding_box()
            x=fbb.min.X
            y=fbb.min.Y-yBitSpacing
            firstInRow=cur
            rowChangeHoleWall=prev
        else:
            x=pbb.max.X+xBitSpacing
            y=pbb.min.Y if rowChangeCount==0 else pbb.max.Y
        # cur= Pos(x,y)*cur
        #why does the work with .posion and not pos *?
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
bitHolderTopHexEdges=bitHolder.edges()-bitHolderTopHexEdges
antiOverhangCones=[Plane(x)*Rot(180,0,0)*Cone(holeWallRad*.90,2,holeWallThickness*2,align=(Align.CENTER,Align.CENTER,Align.MIN))

    for x in bitHolder.faces() if x.center_location.position.Z==-holeDepth]
bitHolder-=antiOverhangCones

#probably could calculate the sweep dims and xPadding better kinda fudged it undtill it looked good visualy
hexPathDimsFace=(topAndSides.faces()|Axis.Z)[-2]
hexPathDimsXEdges=(hexPathDimsFace.edges()>Axis.X)[:2]
hexPathDimsYEdges=(hexPathDimsFace.edges()>Axis.Y)[:2]
hexSweepCenter=hexPathDimsFace.center()
hexSweepCenter.Z=(topAndSides.faces()|Axis.Z)[0].center().Z
hexSweepCenter.X+=6
hexSlotPath=Pos(hexSweepCenter)*\
    Rectangle(mean([y.length for y in hexPathDimsYEdges])+hexSlotRad,mean([x.length for x in hexPathDimsXEdges])+hexSlotRad)
hexSlotProfile=Pos((hexSlotPath.vertices()>>Axis.X<<Axis.Y)[0].center())*Rotation((90,0,0))*RegularPolygon(radius=hexSlotRad,side_count=6)
hexSlotSweep=sweep(hexSlotProfile,hexSlotPath.wire(),transition=Transition.RIGHT )

bitHolder-=hexSlotSweep
bitHolderSideHexGridEdges=bitHolder.edges()

driverHolderBase=(bitHolder.faces()>>Axis.X)[0]
driverHolderBaseEdges=driverHolderBase.edges()
driverVertRef=(driverHolderBase.edges()>>Axis.Y)[0]
driverHoriRef=(driverHolderBase.edges()>>Axis.Z)[0]
r2StBB=row2Start.bounding_box()
driverHolderThicknessCircle=Circle(driverRad+driverWallThickness)
driverYcord=r2StBB.max.Y+textHoleSep+boxWallThickness*2
driverYcord+=abs(r2StBB.min.Y)if r2StBB.min.Y<0 else 0
driverSketchU=(driverVertRef.length-driverHolderThicknessCircle.radius-hexSlotRad-boxWallThickness)/driverVertRef.length
driverSketchV=1-((driverHoriRef.length-(driverYcord))/driverHoriRef.length)
driverThicknessSketch=driverHolderBase.location_at(driverSketchU,driverSketchV)*driverHolderThicknessCircle

#if we use topAndSides post chamfer the BB is wrong
sideHexSketches=[makeHexRectExtArrOnFace((topAndSidesNoChamfer.faces()>Axis.X)[1],hexMaxRad-3),makeHexRectExtArrOnFace((topAndSidesNoChamfer.faces()<Axis.X)[1],hexMaxRad-2,driverThicknessSketch),makeHexRectExtArrOnFace((topAndSidesNoChamfer.faces()<Axis.Y)[1],hexMaxRad-1)]
bitHolder-=sideHexSketches
#driver holder

b=driverHoriRef.bounding_box()

driverHoleSketch=Circle(driverRad)
driverThicknessEx=extrude(driverThicknessSketch,-driverLength)

driverThicknessBB=driverThicknessEx.bounding_box()
a1=radians(180)
a2=radians(180)
poly=Pos((driverRad*cos(a1),driverRad*sin(a1)))*RegularPolygon(driverRad*.6,3,rotation=180)
driverHoleSketch+=poly
driverHoleSketch=driverHolderBase.location_at(driverSketchU,driverSketchV)*driverHoleSketch
# driverHoleSketch=chamfer(driverHoleSketch.vertices(),bigHoleChamferLength)
driverHoleEx=extrude(driverHoleSketch,-(driverLength-8))
driverHoleExEdges=driverHoleEx.edges()
driverHoleBB=driverHoleEx.bounding_box()
driverHoleSketchBB=driverHoleSketch.bounding_box()
bitHolderSideHexGridEdges=bitHolder.edges()-bitHolderSideHexGridEdges
# bitHolderSideHexGridEdges=[x for x in bitHolderSideHexGridEdges if in2d(x,driverHoleSketchBB,driverHoleSketch.normal())]
show_object(bitHolderSideHexGridEdges)
smallHoleChamferEdges+=bitHolderSideHexGridEdges #done

# smallHoleChamferEdges+=[x for x in bitHolder.edges() if x.geom_type==GeomType.CIRCLE  and abs(x.position.Z) in [0,holeDepth+2*holeWallThickness,7]]
# bitHolder=chamfer(smallHoleChamferEdges,smallHoleChamferLength)
# smallHoleChamferEdges+=[x for x in bitHolder.edges() if x.geom_type==GeomType.CIRCLE  and x.length==2*pi*bitHoleRad]


# bitHolder=chamfer(bigHoleChamferList,bigHoleChamferLength)
bitHolderPreDriverEdges=driverHolderBase.edges()
bitHolder+=driverThicknessEx
bitHolder-=driverHoleEx
# bigHoleChamferList+=bitHolder.edges()-bitHolderPreDriverEdges #done
smallHoleChamferEdges+=bitHolderTopHexEdges



bbb=driverThicknessSketch.bounding_box()
bitHolder=chamfer([ x for x in bitHolder.edges() if x in topAndSides.edges()],boxEdgeChamferLength)
# bitHolder=chamfer([ x for x in bitHolder.edges() if x in smallHoleChamferEdges],smallHoleChamferLength)

show_object(smallHoleChamferEdges)
show_object(bitHolder)

export_step(bitHolder,"bitHolder.step")
startTime=reportPerf(startTime)
