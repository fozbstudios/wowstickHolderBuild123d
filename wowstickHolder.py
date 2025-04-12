from ocp_vscode import show, show_object, reset_show, set_port, set_defaults, get_defaults
set_port(3939)
import time
import statistics
from build123d import *
startTime =time.perf_counter_ns()


bitList=\
    ["PH0000","PH000","PH00","PH0","PH1","PH2","SL1.0","SL1.5","SL2.0","SL2.5","SL3.0","SL3.5","SL4.0",\
    "T2","T3","T4","T5","T6","T7","T8","T9","T10","T15","T20","P2","P5","P6","H0.7","H0.9","H1. 3","H1.5",\
    "H2.0","H2.5","H3.0","H4.0","Y0.6","Y1.0","Y2.0","Y2.5","Y3.0","SQ0","SQ1.0","SQ2.0","U2.0","U2.6",\
    "U3.0","tri2.0","tri2.3","tri2.5","tri3.0","sim0.8","w1.5","extPH0","extPH2","extSL2.0","extH2.0"]
holeDia=8.3;holeRad=holeDia/2;holeWallThickness=3; boxWallThickness=3;holeDepth=8;bottomThickness=5; xBitSpacing=4;
yBitSpacing=xBitSpacing;textHoleSep=2+holeWallThickness+holeRad;xBitCount=7; yBitCount=8;hexMaxRad=3;
boxZDist=holeDepth+bottomThickness; holeFilletRad=.6;boxFilletRolad=1.2; textHeight=6; fontSize=6; maxXdist=210;hexSlotRad=4;
xPadding=(12+hexSlotRad)*2; yPadding=xPadding;boxWallHeight=150
rowChangeCount=0
def reportPerf(startTime):   
    endTime=time.perf_counter_ns()
    secs=(endTime-startTime)/1_000_000_000
    mins=secs//60
    secsRem=secs-mins*60
    print(f'{mins}m {secsRem}s')
    return endTime

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
    topAndSides=topAndSides-[wallCutBox,topCutBox]
    return topAndSides
def textFamily(textStr,textBottom=False, neighbor=None):
    text=Text(textStr,textHeight,font_path="C:/Windows/Fonts/ariali.ttf",align=(Align.MIN,Align.MIN if textBottom else Align.MAX))
    tbb=text.bounding_box()
    holePos=Pos(tbb.center().X,tbb.max.Y+textHoleSep) if textBottom else Pos(tbb.center().X,tbb.min.Y-textHoleSep)
    holeWallCircle=holePos*Circle(holeRad+holeWallThickness,align=Align.CENTER)

    tempSketch=Sketch()+[holePos*Circle(holeRad,align=Align.CENTER),text]

    tempSketch.label=bit
    tempSketch.__doc__=bit
    return tempSketch,holeWallCircle


x=y=0
prev=None
firstInRow=None
textAndHolesList=[]
holeWallsList=[]
isFirstBit=False
for bit in bitList:
    cur,holeWallCircle=textFamily(bit,textBottom=rowChangeCount==0)
    if firstInRow is None:
        firstInRow=cur
        isFirstBit=True
    else:
        pbb=prev.bounding_box()
        if pbb.max.X+xBitSpacing+cur.bounding_box().size.X>=maxXdist:
            rowChangeCount+=1
            if rowChangeCount==1:
                cur,holeWallCircle=textFamily(bit,textBottom=rowChangeCount==0)
            fbb=firstInRow.bounding_box()
            x=fbb.min.X
            y=fbb.min.Y-yBitSpacing
            firstInRow=cur
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
holeWallsSketch=Sketch(label='holeWalls')+holeWallsList
textAndHolesSketch=Sketch(label='textAndHoles')+textAndHolesList

#hack to get conmbined bound box
topAndSides=makeTopAndSides(bounding_box(textAndHolesSketch,holeWallsSketch).bounding_box())
holeWallsExtrude=extrude(holeWallsSketch,-(holeDepth+holeWallThickness*2),)
textAndHolesExtrude=extrude(textAndHolesSketch,-holeDepth)
bitHolder=Part()+[topAndSides,holeWallsExtrude]
bitHolder-=textAndHolesExtrude
cheEdges=bitHolder.edges()-topAndSides.edges()
startTime=reportPerf(startTime)
print(len(cheEdges),len([e for e in cheEdges if e.length<=10]))
cheEdges=[e for e in cheEdges if e.length>10]
# bitHolder=chamfer(cheEdges,.02 )
startTime=reportPerf(startTime)
hexPathDimsFace=(topAndSides.faces()|Axis.Z)[-2]
hexPathDimsXEdges=(hexPathDimsFace.edges()>Axis.X)[:2]
hexPathDimsYEdges=(hexPathDimsFace.edges()>Axis.Y)[:2]
print(hexPathDimsXEdges,statistics.mean([x.length for x in hexPathDimsXEdges]))
hexSweepCenter=hexPathDimsFace.center()
hexSweepCenter.Z=(topAndSides.faces()|Axis.Z)[0].center().Z
hexSlotPath=Pos(hexSweepCenter)*\
    Rectangle(statistics.mean([y.length for y in hexPathDimsYEdges])+hexSlotRad,statistics.mean([x.length for x in hexPathDimsXEdges])+hexSlotRad)
hexSlotProfile=Pos((hexSlotPath.vertices()>>Axis.X<<Axis.Y)[0].center())*Rotation((90,0,0))*RegularPolygon(radius=hexSlotRad,side_count=6)
hexSlotSweep=sweep(hexSlotProfile,hexSlotPath.wire(),transition=Transition.RIGHT )
bitHolder-=hexSlotSweep
show(hexSlotSweep,bitHolder,)
startTime=reportPerf(startTime)
