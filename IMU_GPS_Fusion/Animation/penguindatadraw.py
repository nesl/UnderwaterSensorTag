import bpy
import os


coords = []

fileread = open("/Users/eunsunlee/Documents/NESL/UnderwaterSensorTag/IMU_GPS_Fusion/Trajectory/coordinate_from_penguin.txt","r")
for i in fileread:
    sp = i.split()
    coords.append([float(sp[0]), float(sp[1]), float(sp[2])])

fileread.close()


# sample data
#coords = [(0,0,0),(0,0,0),(0,0,0),(1,1,1),(1,1,1),(1,1,1),(2,2,2),(2,2,2),(2,2,2), (3,3,3)]

# create the Curve Datablock
curveData = bpy.data.curves.new('myCurve', type='CURVE')
curveData.dimensions = '3D'
curveData.resolution_u = 2

# map coords to spline
polyline = curveData.splines.new('NURBS')
polyline.points.add(len(coords))
#polyline = curveData.splines.new('BEZIER')
#polyline.bezier_points.add(len(coords))

for i, coord in enumerate(coords):
    x,y,z = coord
    polyline.points[i].co = (x, y, z, 1)
#    polyline.bezier_points[i].co = (x,y,z)

# create Object
curveOB = bpy.data.objects.new('myCurve', curveData)

# attach to scene and validate context
scn = bpy.context.scene
scn.objects.link(curveOB)
scn.objects.active = curveOB
curveOB.select = True