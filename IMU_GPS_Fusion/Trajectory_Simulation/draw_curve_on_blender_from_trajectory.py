import bpy
import os


coords = []
frame_no = 0

fileread = open("/Users/eunsunlee/Documents/NESL/UnderwaterSensorTag/IMU_GPS_Fusion/Trajectory_Simulation/simulated_trajectories/trajectory_simulation_test.txt","r")
#fileread = open("/Users/eunsunlee/Documents/NESL/UnderwaterSensorTag/IMU_GPS_Fusion/Trajectory/coordinate_from_penguin.txt","r")
k = 0 
for i in fileread:
    sp = i.split()
    coord = [float(sp[1]), float(sp[2]), float(sp[3])]
    if k == 0:
        coords.append(coord)
        coords.append(coord)
        k = k + 1
    print(i)
    coords.append(coord)
    #add coordinate one more time to strictly stick with the location
    #coords.append([float(sp[1]), float(sp[2]), float(sp[3])])
    frame_no = int(sp[0])
   
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

print(frame_no)
bpy.ops.mesh.primitive_cube_add(location=(0,0,0))

# Align cube to the path 
# follow path 
    # outliner
    # click cube + shift
    # click path 
    # ctrl + p 
# object -> select curve
# object data -> path animation/frames -> frame_no
