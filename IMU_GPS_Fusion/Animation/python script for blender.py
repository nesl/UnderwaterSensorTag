import bpy
import os
#ob = bpy.context.object
#for p in ob.data.splines.active.bezier_points:
#    print(p.co)


sce = bpy.context.scene
ob = bpy.context.object

file_name = open("/Users/eunsunlee/Documents/NESL/UnderwaterSensorTag/IMU_GPS_Fusion/Animation/path_output_from_blender.txt", "w")
for f in range(sce.frame_start, sce.frame_end+1):
    sce.frame_set(f)
    if f < 200: 
        file_name.write("F %i" % f)
        #print("F %i " % f, file = file_name)
        obj = bpy.data.objects["Cube"]
        file_name.write(" " + str(obj.matrix_world.translation[0]))
        file_name.write(" " + str(obj.matrix_world.translation[1]))
        print(" " + str(obj.matrix_world.translation[2]), file  = file_name)

#    for i in sce.objects: #bpy.data.objects:
#        #fi.write(str(i.name))
#        print(str(i.name), file = file_name)
#        print(str(i.location), file = file_name)
    
#    print(
#    for pbone in ob.pose.bones:
#        print(pbone.name, pbone.matrix) # bone in object space

#print("hi")
#print ("F %f %f %f" % \
#            tuple(states[i] + [scanner_displacement * cos(states[i][2]),
#                               scanner_displacement * sin(states[i][2]),
#                               0.0]), file = f)

file_name.close()
print("done")