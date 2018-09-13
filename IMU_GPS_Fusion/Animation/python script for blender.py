import bpy
import os

sce = bpy.context.scene
ob = bpy.context.object
obj = bpy.data.objects["Cube"]

file_name = open("/Users/eunsunlee/Documents/NESL/UnderwaterSensorTag/IMU_GPS_Fusion/Animation/path_output_from_blender.txt", "w")
for f in range(sce.frame_start, sce.frame_end+1):
    sce.frame_set(f)

    if f <= 80: 
        print("frame:",f)
        file_name.write("F %i" % f)

        file_name.write(" " + str(obj.matrix_world.translation[0]))
        file_name.write(" " + str(obj.matrix_world.translation[1]))
        print(" " + str(obj.matrix_world.translation[2]), file  = file_name)

        file_name.write("Q %i" % f)
        file_name.write(" " + str(obj.matrix_world.to_euler()[0]))
        file_name.write(" " + str(obj.matrix_world.to_euler()[1]))
        print(obj.matrix_world.to_euler())
        print(" " + str(obj.matrix_world.to_euler()[2]), file = file_name)


file_name.close()
print("done")