def saveTrajectory(trajectory, samplePeriod, endtime): 
	time = trajectory.startTime
	out_file = file("trajectory.txt", "w")
	while time <= endtime: 
		px = trajectory.position(time)[0][0]
		py = trajectory.position(time)[1][0]
		pz = trajectory.position(time)[2][0]
		vx = trajectory.velocity(time)[0][0]
		vy = trajectory.velocity(time)[1][0]
		vz = trajectory.velocity(time)[2][0]
		ax = trajectory.acceleration(time)[0][0]
		ay = trajectory.acceleration(time)[1][0]
		az = trajectory.acceleration(time)[2][0]

		print >>out_file, time, px, py, pz, vx, vy, vz, ax, ay, az
		time = time + samplePeriod

	out_file.close()

def saveIMU(imu):
	acc = imu.accelerometer.rawMeasurements
	gyro = imu.gyroscope.rawMeasurements
	mag = imu.magnetometer.rawMeasurements
	out_file = file("imu_values.txt","w")

	for i in range(len(acc.timestamps)):
		time = acc.timestamps[i]
		ax = acc.values[0][i]
		ay = acc.values[1][i]
		az = acc.values[2][i]
		gx = gyro.values[0][i]
		gy = gyro.values[1][i]
		gz = gyro.values[2][i]
		mx = mag.values[0][i]
		my = mag.values[1][i]
		mz = mag.values[2][i]
		print >> out_file, time, ax, ay, az, gx, gy, gz, mx, my, mz

