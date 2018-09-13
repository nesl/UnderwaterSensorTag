from imusim.all import * 
from log import saveIMU, saveTrajectory

def readtrajectory():
	filename = open("path_output_from_blender.txt", "r")
	times = []	
	x = []
	y = []
	z = []
	rotation = []
	timestamp = 0
	prev_val = 0.5
	sigma = 0.05
	for i in filename: 
		sp = i.split()
	
			
		if sp[0] == 'F':
			times.append(float(sp[1]))
			x.append(float(sp[2]))
			y.append(float(sp[3]))
			z.append(float(sp[4]))
			s = np.random.normal(prev_val, sigma, 1)[0]
			if (s-sigma > 0.3) & (s+sigma<0.7):
				prev_val = s
			else: 
				prev_val = prev_val
			timestamp = timestamp + s
			#times.append(float(timestamp))	
		if sp[0] == 'Q':
			ex = float(sp[1])
			ey = float(sp[2])
			ez = float(sp[4])
			q = Quaternion.fromEuler((ex,ey,ez),'xyz')
			
			#qw = float(sp[2])
			#qx = float(sp[3])
			#qy = float(sp[4])
			#qz = float(sp[5])
			rotation.append(q)
	times = np.array(times)
	positions = np.array([x,y,z])
	rotations = QuaternionArray(np.array(rotation))
	return times, positions, rotations

def runSimulation(endtime): 
	samplingPeriod = 0.025
	times, positions, rotations = readtrajectory()
	sampledTrajectory = SampledTrajectory.fromArrays(times,positions,rotations)
	trajectory = SplinedTrajectory(sampledTrajectory)
	sim = Simulation()
	imu = IdealIMU(sim,trajectory)
	behaviour = BasicIMUBehaviour(imu,samplingPeriod)
	sim.time = trajectory.startTime
	sim.run(endtime)
	saveIMU(imu)
	saveTrajectory(trajectory,samplingPeriod,endtime)
	
	
