#!/usr/bin/env python

# Author: Leon Kozinakov
import sys

# open input/output files

#inputfile = open('aquamote_fullmem_20180122_220229.log', 'r') #big file 256000baud
#inputfile = open('aquamote_fullmem_20180125_001106.log', 'r') #big file 115200baud
inputfile = open('testfile_20180122_202017.log', 'r') #small test file
outputfile = open('output.csv', 'w')

# The variable "lines" is a list containing all lines
lines = inputfile.readlines()

linecounter = 0
found = 0
for line in lines:
	if "Retreiving from: 0x" in line:
		outputfile.write("Aquamote MAC: ")
		outputfile.write(line.split()[2])
		outputfile.write("\n")
		found = 1
		break
	linecounter += 1

if(found == 0):
	print("Error: Couldn't find MAC address of originating device.")
	inputfile.close()
	outputfile.close()
	sys.exit()

linecounter2 = 0
for line in lines:
	if(linecounter2-1 > linecounter):
		outputfile.write(line.split()[1])
		outputfile.write(" Pages\n##################################################################\n")
		numPages = int(line.split()[1])
		break
	linecounter2 += 1
	
linecounter3 = 0
chunkCounter = 0
currentPage = 0
found = 0
emptyChunks = 0
concatenatedPage = ""
concatenatedLines = 0
runOnce = 0
type_of_page = 0 # 0 for empty, 1 for initial values, and 2 for data page
for line in lines:
	if(linecounter3 > linecounter2):
		if "END OF TRANSMISSION" in line:
			found = 1
			break
		if(len(line) > 257):
			print("Error: Received chunk longer than 128 bytes")
			inputfile.close()
			outputfile.close()
			sys.exit()
		
		if "FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF" in line:
			type_of_page = 0
			if(emptyChunks == 0):
				tempCounter = linecounter3
			emptyChunks += 1
			if(emptyChunks == 16): 						#Check if there were 16 empty chunks (1 page)
				if(linecounter3 == (tempCounter + 15)): #Check if the 16 chunks were consecutive
					outputfile.write("Empty Page\n")
				else:
					print("The 16 empty chunks were not consecutive, possible data error")
					inputfile.close()
					outputfile.close()
					sys.exit()
				emptyChunks = 0
				type_of_page = 1
				chunkCounter += 1
				continue

		if(type_of_page == 1):
			if(runOnce == 0):
				initialValues = []
				tempCounter2 = linecounter3
				outputfile.write("Initial Values: ")
				for x in range(0, 7): #executes 8 times
					initialValues.append(int(line[(4*x):(4*x+3)], 16))
				initialValues = ','.join(map(str, initialValues)) # Insert commas between values
				outputfile.write(initialValues)
				outputfile.write("\n")
				runOnce = 1
			if(linecounter3 == (tempCounter2 + 15)):
				runOnce = 0
				type_of_page = 2
				chunkCounter += 1
				continue
		if(type_of_page == 2):
			#normal page of data
			#first, concatenate 16 chunks into one long string, then process the whole thing
			concatenatedPage += line[0:256]
			concatenatedLines += 1
			if(concatenatedLines == 16):
				#Process the whole concatenated page of data
				concatenatedLines = 0
				if(len(concatenatedPage) != 4096):
					print("Error: Concatenated page is not of length 4096")
					print(len(concatenatedPage))
					print(concatenatedPage)
					inputfile.close()
					outputfile.close()
					sys.exit()
				if(concatenatedPage[4088:4095] == "47470D0A"):
					outputfile.write("Bad Page: Missing ending string\n")
				else:
					timecount = []
					seconds = []
					fraction = []
					
					accelValsX = []
					gyroValsX = []
					magValsX = []
					accelValsY = []
					gyroValsY = []
					magValsY = []
					accelValsZ = []
					gyroValsZ = []
					magValsZ = []
					pressTempVals = []

					for x in range(0,7):
						timecount.append(int(concatenatedPage[0:4], 16))
						concatenatedPage = concatenatedPage[4:]
						for y in range(0,2):
							for z in range(0,4):
								#A
								accelValsX.append(int(concatenatedPage[0:4], 16))
								outputfile.write(concatenatedPage[0:4] + ",")
								concatenatedPage = concatenatedPage[4:]
								accelValsY.append(int(concatenatedPage[0:4], 16))
								outputfile.write(concatenatedPage[0:4] + ",")
								concatenatedPage = concatenatedPage[4:]
								accelValsZ.append(int(concatenatedPage[0:4], 16))
								outputfile.write(concatenatedPage[0:4] + ",")
								concatenatedPage = concatenatedPage[4:]
								#G
								gyroValsX.append(int(concatenatedPage[0:4], 16))
								outputfile.write(concatenatedPage[0:4] + ",")
								concatenatedPage = concatenatedPage[4:]
								gyroValsY.append(int(concatenatedPage[0:4], 16))
								outputfile.write(concatenatedPage[0:4] + ",")
								concatenatedPage = concatenatedPage[4:]
								gyroValsZ.append(int(concatenatedPage[0:4], 16))
								outputfile.write(concatenatedPage[0:4] + ",")
								concatenatedPage = concatenatedPage[4:]
								if(z <= 2):
									outputfile.write("Null,Null,Null,Null,Null,Null,\n")
							#M
							magValsX.append(int(concatenatedPage[0:4], 16))
							outputfile.write(concatenatedPage[0:4] + ",")
							concatenatedPage = concatenatedPage[4:]
							magValsY.append(int(concatenatedPage[0:4], 16))
							outputfile.write(concatenatedPage[0:4] + ",")
							concatenatedPage = concatenatedPage[4:]
							magValsZ.append(int(concatenatedPage[0:4], 16))
							outputfile.write(concatenatedPage[0:4] + ",")
							concatenatedPage = concatenatedPage[4:]
							outputfile.write("Null,Null,Null\n")
						for y in range(0,2):
							#A
							accelValsX.append(int(concatenatedPage[0:4], 16))
							outputfile.write(concatenatedPage[0:4] + ",")
							concatenatedPage = concatenatedPage[4:]
							accelValsY.append(int(concatenatedPage[0:4], 16))
							outputfile.write(concatenatedPage[0:4] + ",")
							concatenatedPage = concatenatedPage[4:]
							accelValsZ.append(int(concatenatedPage[0:4], 16))
							outputfile.write(concatenatedPage[0:4] + ",")
							concatenatedPage = concatenatedPage[4:]
							#G
							gyroValsX.append(int(concatenatedPage[0:4], 16))
							outputfile.write(concatenatedPage[0:4] + ",")
							concatenatedPage = concatenatedPage[4:]
							gyroValsY.append(int(concatenatedPage[0:4], 16))
							outputfile.write(concatenatedPage[0:4] + ",")
							concatenatedPage = concatenatedPage[4:]
							gyroValsZ.append(int(concatenatedPage[0:4], 16))
							outputfile.write(concatenatedPage[0:4] + ",")
							concatenatedPage = concatenatedPage[4:]
							if(y <= 0):
								outputfile.write("Null,Null,Null,Null,Null,Null,\n")
						#PT
						outputfile.write("Null,Null,Null,")
						pressTempVals.append(int(concatenatedPage[0:4], 16))
						outputfile.write(concatenatedPage[0:4] + ",")
						concatenatedPage = concatenatedPage[4:]
						pressTempVals.append(int(concatenatedPage[0:4], 16))
						outputfile.write(concatenatedPage[0:4] + ",")
						concatenatedPage = concatenatedPage[4:]
						pressTempVals.append(int(concatenatedPage[0:4], 16))
						outputfile.write(concatenatedPage[0:4] + ",")
						concatenatedPage = concatenatedPage[4:]
						outputfile.write("\n")
						#TODO implement
						for y in range(0,2):
							#A
							accelValsX.append(int(concatenatedPage[0:4], 16))
							outputfile.write(concatenatedPage[0:4] + ",")
							concatenatedPage = concatenatedPage[4:]
							accelValsY.append(int(concatenatedPage[0:4], 16))
							outputfile.write(concatenatedPage[0:4] + ",")
							concatenatedPage = concatenatedPage[4:]
							accelValsZ.append(int(concatenatedPage[0:4], 16))
							outputfile.write(concatenatedPage[0:4] + ",")
							concatenatedPage = concatenatedPage[4:]
							#G
							gyroValsX.append(int(concatenatedPage[0:4], 16))
							outputfile.write(concatenatedPage[0:4] + ",")
							concatenatedPage = concatenatedPage[4:]
							gyroValsY.append(int(concatenatedPage[0:4], 16))
							outputfile.write(concatenatedPage[0:4] + ",")
							concatenatedPage = concatenatedPage[4:]
							gyroValsZ.append(int(concatenatedPage[0:4], 16))
							outputfile.write(concatenatedPage[0:4] + ",")
							concatenatedPage = concatenatedPage[4:]
							if(y <= 0):
								outputfile.write("Null,Null,Null,Null,Null,Null,\n")
						#M
						magValsX.append(int(concatenatedPage[0:4], 16))
						outputfile.write(concatenatedPage[0:4] + ",")
						concatenatedPage = concatenatedPage[4:]
						magValsY.append(int(concatenatedPage[0:4], 16))
						outputfile.write(concatenatedPage[0:4] + ",")
						concatenatedPage = concatenatedPage[4:]
						magValsZ.append(int(concatenatedPage[0:4], 16))
						outputfile.write(concatenatedPage[0:4] + ",")
						concatenatedPage = concatenatedPage[4:]
						outputfile.write("Null,Null,Null\n")
						for y in range(0,2):
							for z in range(0,4):
								#A
								accelValsX.append(int(concatenatedPage[0:4], 16))
								outputfile.write(concatenatedPage[0:4] + ",")
								concatenatedPage = concatenatedPage[4:]
								accelValsY.append(int(concatenatedPage[0:4], 16))
								outputfile.write(concatenatedPage[0:4] + ",")
								concatenatedPage = concatenatedPage[4:]
								accelValsZ.append(int(concatenatedPage[0:4], 16))
								outputfile.write(concatenatedPage[0:4] + ",")
								concatenatedPage = concatenatedPage[4:]
								#G
								gyroValsX.append(int(concatenatedPage[0:4], 16))
								outputfile.write(concatenatedPage[0:4] + ",")
								concatenatedPage = concatenatedPage[4:]
								gyroValsY.append(int(concatenatedPage[0:4], 16))
								outputfile.write(concatenatedPage[0:4] + ",")
								concatenatedPage = concatenatedPage[4:]
								gyroValsZ.append(int(concatenatedPage[0:4], 16))
								outputfile.write(concatenatedPage[0:4] + ",")
								concatenatedPage = concatenatedPage[4:]
								if(z <= 2):
									outputfile.write("Null,Null,Null,Null,Null,Null,\n")
							#M
							magValsX.append(int(concatenatedPage[0:4], 16))
							outputfile.write(concatenatedPage[0:4] + ",")
							concatenatedPage = concatenatedPage[4:]
							magValsY.append(int(concatenatedPage[0:4], 16))
							outputfile.write(concatenatedPage[0:4] + ",")
							concatenatedPage = concatenatedPage[4:]
							magValsZ.append(int(concatenatedPage[0:4], 16))
							outputfile.write(concatenatedPage[0:4] + ",")
							concatenatedPage = concatenatedPage[4:]
							if(y <= 0):
								outputfile.write("Null,Null,Null\n")
						#PT
						pressTempVals.append(int(concatenatedPage[0:4], 16))
						outputfile.write(concatenatedPage[0:4] + ",")
						concatenatedPage = concatenatedPage[4:]
						pressTempVals.append(int(concatenatedPage[0:4], 16))
						outputfile.write(concatenatedPage[0:4] + ",")
						concatenatedPage = concatenatedPage[4:]
						pressTempVals.append(int(concatenatedPage[0:4], 16))
						outputfile.write(concatenatedPage[0:4] + ",")
						concatenatedPage = concatenatedPage[4:]
						#seconds
						seconds.append(int(concatenatedPage[0:7], 16))
						outputfile.write(str(int(concatenatedPage[0:8], 16)) + ",")
						concatenatedPage = concatenatedPage[8:]
						#fraction
						fraction.append(int(concatenatedPage[0:7], 16))
						outputfile.write(str(int(concatenatedPage[0:8], 16)) + ",\n")
						concatenatedPage = concatenatedPage[8:]
					#then reset for next time
					concatenatedPage = ""
		chunkCounter += 1
	linecounter3 += 1

if(found == 1):
	#TODO Do further checks
	if((chunkCounter/16.0) == float(numPages)):
		print("Finished")
	else:
		print("Error: Total number of pages received doesn't match what's expected")
else:
	print("No EOT string found, file may be incomplete")

# close the file after reading the lines.
inputfile.close()
outputfile.close()