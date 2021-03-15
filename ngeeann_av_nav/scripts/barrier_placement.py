from __future__ import print_function
import math
import numpy as np

r = input("Radius in metres: ")
angle = input("Angle Step [rad]: ")
start_angle = input("Start Angle [rad]: ")
final_angle = input("Final Angle [rad]: ")

print("\nRadius: ", r, "\nAngle: ", angle)

theta = 0

for n in np.arange(0, 2*math.pi, angle):
	x = r * math.cos(theta)
	y = r * math.sin(theta)
	theta = theta + angle
	print ('<model name=\'drc_practice_blue_cylinder\'>')
	print ('  <pose frame=\'\'>', x, ' ', y, ' ', 0, ' 0 0 -0 0</pose>', sep = '')
	print ('  <scale>1 1 1</scale>')
	print ('  <link name=\'link\'>')
	print ('    <pose frame=\'\'>', x, ' ', y, ' ', 0, ' 0 0 -0 0</pose>', sep = '')
	print ('    <velocity>0 0 0 0 -0 0</velocity>')
	print ('    <acceleration>0 0 0 0 -0 0</acceleration>')
	print ('    <wrench>0 0 0 0 -0 0</wrench>')
	print ('  </link>')
	print ('</model>')


