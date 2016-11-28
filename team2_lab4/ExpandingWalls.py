
''' From Lab 3 '''


def publishCells(grid):
	global pub
	#print "publishing"

	# resolution and offset of the map
	k=-2
	cells = GridCells()
	cells.header.frame_id = 'map'
	cells.cell_width = resolution 
	cells.cell_height = resolution

	for i in range(0,height): #height should be set to height of grid
		for j in range(0,width): #width should be set to width of grid
			#print k # used for debugging
			if (grid[i*width+j] == 100):
				point=Point()
				point.x=(j*resolution)+offsetX + (.5 * resolution)
				point.y=(i*resolution)+offsetY + (.5 * resolution)
				point.z=0
				cells.cells.append(point)
	pub.publish(cells)	






	''' # How the Function is called
	while not rospy.is_shutdown():
		publishCells(mapData) #publishing map data every 2 seconds
		try:
			print "initializing"
			Waypoints()
			#retVal = aStar((3, 3), (7, 5), mapGrid)
			#if(retVal != None):
				#print "Publishing path"
		#		pubpath.publish(retVal)
			
		except NameError:
			rospy.sleep(.1)
		rospy.sleep(2)
	'''


''' From Lab 4 '''


def expand(map_data):












# An example of the generated path
def rvizPath(cell_list, worldMap):
global path_pub

path_GC = GridCells()
path_GC.cell_width = worldMap.info.resolution
path_GC.cell_height = worldMap.info.resolution
path_GC.cells = []
for cell in cell_list:
	path_GC.cells.append(gridToWorld(cell, worldMap))
path_GC.header.frame_id = 'map'
path_pub.publish(path_GC)