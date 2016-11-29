''' Make sure to add the global pub_frontier to the lsit of globals at the bottom and top '''

def publishFrontier(grid):
	global pub_frontier
		# resolution and offset of the map
	k=0
	cells = GridCells()
	cells.header.frame_id = 'map'
	cells.cell_width = resolution 
	cells.cell_height = resolution

	for node in grid:
		point=Point()
		point = worldToGrid(worldPoint, worldMap)
		cells.cells.append(point)
	pub_frontier.publish(cells)




#Publisher
#pub_frontier = rospy.Publisher('map_frontier', GridCells, queue_size=1)