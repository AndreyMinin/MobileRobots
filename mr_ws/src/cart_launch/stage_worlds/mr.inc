
define mr_base position
(
  color "green"			# Default color.
  drive "diff"		 	# Differential steering model.
  gui_nose 1                  	# Draw a nose on the robot so we can see which way it points
  obstacle_return 1           	# Can hit things.
  ranger_return 0.5            	# reflects sonar beams
  blob_return 1               	# Seen by blobfinders
  fiducial_return 1           	# Seen as "1" fiducial finders

  localization "gps"
  localization_origin [0 0 0 0] 	# Start odometry at (0, 0, 0).

  # alternative odometric localization with simple error model
  # localization "odom"         		# Change to "gps" to have impossibly perfect, global odometry
  #odom_error [ 0.5 0.5 0.1 0.1]   	# Odometry error or slip in X, Y and Theta
                                   	# (Uniform random distribution)

  # four DOF kinematics limits
  # [ xmin xmax ymin ymax zmin zmax amin amax ]
  velocity_bounds [-20 20 0 0 0 0 -90.0 90.0 ]
  acceleration_bounds [-1.5 1.5 0 0 0 0 -90 90.0 ]
)

define mr mr_base
(
	size [2.5 1.5 1.0]
	origin [ 0 0 0.4 0 ]
	mass 100
	#main body
	block
	(
		points 4
		point[0] [ -1.25  0.75 ]
		point[1] [  1.25  0.75 ]
		point[2] [  1.25 -0.75 ]
		point[3] [ -1.25 -0.75 ]
        z [ 0.2 0.9 ]
	)

	#right fwheel
	block
	(
		points 4
		point[0] [ 1.2  0.6 ]
		point[1] [ 0.7  0.6 ]
		point[2] [ 0.7  0.85 ]
		point[3] [ 1.2  0.85 ]
        z [ 0 0.5 ]
        color "gray15"
	)

    #left fwheel
	block
	(
		points 4
		point[0] [ 1.2  -0.6 ]
		point[1] [ 0.7  -0.5 ]
		point[2] [ 0.7  -0.85 ]
		point[3] [ 1.2  -0.85 ]
        z [ 0 0.5 ]
        color "gray15"
	)

	#right bwheel
	block
	(
		points 4
		point[0] [ -1.2  0.6 ]
		point[1] [ -0.7  0.6 ]
		point[2] [ -0.7  0.85 ]
		point[3] [ -1.2  0.85 ]
        z [ 0 0.5 ]
        color "gray15"
	)

    #left bwheel
	block
	(
		points 4
		point[0] [ -1.2  -0.6 ]
		point[1] [ -0.7  -0.5 ]
		point[2] [ -0.7  -0.85 ]
		point[3] [ -1.2  -0.85 ]
        z [ 0 0.5 ]
        color "gray15"
	)
)

# a Pioneer 2 or 3 in standard configuration
define fancypioneer2dx pioneer2dx_base
(
  # this set of blocks approximates the shape of a real Pioneer

  #  The geometry is from the Webots v5.3.0 manual. Thanks to Webots
  #  and Olivier Michel. If Stage or Gazebo do not do what you want,
  #  take a look at Webots. It's a very nice commercial simulator.

  # main body
  block
  (
    points 8
    point[0] [ -0.185 -0.135 ]
    point[1] [  0.095 -0.135 ]
    point[2] [  0.11  -0.08  ]
    point[3] [  0.11   0.08  ]
    point[4] [  0.095  0.135 ]
    point[5] [ -0.185  0.135 ]
    point[6] [ -0.215  0.1   ]
    point[7] [ -0.215 -0.1   ]
    z [ 0.059 0.234 ]
  )

  # sonar case
  block
  (
    points 9
    point[0]  [ -0.135  0.136 ]
    point[1]  [ -0.185  0.136 ]
    point[2]  [ -0.223  0.101 ]
    point[3]  [ -0.248  0.054 ]
    point[4]  [ -0.258  0     ]
    point[5]  [ -0.248 -0.054 ]
    point[6]  [ -0.223 -0.101 ]
    point[7]  [ -0.185 -0.136 ]
    point[8]  [ -0.135 -0.136 ]
    z [ 0.184 0.234 ]
  )

  # sonar case
  block
  (
    points 9
    point[0]  [ 0.046 -0.136 ]
    point[1]  [ 0.096 -0.136 ]
    point[2]  [ 0.134 -0.101 ]
    point[3]  [ 0.159 -0.054 ]
    point[4]  [ 0.168  0     ]
    point[5]  [ 0.159  0.054 ]
    point[6]  [ 0.134  0.101 ]
    point[7]  [ 0.096  0.136 ]
    point[8]  [ 0.046  0.136 ]
    z [ 0.184 0.234 ]
  )

  # left wheel
  block
  (
    points 4
    point[0] [  0.083  0.177 ]
    point[1] [ -0.083  0.177 ]
    point[2] [ -0.083  0.140 ]
    point[3] [  0.083  0.140 ]
    z [0 0.165 ]
    color "gray15"
  )

  # right wheel
  block
  (
    points 4
    point[0] [  0.083  -0.14 ]
    point[1] [ -0.083  -0.14 ]
    point[2] [ -0.083  -0.177 ]
    point[3] [  0.083  -0.177 ]
    z [ 0 0.165 ]
    color "gray15"
  )

  # castor
  block
  (
    points 4
    point[3] [ -0.2475  0.012 ]
    point[2] [ -0.1825  0.012 ]
    point[1] [ -0.1825 -0.012 ]
    point[0] [ -0.2475 -0.012 ]
    z [ 0 0.065 ]
    color "gray15"
  )

  # lid
  block
  (
    points 22
    point[21] [  0.174 0 ]
    point[20] [  0.166 -0.056 ]
    point[19] [  0.145 -0.107 ]
    point[18] [  0.112 -0.155 ]
    point[17] [  0.064 -0.190 ]
    point[16] [  -0.074 -0.190 ]
    point[15] [  -0.096 -0.160 ]
    point[14] [  -0.151 -0.160 ]
    point[13] [  -0.2   -0.155 ]
    point[12] [  -0.236 -0.107 ]
    point[11] [  -0.256 -0.056 ]
    point[10] [  -0.264  0     ]
    point[9] [  -0.256  0.056 ]
    point[8] [ -0.236  0.107 ]
    point[7] [ -0.2    0.155 ]
    point[6] [ -0.151  0.160 ]
    point[5] [ -0.096  0.160 ]
    point[4] [ -0.074  0.190 ]
    point[3] [  0.064  0.190 ]
    point[2] [  0.112  0.155 ]
    point[1] [  0.145  0.107 ]
    point[0] [  0.166  0.056 ]
    z [ 0.234 0.24 ]

    # a dark top looks more realistic, but isn't very useful
    # for a top-down view
    #color "gray10"
  )
)



# The Pioneer3DX standard configuration
define pioneer3dx pioneer_base
(
  # Actual size
  size [0.511 0.4 0.22 ]

  # The pioneer's center of rotation is offset from its center of area
  origin [-0.04465 0.0 0.0]

  # Estimated mass in KG
  mass 23.0

  # Body shape:
  block(
    points 8
    point[0] [-0.2 0.12]
    point[1] [-0.2 -0.12]
    point[2] [-0.12 -0.2555]
    point[3] [0.12 -0.2555]
    point[4] [0.2 -0.12]
    point[5] [0.2 0.12]
    point[6] [0.12 0.2555]
    point[7] [-0.12 0.2555]
    z [0 0.22]
  )

  # Use the sonar array defined above
  p3dx_sonar( pose [ 0 0 -0.03 0 ] )
)

# The Pioneer3AT standard configuration
define pioneer3at pioneer_base
(
  # Actual size
  size [0.626 0.505]

  # The pioneer's center of rotation is offset from its center of area
  origin [-0.04465 0.0 0.0]

  # Estimated mass in KG
  mass 40.0

  # Body shape:
  block(
    points 8
    point[0] [-0.18 0.313]
    point[1] [0.18 0.313]
    point[2] [0.2525 0.18]
    point[3] [0.2525 -0.18]
    point[4] [0.18 -0.313]
    point[5] [-0.18 -0.313]
    point[6] [-0.2525 -0.18]
    point[7] [-0.2525 0.18]
  )

  # Use the sonar array defined above
  p3at_sonar( pose [ 0 0 -0.03 0 ] )
)

