import rospy
import tf
from nav_msgs.msg import Odometry
from aruco_msgs.msg import MarkerArray
import numpy as np
import matplotlib.pyplot as plt
import time
import copy


#Ponto de partida (0,0) - Porta da sala
#Marker de id 0 tem coordenadas (x_map0,y_map0) em metros

x_map=[0.0]*41;
y_map=[0.0]*41;

x_map[0]=0.097+0.385;
y_map[0]=0.945;
x_map[1]=x_map[0]+0.77;
y_map[1]=y_map[0];
x_map[2]=x_map[1]+0.70;
y_map[2]=y_map[0];
x_map[3]=x_map[2]+2.495;
y_map[3]=y_map[0];
x_map[4]=x_map[3]+3.00;
y_map[4]=y_map[0];
x_map[5]=x_map[4]+3.00;
y_map[5]=y_map[0];
x_map[6]=x_map[5]+2.74;
y_map[6]=y_map[0];
x_map[7]=x_map[6]+2.643;
y_map[7]=y_map[0]-0.20;
x_map[8]=x_map[7];
x_map[9]=x_map[7];
x_map[10]=x_map[7];
x_map[11]=x_map[7];
y_map[8]=y_map[7]-1.205;
y_map[9]=y_map[8]-1.17;
y_map[10]=y_map[8]-4.84;
y_map[11]=y_map[10]-6.93;
y_map[12]=y_map[11]-2.55;
x_map[12]=x_map[11]-0.27;
y_map[13]=y_map[14]=y_map[15]=y_map[16]=y_map[17]=y_map[12];
x_map[13]=x_map[12]-0.60;
x_map[14]=x_map[13]-0.95;
x_map[15]=x_map[14]-4.095;
x_map[16]=x_map[15]-3.005;
x_map[17]=x_map[16]-4.20;
x_map[18]=x_map[19]=x_map[23]=x_map[30]=x_map[20]=x_map[21]=0.097;
x_map[22]=0;
y_map[22]=y_map[18]+0.67;
y_map[18]=y_map[17]+0.22;
y_map[19]=y_map[18]+0.67+0.615;
y_map[23]=y_map[19]+0.55;
y_map[30]=y_map[23]+0.70;
y_map[20]=y_map[23]+3.93;
y_map[21]=y_map[20]+5.895;
x_map[24]=1.67+0.097;
y_map[24]=-0.84;
x_map[25]=x_map[4];
y_map[25]=y_map[4]-1.575;
x_map[26]=x_map[9]-1.235;
y_map[26]=y_map[9];
x_map[28]=x_map[26]+0.10;
y_map[28]=y_map[26]-2.00;
x_map[27]=x_map[26];
y_map[27]=y_map[26]-2.00-4.21;
x_map[29]=x_map[14]-1.45;
y_map[29]=y_map[14]+0.20;

# Markers da zona dos elevadores

x_map[31]=x_map[21]+1.79;
y_map[31]=y_map[21]+0.095;
x_map[33]=x_map[31]+2.15;
y_map[33]=y_map[31]+0.10;
y_map[37]=y_map[31];
x_map[37]=x_map[33]+1.00;
x_map[40]=x_map[37]+1.00;
y_map[40]=y_map[37]-1.10;
x_map[39]=x_map[37]+2.65;
y_map[39]=y_map[37]+0.35;
x_map[38]=x_map[39];
y_map[38]=y_map[39]-2.66;
y_map[36]=y_map[34]=y_map[35]=y_map[38];
x_map[36]=x_map[38]-1.46;
x_map[34]=x_map[36]-1.96;
x_map[35]=x_map[34]-1.46;
y_map[32]=y_map[35]+0.28;
x_map[32]=x_map[35]-0.68-0.28;


camara_distance_z = 0.12 # 15.5 cm  <-> 13 cm #dia 13/12/2018 <-> 12 cm => 12.5 cm   inicio a 81 cm
camara_distance_x = 0.011 # 1.1 cm

# Constants
NUMBER_MARKERS = 41
KEY_NUMBER = 2**(5*5) # number of total combinations possible in aruco code
number_of_dimensions = 2
Frequency = 9.5

NUMBER_PARTICLES = 100
translation_noise = 0.1
rotation_noise = 0.1
noise_factor = 1
minimum_move = 0
Sensor_noise = 0.1
validity_threshold = 50

circle = np.arange(0, 2*np.pi, 0.1)
o_size = 0.3
line = np.arange(0, o_size, o_size)

fig, ax = plt.subplots()
robot_line, = ax.plot([0], [0], color='black', marker='o', markersize=12)
robot_orientation, = ax.plot(line, line, color='lime', marker='.', markersize=2, linewidth=2)
marker_line, = ax.plot(circle, circle, color='red', marker='.', markersize=8, linestyle="")
robot_path, = ax.plot([0], [0], color='black', marker='.', markersize=2, linewidth=0.2)
path_map, = plt.plot(x_map, y_map, color='grey', marker='*', markersize=8, linestyle="")

x_f = [circle]*NUMBER_MARKERS
y_f = [circle]*NUMBER_MARKERS

plt.ion()
plt.xlim(-10, 20)
plt.ylim(-20, 10)
plt.xlabel('X', fontsize=10)  # X axis label
plt.ylabel('Y', fontsize=10)  # Y axis label
plt.title('FastSlam')
#plt.legend()
plt.grid(True)  # Enabling gridding

def drawing_plot(particles):

	Max = 0
	Max_id = 0

	for i in range(NUMBER_PARTICLES):
		if particles[i].get_weight() > Max:
			Max = particles[i].get_weight()
			Max_id = i

	pose = particles[Max_id].get_position()
	x = pose[0]
	y = pose[1]
	o = pose[2]
	x_o = x + o_size*np.cos(o)
	y_o = y + o_size*np.sin(o)
	x_path, y_path = particles[Max_id].get_path()

	plt.show(block=False)

	robot_path.set_xdata(x_path)
	robot_path.set_ydata(y_path)
	ax.draw_artist(ax.patch)
	ax.draw_artist(robot_path)

	robot_line.set_xdata(x)
	robot_line.set_ydata(y)
	ax.draw_artist(ax.patch)
	ax.draw_artist(robot_line)

	robot_orientation.set_xdata([x, x_o])
	robot_orientation.set_ydata([y, y_o])
	ax.draw_artist(ax.patch)
	ax.draw_artist(robot_orientation)

	Landmarkers = particles[Max_id].get_landmarkers()

	i = 0
	for marker in Landmarkers:
		if marker == None:
			x_f[i] = KEY_NUMBER + circle
			y_f[i] = KEY_NUMBER + circle
			i += 1
			continue
		pose_m = marker.get_marker_position()
		x_m = pose_m[0]
		y_m = pose_m[1]
		std_m = marker.get_marker_covariance()
		x_std_m = std_m[0][0]
		y_std_m = std_m[1][1]
		x_f[i] = x_m + x_std_m * np.cos(circle)
		y_f[i] = y_m + y_std_m * np.sin(circle)
		i += 1
	marker_line.set_xdata(x_f)
	marker_line.set_ydata(y_f)
	ax.draw_artist(ax.patch)
	ax.draw_artist(marker_line)
	fig.canvas.flush_events()


def resample_particles(particles, updated_marker):

	# Returns a new set of particles obtained by performing stochastic universal sampling, according to the particle weights.
	# distance between pointers
	step = 1.0/NUMBER_PARTICLES
	# random start of first pointer
	r = np.random.uniform(0,step)
	# where we are along the weights
	c = particles[0].get_weight()
	# index of weight container and corresponding particle
	i = 0
	index = 0

	new_particles = []

	#loop over all particle weights
	for particle in particles:
		#go through the weights until you find the particle
		u = r + index*step
		while u > c:
			i = i + 1
			c = c + particles[i].get_weight()

		#add that particle
		if i == index:
			new_particle = particles[i]
			new_particle.set_weight(step)
		else:
			new_particle = particles[i].copy(updated_marker)
		#new_particle = copy.deepcopy(particles[i])
		#new_particle.set_weight(step)
		new_particles.append(new_particle)
		#increase the threshold

		index += 1

	del particles

	return new_particles


class Particle():

	#each particle has a pose(x,y,o), a weight(w) and a series of kalman filters for every landmark
	#in the beggining all particles are in the origin frame of the world (0,0,0)

	def __init__(self):

		self.X_robot = np.array([0, 0, 0], dtype='float64').transpose()
		self.weight = 1.0/NUMBER_PARTICLES
		self.Landmarkers = [None]*NUMBER_MARKERS
		self.x_path = np.array([0], dtype='float64')
		self.y_path = np.array([0], dtype='float64')


	def get_kalman_filters(self, marker_id, Z):

		if self.Landmarkers[marker_id] == None:
			self.Landmarkers[marker_id] = KalmanFilter()

		return self.Landmarkers[marker_id]


	def particle_prediction(self, motion_model):

		#if the robot moves we just add the motion model to the previous pose to predict the particle position
		x = 0
		y = 1
		o = 2

		noise = np.array([np.random.normal(0,translation_noise), np.random.normal(0,translation_noise), np.random.normal(0,rotation_noise)], dtype='float64').transpose()
		noise = noise*motion_model*noise_factor

		self.X_robot = self.X_robot + motion_model + noise

		while self.X_robot[o] > np.pi:
			self.X_robot[o] = self.X_robot[o] - 2*np.pi
		while self.X_robot[o] < -np.pi:
			self.X_robot[o] = self.X_robot[o] + 2*np.pi

		self.x_path = np.insert(self.x_path, 0, self.X_robot[x])
		self.y_path = np.insert(self.y_path, 0, self.X_robot[y])

		return self.X_robot


	def update_weight(self, marker_id):

		std = self.Landmarkers[marker_id].get_marker_covariance()
		dev = self.Landmarkers[marker_id].get_marker_validity()

		fact = np.sqrt(np.linalg.det(2* np.pi * std))
		expo = - np.dot(dev.T, np.linalg.inv(std)).dot(dev)/2
		self.weight = self.weight / fact * np.exp(expo)

	def get_weight(self):
		return self.weight

	def normalize_weight(self, total_weight):
		self.weight = self.weight / total_weight

	def set_weight(self, new_weight):
		self.weight = new_weight

	def get_position(self):
		return self.X_robot

	def get_landmarkers(self):
		return self.Landmarkers

	def get_path(self):
		return self.x_path, self.y_path

	def copy(self, updated_marker):
		new_particle = Particle()

		del new_particle.x_path
		del new_particle.y_path
		del new_particle.Landmarkers
		del new_particle.X_robot 

		new_particle.x_path = np.copy(self.x_path)
		new_particle.y_path = np.copy(self.y_path)
		
		for i in range(len(self.Landmarkers)):
			if self.Landmarkers[i] != None and updated_marker[i] == True:
				self.Landmarkers[i] = self.Landmarkers[i].copy()

		new_particle.Landmarkers = self.Landmarkers
		new_particle.X_robot = np.copy(self.X_robot)

		return new_particle


		

class KalmanFilter():

	def __init__(self):

		# X_ espected value of landmarks' position (x,y)
		# X_robot (x, y, yaw)
		# G gradient of markers' relative position to robot (h:(x_m,y_m) -> (distance, orientation); G = dh/dX_ and X_ = X(k+1|k)) 
		# S covariance matrix markers' position
		# Q covariance matrix markers' measurement
		# V diference between measurement and estimated markers' position

		self.first = True
		self.Q_t = np.identity(number_of_dimensions, dtype='float64')*Sensor_noise #sensor noise


	def compute_G(self, X_robot):

		x = 0 # x position
		y = 1 # y position

		y = self.X_[y] - X_robot[y]
		x = self.X_[x] - X_robot[x]

		# compute G
		denominator = x**2 + y**2
		h11 = x / np.sqrt(denominator)
		h12 = y / np.sqrt(denominator)
		h21 = -y / denominator
		h22 = x / denominator
		self.G = np.array([[h11, h12], [h21, h22]])


	def Apply_EKF(self, X_robot, Z):

		x = 0 # x position
		y = 1 # y position
		o = 2 # o orientaio

		d = 0 # distance measured
		fi = 1 # orientaion of the measurement

		if self.first == True:
			# the angle is in the direction y to x, reverse of the usual x to y
			angle = (X_robot[o] + Z[fi])

			self.X_ = np.array([X_robot[x] + Z[d]*np.cos(angle), X_robot[y] + Z[d]*np.sin(angle)], dtype='float64').transpose() # first landmark position

			self.compute_G(X_robot)
			G_inv = np.linalg.inv(self.G)
			self.S = G_inv.dot(self.Q_t).dot(G_inv.T)

			self.V = np.array([0, 0], dtype='float64').transpose()
			self.Q = np.identity(number_of_dimensions, dtype='float64')

		else:
			# Prediction
			y = self.X_[y] - X_robot[y]
			x = self.X_[x] - X_robot[x]

			d = np.sqrt(x**2 + y**2) # distance
			fi = np.arctan2(y, x) - X_robot[o] # direction

			while fi > np.pi:
				fi = fi - 2*np.pi
			while fi < -np.pi:
				fi = fi + 2*np.pi

			Z_ = np.array([d, fi], dtype='float64').transpose()

			self.compute_G(X_robot)
			self.Q = self.G.dot(self.S).dot(self.G.T) + self.Q_t

			# Observation
			self.V = np.subtract(Z, Z_) # Z = [d, teta] 

	def Update(self):
		# Update
		if self.first == False:
			# K kalman gain
			K = self.S.dot(self.G.T).dot(np.linalg.inv(self.Q))
			#K = self.S.dot(self.G).dot(np.linalg.inv(self.Q))

			self.X_ = self.X_ + K.dot(self.V)
			self.S = (np.identity(number_of_dimensions)- K.dot(self.G)).dot(self.S)
			#self.S = (np.identity(number_of_dimensions)- K.dot(self.G.T)).dot(self.S)		
		else:
			self.first = False

	def get_marker_position(self):
		return self.X_

	def get_marker_covariance(self):
		return self.Q

	def get_marker_validity(self):
		return self.V

	def measurement_validition(self):
		return np.dot(self.V.T, np.linalg.inv(self.Q)).dot(self.V)

	def copy(self):
		new_KF = KalmanFilter()
		new_KF.X_ = np.copy(self.X_)
		new_KF.Q = np.copy(self.Q)
		new_KF.V = np.copy(self.V)
		new_KF.S = np.copy(self.S)
		new_KF.first = False

		return new_KF


class markers():

	def __init__(self):

		self.could_it_read = False
		self.z_distance_left_eye_to_robot_wheel = camara_distance_z
		self.x_distance_left_eye_to_robot_wheel = camara_distance_x
		self.markers_info = [None]*NUMBER_MARKERS
		self.list_ids = np.ones(NUMBER_MARKERS, dtype='int32')*KEY_NUMBER


	def callback_Markers(self, data):
		
		# static tf could be applied here: z = z + z_distance_left_eye_to_robot_wheel, x = x + x_distance_left_eye_to_robot_wheel
		for i in range(NUMBER_MARKERS):
			try:
				marker_info = data.markers.pop()
			except:
				break
		
			self.list_ids[i] = marker_info.id
			self.markers_info[marker_info.id] = marker_info


	def get_measerment(self, index):

		x = self.markers_info[index].pose.pose.position.x # right-left
		z = self.markers_info[index].pose.pose.position.z # front-back

		# position of the marker relative to base_link
		z = z + self.z_distance_left_eye_to_robot_wheel
		x = x + self.x_distance_left_eye_to_robot_wheel

		marker_distance = np.sqrt(z**2+x**2)
		marker_direction = np.arctan(x/z)

		return np.array([marker_distance, -marker_direction], dtype='float64').transpose()


	def get_list_ids(self):
		return self.list_ids

	def reset_list_ids(self):
		i = 0
		while self.list_ids[i] != KEY_NUMBER:
			self.list_ids[i] = KEY_NUMBER
			i += 1

	def marker_info(self, index):
		return self.markers_info[index]
		

class odom():

	def __init__(self):

		self.read_move = np.array([0, 0, 0], dtype='float64').transpose()
		self.first_read = True
				
	def callback_odom(self, data):

		# robo_frame
		frame_id = data.header.frame_id # odom
		child_frame_id = data.child_frame_id # base_link

		# pose
		x = data.pose.pose.position.x # front-back
		y = data.pose.pose.position.y # right-left
				
		orientation_x = data.pose.pose.orientation.x
		orientation_y = data.pose.pose.orientation.y
		orientation_z = data.pose.pose.orientation.z
		orientation_w = data.pose.pose.orientation.w
		roll, pitch, yaw = tf.transformations.euler_from_quaternion((orientation_x, orientation_y, orientation_z, orientation_w))

		if self.first_read == True:
			self.last_position = np.array([x, y, yaw], dtype='float64').transpose()
			self.total_movement = np.array([0, 0, 0], dtype='float64').transpose()
			self.first_read = False

		self.odom_position = np.array([x, y, yaw], dtype='float64').transpose()
		self.movement = np.subtract(self.odom_position, self.last_position)
		self.total_movement = np.add(self.total_movement, np.absolute(self.movement))

		if self.movement[2] > np.pi:
			self.movement[2] = 2*np.pi - self.movement[2]
		if self.movement[2] < -np.pi:
			self.movement[2] = - 2*np.pi - self.movement[2]
		self.last_position = self.odom_position
		self.read_move = np.add(self.read_move, self.movement)

	def actual_movement(self): 
		return self.read_move

	def get_movement(self):
		msg = self.read_move
		self.read_move = np.array([0, 0, 0], dtype='float64').transpose()
		return msg

	def get_total_movement(self):
		return self.total_movement


def FastSlam():

	odom_measurement = odom()
	
	particles = [Particle() for i in range(NUMBER_PARTICLES)]
	updated_marker = [False]*NUMBER_MARKERS

	marker_measurement = markers()

	rospy.init_node('FastSlam', anonymous=True)

	frequency = rospy.Rate(Frequency)

	rospy.Subscriber("RosAria/pose", Odometry, odom_measurement.callback_odom)
	#Subscriber of aruco publisher topic with arucos observations
	rospy.Subscriber('aruco_marker_publisher/markers', MarkerArray, marker_measurement.callback_Markers)

	while not rospy.is_shutdown():
		start_time = time.time()
		motion_model = odom_measurement.actual_movement()
		landmarkers_ids = marker_measurement.get_list_ids()
		if landmarkers_ids[0] != KEY_NUMBER and np.linalg.norm(motion_model) > minimum_move:
			total_weight = 0
			motion_model = odom_measurement.get_movement()
			for i in range(NUMBER_PARTICLES):
				expected_state = particles[i].particle_prediction(motion_model)
				for marker_id in landmarkers_ids:
					if marker_id == KEY_NUMBER:
						break
					updated_marker[marker_id] = True
					landmarker_measurement = marker_measurement.get_measerment(marker_id)
					kalman_filter = particles[i].get_kalman_filters(marker_id, landmarker_measurement)
					
					kalman_filter.Apply_EKF(expected_state, landmarker_measurement)
					validity_info = kalman_filter.measurement_validition()
					particles[i].update_weight(marker_id)
					##############################################################
					if np.linalg.norm(validity_info) < validity_threshold:
					##############################################################
						kalman_filter.Update()

				total_weight += particles[i].get_weight()

			sum_weights = 0
			for i in range(NUMBER_PARTICLES):
				particles[i].normalize_weight(total_weight)
				sum_weights += particles[i].get_weight()**2

			drawing_plot(particles)
			neff = 1.0/sum_weights
			if neff < float(NUMBER_PARTICLES)/2:
				particles = resample_particles(particles, updated_marker)
				del updated_marker
				updated_marker = [False]*NUMBER_MARKERS

		marker_measurement.reset_list_ids()
		elapsed_time = time.time() - start_time
		if elapsed_time > 9*10**-2:
			print elapsed_time
		frequency.sleep()

if __name__ == '__main__':
	FastSlam()