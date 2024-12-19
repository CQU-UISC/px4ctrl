Header header
float64[4] u
float64[3] pos
float64[3] vel
float64[3] acc
float64[3] jerk
float64 yaw
float64 yawdot
float64[4] quat

uint8 type
uint8 THRUST_BODYRATE = 0 #collective thrust and bodyrate
uint8 THRUST_TORQUE = 1 #collective thrust and torque
uint8 ROTORS_FORCE = 2 #each rotor force
uint8 DESIRED_POS = 3 #desired state