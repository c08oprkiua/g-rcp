extends RigidBody

#AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA
var TorqueScale = 0.75

#gameplay
var PhysicsLevel = 2 # Simple 0, Basic 1, Advanced 2
var UseMouseSteering = true # Use Mouse Steering
var SteeringAssistance = 0.0 # Auto Counter-Steer Rate
var SteeringAssistanceAngular = 0.0 # Auto Counter-Steer Rate (AngularVelocity-based)
var MouseSteerDeadzone = 0.0 # Mouse Steer Deadzone
var MouseSteerSensitivity = 1.0 # Mouse Steer Sensitivity
var MouseSteeringLinearity = 100.0 # Mouse Steer Linearity
var KeyboardSteerSpeed = 0.03 # Steering Rate
var KeyboardReturnSpeed = 0.05 # Steering Return Rate
var KeyboardSteerDecay = 0.025 # Steering rate decaying under car's velocity
var KeyboardReturnDecay = 0.025 # Steering return rate decaying under car's velocity
var KeyboardSteerAmountDecay = 0.075 # Understeer Prevention
var NoStall = false # Prevents engine from going under 100 RPM
var GearAssistant = [20,0,0.944087,6000,5500] # Shift delay, Level (0 - 2), Speed Influence (will be automatically set), Downshift RPM Influence,  Upshift RPM Influence

var GasSpeed = [0.25,0.25] # Release, Press
var BrakeSpeed = [0.075,0.1] # Release, Press
var HandBrakeSpeed = [0.25,0.25] # Release, Press
var ClutchSpeed = [0.25,0.25] # Release, Press

#stability
var ABS = [0.04, 0.0, 5.0, 0.3, 1] # Sensitivity, Threshold, Disable Speed, Maximum Brake, Enabled

#experimental
var ESP = [1, 0.5, 0] # Sensitivity, Threshold, Enabled
var TCS = [0.05, 0.5, 0] # Sensitivity, Threshold, Enabled

#TUNING

#transmission
var TransmissionType = 0 # 0 = Full Manual, 1 = Sequential
var ClutchGrip = 0.75 # Grip between the engine and the driveshaft.
var FinalDriveRatio = 4.25
var GearRatios = [ 3.250, 1.894, 1.259, 0.937, 0.771 ]
var ReverseRatio = 3.153
var RatioMult = 9.5 # FinalDriveRatio multiplier to find accuracy.
var ShiftClutchOff = 0.75

#power-to-wheel
var PowerToWheel = 0.9 # Amount of torque being sent to the wheels upon higher gear ratio. (Keep untouched recommended)
var DriveShaftGrip = 0.75 # Grip between the wheels and the driveshaft. (Must be matched to ClutchGrip in most cases)
var BiteStrength = 3.0 # i forgor

# Needed to keep the powertrain from spazzing. But could sometimes be pretty messy to find balance between horrible sense of acceleration and instability.
var DriveShaftStability = 12.5*DriveShaftGrip
var ClutchStability = 35.0*ClutchGrip
var StabilityThreshold = 110.0

#diff (WIP)
var DiffType = "Limited"
var DiffScale = 0.05

#engine
var EngineWeight = 1.6 # Higher weight makes it more invulnerable to resistable forces but responds slowly.
var MaxTorque = 10000.0 # Amount of torque can recieve before valve-float occurs.
var TorqueDecline = 1.0 # Probability of valve-floating.
var PotentialTorque = 0.049 # Basically a multiplier of the two above.
var StartingTorque = 40.0 # Adds up extra torque to compensate at low RPMs.
var CrossPoint = 5252.0 # Higher numbers makes the horsepower graph rise earlier. (just for measurements)
var EngineFriction = 1.0 # Decreases rev.
var EngineDrag = 5.0 # Decreases rev as it gets higher.
var ThrottleResponse = 0.5 # Fuel injection rate. (range: 0-1)
var RPMLimit = 7000.0
var IdleRPM = 800.0
var IdleThrottle = 0.1 # Throttle on idle RPM

var RevBouncing = true # Enable realistic rev limiter
var RevLimiterDelay = 5.0 # Redline bounce duration (Set to 0 to rev beyond RPM limit)

var LaunchRPM = 0.0 # Set to 0 to disable.

#turbo
var TurboInstalled = true # Enables turbo
var TurboAmount = 1 # Turbo power multiplication.
var MaxPSI = 8.0 # Maximum air generated by the turbo
var EngineCompressionRatio = 9.0 # Piston travel distance
var TurboSize = 8.0 # Higher = More turbo lag
var Compressor = 0.14 # Higher = Allows more spooling on low RPM
var SpoolThreshold = 0.1 # Range: 0 - 0.9999
var BlowoffRate = 0.14 # Range: 0 - 1
var TurboEfficiency = 0.06 # Range: 0 - 1
var TurboVacuum = 1.0 # Torque deficiency upon idling turbo

var BackfireRate = 3.0
var BackfireThreshold = 0.9

#chassis
var BrakeStrength = 50.0

#aero
var Drag = 3.0/2.0
var Downforce = 1.0
var Pitching = -0.05
var Tilting = 0.0
var Yawing = 0.0

#----

#system
var visualisation = false
var wheels = 0
var reverseassistdelay = 0.0
var dsweight = 0.0
var dsweightrun = 0.0
var clutchon = 0.0
var rpm = 0.0
var feedback = 0.0
var gas = 0.0
var brake = 0.0
var handbrake = 0.0
var gear = 0
var steer = 0.0
var steer2 = 0.0
var throttle = 0.0
var ratio = 1
var torquedrag = 1.0
var resistance = 0.0
var resistance2 = 0.0
var limitdel = 0.0
var shiftdel = 0.0
var torquereadout = 0.0
var hpreadout = 0.0
var psi = 0.0
var pastpsi = 0.0
var vacuum = 0.0
var boosting = 0.0
var speedrpm = 0.0
var wheelsonground = 0
var onground = false

var currenttorque = 0.0

var tq = 0.0
var hp = 0.0

var gpedal = false
var bpedal = false
var cpedal = false
var hb = false
var sleft = false
var sright = false
var su = false
var sd = false

#misc
var cgroundmaterial = 0.0
var wheelsforce = 0.0

var absflashed = false
var tcsflashed = false
var espflashed = false
var backfiretriggered = false

var skidding = 0.0
var skidding2 = 0.0
var wind = 0.0
var pastvelocity = 0.0
var gforce = 0.0

var blow_psi = 0.0
var blowvol = 0.0

#-------

func _ready():
	
	# individual tyre settings
	get_node("fl").set("SteerAngle_Left",32.0)
	get_node("fl").set("SteerAngle_Right",29.0)
	get_node("fr").set("SteerAngle_Left",29.0)
	get_node("fr").set("SteerAngle_Right",32.0)

#	get_node("fl").set("SteerAngle_Left",50.0)
#	get_node("fl").set("SteerAngle_Right",50.0)
#	get_node("fr").set("SteerAngle_Left",0.0)
#	get_node("fr").set("SteerAngle_Right",0.0)
#	get_node("rl").set("SteerAngle_Left",50.0)
#	get_node("rl").set("SteerAngle_Right",50.0)

#	get_node("rl").set("tyre_code","1150-195-060-14-060")
#	get_node("rr").set("tyre_code","1150-195-060-14-060")
#	get_node("rl").refreshtyres()
#	get_node("rr").refreshtyres()
	
#	RPMLimit = 1239123.0

	get_node("fl").set("Differential_Connection","fr")
	get_node("fr").set("Differential_Connection","fl")
	get_node("rl").set("Differential_Connection","rr")
	get_node("rr").set("Differential_Connection","rl")

	get_node("fl").set("Connection",1.0)
	get_node("fr").set("Connection",1.0)
	get_node("rl").set("Connection",0.0)
	get_node("rr").set("Connection",0.0)

#	get_node("fl").set("Connection",0.0)
#	get_node("fr").set("Connection",0.0)
#	get_node("rl").set("Connection",1.0)
#	get_node("rr").set("Connection",1.0)

	get_node("fl").set("BrakeInfluence",1.0)
	get_node("fr").set("BrakeInfluence",1.0)
	get_node("rl").set("BrakeInfluence",0.25)
	get_node("rr").set("BrakeInfluence",0.25)

	get_node("fl").set("Rest",0.025)
	get_node("fr").set("Rest",0.025)
	get_node("rl").set("Rest",0.025)
	get_node("rr").set("Rest",0.025)

	get_node("fl").set("StrutOffset",0.15)
	get_node("fr").set("StrutOffset",0.15)
	get_node("rl").set("StrutOffset",0.15)
	get_node("rr").set("StrutOffset",0.15)

	get_node("fl").set("stiffness",475.0)
	get_node("fr").set("stiffness",475.0)
	get_node("rl").set("stiffness",375.0)
	get_node("rr").set("stiffness",375.0)

	get_node("fl").set("elasticity",10.0)
	get_node("fr").set("elasticity",10.0)
	get_node("rl").set("elasticity",10.5)
	get_node("rr").set("elasticity",10.5)

	get_node("fl").set("HandbrakeInfluence",0.0)
	get_node("fr").set("HandbrakeInfluence",0.0)
	get_node("rl").set("HandbrakeInfluence",0.7)
	get_node("rr").set("HandbrakeInfluence",0.7)

	get_node("fl").set("SwayBar_Connection","fr")
	get_node("fr").set("SwayBar_Connection","fl")
	get_node("rl").set("SwayBar_Connection","rr")
	get_node("rr").set("SwayBar_Connection","rl")
	#----------


func _physics_process(delta):
	if PhysicsLevel<2:
		var lim = 50.0/(linear_velocity.length() +1)
		if angular_velocity.y>lim:
			angular_velocity.y = lim
		elif angular_velocity.y<-lim:
			angular_velocity.y = -lim
	
	var mouseposx = get_viewport().get_mouse_position().x/get_viewport().size.x

	var assist = -(global_transform.basis.orthonormalized().xform_inv(linear_velocity).x/(linear_velocity.length() +1))*SteeringAssistance
	assist += global_transform.basis.orthonormalized().xform_inv(angular_velocity).y*SteeringAssistanceAngular
	
	sright = Input.is_action_pressed("right")
	sleft = Input.is_action_pressed("left")
	
	if Input.is_action_just_pressed("toggle_visualisation"):
		if not visualisation:
			visualisation = true
		else:
			visualisation = false
	
	var siding = abs(global_transform.basis.orthonormalized().xform_inv(linear_velocity).x)
	var going = global_transform.basis.orthonormalized().xform_inv(linear_velocity).z/(siding +1)
	if going<0:
		going = 0
	if sleft:
		if steer2>0:
			steer2 += KeyboardSteerSpeed/(going*KeyboardSteerDecay +1)
		else:
			steer2 += KeyboardReturnSpeed/(going*KeyboardReturnDecay +1)
	elif sright:
		if steer2<0:
			steer2 -= KeyboardSteerSpeed/(going*KeyboardSteerDecay +1)
		else:
			steer2 -= KeyboardReturnSpeed/(going*KeyboardReturnDecay +1)
	else:
		if steer2>KeyboardReturnSpeed/(going*KeyboardReturnDecay +1):
			steer2 -= KeyboardReturnSpeed/(going*KeyboardReturnDecay +1)
		elif steer2<-KeyboardReturnSpeed/(going*KeyboardReturnDecay +1):
			steer2 += KeyboardReturnSpeed/(going*KeyboardReturnDecay +1)
		else:
			steer2 = 0
		
	var maxsteer = 1/(going*KeyboardSteerAmountDecay +1)
		
	if steer2>maxsteer:
		steer2 = maxsteer
	elif steer2<-maxsteer:
		steer2 = -maxsteer
		
	steer = steer2 -assist
		
	if UseMouseSteering:            
		var stinfluence = (mouseposx-0.5)*(MouseSteerSensitivity*2)
		if stinfluence<0:
			stinfluence = -stinfluence
			
		stinfluence = stinfluence*MouseSteeringLinearity
			
		if stinfluence>1:
			stinfluence = 1
			   
		steer = -((mouseposx-0.5)*stinfluence)*(MouseSteerSensitivity*2)  -assist
		

		
	if steer>1.0:
		steer = 1.0
	elif steer<-1.0:
		steer = -1.0
		
	var ptorque = PotentialTorque
	var storque = StartingTorque

	#aerodynamics
	wind = global_transform.basis.orthonormalized().xform_inv(get_linear_velocity())/60.0
	
	add_central_force(Vector3(global_transform.basis.orthonormalized().xform(Vector3(-(wind.x*Drag)*60.0,-wind.z*Downforce -wind.y*Drag,-(wind.z*Drag)*60.0))))
	
	currenttorque = (rpm - (rpm*(rpm*TorqueDecline))/MaxTorque)*ptorque +storque
	if currenttorque<0.0:
		currenttorque = 0.0
	
	if TurboInstalled:
		var yes = (pastpsi - psi)*BackfireRate
		pastpsi = psi
		if yes>1.0:
			yes = 1.0
		elif yes<0.0:
			yes = 0.0
		var radomed = rand_range(0.0,yes)
		if radomed>BackfireThreshold:
			backfiretriggered = true
			
		var thr = (throttle-SpoolThreshold)/(1.0-SpoolThreshold)
		
		if boosting>thr:
			boosting = thr
		else:
			boosting -= (boosting - thr)*TurboEfficiency
			
		psi += (boosting*rpm)/((TurboSize/Compressor)*60.9)
		psi -= psi*BlowoffRate
		if psi>MaxPSI:
			psi = MaxPSI
		elif psi<-TurboVacuum:
			psi = -TurboVacuum
			
		currenttorque += ( (psi*TurboAmount) * (EngineCompressionRatio*0.609))
		
	tq = (currenttorque -rpm*(EngineDrag/1000.0) -EngineFriction)*TorqueScale
	hp = (rpm/CrossPoint)*tq
	
	if gear>0:
		ratio = (GearRatios[gear-1]*FinalDriveRatio)*RatioMult
	else:
		ratio = -(ReverseRatio*FinalDriveRatio)*RatioMult
		
	var ra = abs(ratio)
	
	var assistshiftspeed = (GearAssistant[3]/ra)*GearAssistant[2]
	var assistdownshiftspeed = (GearAssistant[4]/abs((GearRatios[gear-2]*FinalDriveRatio)*RatioMult))*GearAssistant[2]
	torquedrag = (PowerToWheel/10.0)*ra
	
	gpedal = false
	bpedal = false
	cpedal = false
	hb = false
	
	if UseMouseSteering:
		if GearAssistant[1]<2:
			su = Input.is_action_just_pressed("shiftup_mouse")
			sd = Input.is_action_just_pressed("shiftdown_mouse")
		else:
			su = false
			sd = false
		
		gpedal = Input.is_action_pressed("gas_mouse")
		bpedal = Input.is_action_pressed("brake_mouse")
		cpedal = Input.is_action_pressed("clutch_mouse")
		hb = Input.is_action_pressed("handbrake_mouse")
	else:
		if GearAssistant[1]<2:
			su = Input.is_action_just_pressed("shiftup")
			sd = Input.is_action_just_pressed("shiftdown")
		else:
			su = false
			sd = false
		
		gpedal = Input.is_action_pressed("gas")
		bpedal = Input.is_action_pressed("brake")
		cpedal = Input.is_action_pressed("clutch")
		hb = Input.is_action_pressed("handbrake")
		
	if GearAssistant[1] == 2:
		if gpedal and gear == 0:
			gear = 1
		elif bpedal and gear == 0:
			gear = -1
		if linear_velocity.length()<1:
			if gear == -1 and gpedal or gear>0 and bpedal or not bpedal and not gpedal:
				gear = 0
		if gear>0:
			if linear_velocity.length()>assistshiftspeed/2.0:
				su = true
			elif linear_velocity.length()<assistdownshiftspeed/2.0 and gear>1:
				sd = true
			
		if gear == -1:
			var interferance = gpedal
			gpedal = bpedal
			bpedal = interferance
			
			
	if gear<len(GearRatios):
		if su and clutchon<ShiftClutchOff or su and GearAssistant[1]>0:
			if GearAssistant[1]>0 and not gear<1:
				shiftdel = GearAssistant[0]
			gear += 1
	if gear>-1:
		if sd and clutchon<ShiftClutchOff or sd and GearAssistant[1]>0:
			gear -= 1
		
		
	if gpedal and shiftdel<0:
		gas += GasSpeed[1]
	else:
		gas -= GasSpeed[0]
		
	if bpedal:
		brake += BrakeSpeed[1]
	else:
		brake -= BrakeSpeed[0]
		
	if hb:
		handbrake += HandBrakeSpeed[1]
	else:
		handbrake -= HandBrakeSpeed[0]
		
	shiftdel -= 1
		
	if cpedal or shiftdel>0 and GearAssistant[1]>0 or GearAssistant[1]>0 and rpm<2000:
		clutchon -= ClutchSpeed[1]
	else:
		clutchon += ClutchSpeed[0]
		
	if gas<0.0:
		gas = 0.0
	elif gas>1.0:
		gas = 1.0
	if brake<0.0:
		brake = 0.0
	elif brake>1.0:
		brake = 1.0
	if handbrake<0.0:
		handbrake = 0.0
	elif handbrake>1.0:
		handbrake = 1.0
	if clutchon<0.0:
		clutchon = 0.0
	elif clutchon>1.0:
		clutchon = 1.0
		
	if rpm>RPMLimit and RevBouncing or rpm>LaunchRPM and not LaunchRPM == 0.0 and gear == 0:
		limitdel = RevLimiterDelay
		
	limitdel -= 1
	
	if limitdel<0:
		if throttle<gas:
			throttle -= (throttle - gas)*ThrottleResponse
		else:
			throttle = gas
		
		if rpm<IdleRPM:
			if throttle<IdleThrottle:
				throttle = IdleThrottle
	else:
		throttle = 0.0
		
	rpm += resistance/(EngineWeight/1.6)

	if rpm>EngineFriction/(EngineWeight/1.6):
		rpm -= EngineFriction/(EngineWeight/1.6)
	elif rpm<-EngineFriction/(EngineWeight/1.6):
		rpm += EngineFriction/(EngineWeight/1.6)
	else:
		rpm = 0.0

	speedrpm += resistance2
	resistance = 0.0
	resistance2 = 0.0

	rpm += (((currenttorque*2.0)*throttle)/(EngineWeight/1.6))
		
	rpm = rpm/((EngineDrag/(EngineWeight/1.6))/1000.0 +1)

	if NoStall and rpm<100:
		rpm = 100
	if rpm>RPMLimit and not RevBouncing:
		rpm = RPMLimit
		
	dsweightrun = dsweight
		
#	print(dsweightrun)


	dsweight = 0.0
			
	#misc
	var soundvolume = 1.0
	
	get_node("engine").pitch_scale = abs(rpm)/5430.0
	get_node("engine").unit_db = linear2db((throttle*0.5 +0.5)*soundvolume)
	get_node("engine").max_db = get_node("engine").unit_db
	
	var skikd = skidding2/25.0 -0.5
	if skikd<0.0:
		skikd = 0.0
	elif skikd>1.0:
		skikd = 1.0
			
	if wheelsonground>0:
		cgroundmaterial /= float(wheelsonground)
		wheelsforce /= float(wheelsonground)

		if wheelsforce>100.0:
			wheelsforce = 100.0
		get_node("skid").pitch_scale = 0.7 +skikd*0.25
		get_node("skid").unit_db = linear2db(((skikd/1.5)*(1.0-cgroundmaterial))*soundvolume)
		get_node("dirt").unit_db = linear2db(((wheelsforce/100.0)*cgroundmaterial)*soundvolume)
		get_node("dirt").pitch_scale = 1.0 +wheelsforce/100.0
	else:
		get_node("skid").unit_db = linear2db(0)
		get_node("dirt").unit_db = linear2db(0)

	get_node("skid").max_db = get_node("skid").unit_db
	get_node("dirt").max_db = get_node("dirt").unit_db
	
	skidding = 0.0
	skidding2 = 0.0	
	
	cgroundmaterial = 0.0
	wheels = 0
	wheelsonground = 0
	wheelsforce = 0.0
	
	gforce = (linear_velocity.length() - pastvelocity)*((0.30592/9.8)*120.0)
	pastvelocity = linear_velocity.length()
		
	var turbovolume = 0.2

	var dist = blow_psi - psi
	blow_psi = psi
			
	blowvol = dist
	if blowvol<0:
		blowvol = 0
	elif blowvol>1:
		blowvol = 1
		

	var spoolvol = psi/10.0
	if spoolvol<0:
		spoolvol = 0
	elif spoolvol>1:
		spoolvol = 1
	
		
	get_node("blow").unit_db = linear2db((blowvol*turbovolume)*soundvolume)
	get_node("spool").unit_db = linear2db(((spoolvol*1.5)*turbovolume)*soundvolume)
	get_node("spool").pitch_scale = 0.8 +spoolvol*0.5
		
	get_node("blow").max_db = get_node("blow").unit_db
	get_node("spool").max_db = get_node("spool").unit_db
		
