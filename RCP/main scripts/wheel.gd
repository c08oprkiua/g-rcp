extends RayCast

class_name SeVeWheel

export var benchmarked:bool = false

# external (could be adjusted for individual wheels by another script)
export var tyre_code:String = "0250-185-060-14" # the first 4-row is the grip amount. set to "0250" by default
export var roughness:float = 0.25
export var forward_grip_deficiency:float = 1.0
export var backward_grip_deficiency:float = 4.0

export var WheelScale:float = 0.5
export var TyreGrip:float = 1.0
export var RealisticTyres:bool = false # uses tyre_code to determine grip and wheel scale

export var stiffness:float = 475.0 # Spring stiffness
export var elasticity:float = 10.0 # Spring rebound rate
export var damp:float = 1.0 # Rebound Dampening

# placeholder
export var stiffness_struf:float = 2.5
export var elasticity_struf:float = 75.0
export var StrutDistance:float = 1.0
#---

export var stiffness_swaybar:float = 0.0 # Swaybar Stiffness


export var Connection:float = 0.0 # Connection between the driveshaft and the wheel itself.
export var BrakeInfluence:float = 0.0 # The amount of stopping torque taken from the "BrakeStrength" property when pressing the brake.
export var HandbrakeInfluence:float = 0.0 # The amount of stopping torque taken from the "BrakeStrength" property when pulling the handbrake.
export var Camber:float = 0.0 # The slant angle of the wheel.
export var Caster:float = 0.0 # Steering caster angle.
export var Toe:float = 0.0 # Toe-In Angle
export var Rest:float = 0.5 # Suspension Rest Distance
export var Offset:float = 0.0 # Hub Offset
export var StrutOffset:float = 0.0 # (WIP)
export var SteerAngle_Left:float = 0.0 # Left steering angle
export var SteerAngle_Right:float = 0.0 # Right steering angle
export var Suspension_Geometry:float = 25.0 # Higher numbers causes more negative camber upon compression.
export var Differential_Connection = "" # (WIP) Connects the differential to another wheel.
export var SwayBar_Connection = "" # Connects the sway bar to another wheel's axle.

export var abs_strength:float = 1.0 # TCS Sensitivity
export var tcs_strength:float = 1.0 # ABS Sensitivity
export var esp_strength:float = 1.0 # ESP Sensitivity

# internal
var grip = int(tyre_code.substr(0,4))
var tyrewidth = int(tyre_code.substr(5,3))
var tyrear = int(tyre_code.substr(9,4))
var rimsize = int(tyre_code.substr(13,2))
var tread = tyrear


var wheelsize =  (( float(tyrewidth)*((float(tyrear)*2.0)/100.0) + float(rimsize)*25.4 )*0.003269)/4.0
#var wheelsize:float =  0.2

var q:float = ((wheelsize/((1.0*0.003269)/2.0))*0.003269)/2.0
var tyrelc:float = ((wheelsize/((1.0*0.003269)/2.0))*q)/125.0

var lateraldamp:float = float(tread)*0.0085
var wheelweight:float = wheelsize*2.0
var tyreprofile:float = float(tread)/10.0
var coefficiency:float = float(tyrewidth)*0.1
var contact:float = 8.0
var tyrecompressrate:float = 0.9
var tyrecompressiongripmultiply:float = float(tyrewidth)/500.0
var thread:float = 1.0
var compress:float = 0.0
var compress2:float = 0.0


var rigidity:float = 60.0
var tyrerigidity:float = 60.0

# system
var forcedata = Vector2(0,0)
var wv:float = 0.0
var tyrecompressed:float = 0.0
var tyrecompressedgrip:float = 0.0
var tyrecompressedscrub:float = 0.0
var currentgrip:float = 0.0
var gripscrub:float = 0.0
var wheelangle:float = 0.0
var currentcamber:float = 0.0
var wheelcompression:float = 0.0
var patch = Vector2(0,0)
var scrub = 0.0
var dist:float = 0.0
var contactforce:float = 0.0
var brakeforce:float = 0.0
var currentconnection:float = 0.0
var slip:float = 0.0
var slipz:float = 0.0
var brokencontact:float = 0.0
var brokencontactspin:float = 0.0
var skidspin:float = 0.0
var skid:float = 0.0
var skid2:float = 0.0

var currentstif:float = 0.0
var currentelast:float = 0.0

var wsing:float = 0.0


# effects
var cgroundmaterial:float = 0.0
var bumpy:float = 0.0
var bumpycurrent:float = 0.0
var bumpfrequency:float = 0.0
var bumpfrequencyrandomize:float = 0.0
var bumpinverted:bool = false
var griploss:float = 0.0
var currentelasticity:float = 0.0
var currentstiffness:float = 0.0

var cast_current:float = 0.0

onready var n_velocity:RigidBody = $velocity
onready var n_velocity2:RigidBody = $velocity2
onready var n_geometry:MeshInstance = $geometry
onready var n_geometry_lateral:MeshInstance = $geometry/lateral
onready var n_geometry_compress:MeshInstance = $geometry/compress
onready var n_geometry_longi:MeshInstance = $geometry/longi
onready var n_contact_axis:Position3D = $contact_axis
onready var n_contact_axis_force:Position3D = $contact_axis/force
onready var n_axis:Position3D = $axis
onready var n_animation:Position3D = $animation
onready var n_animation_spinning:Position3D = $animation/spinning

onready var n_car:SeVeCar = n_car
onready var n_diff_connection:SeVeWheel
onready var n_swaybar_connection:SeVeWheel


func alignAxisToVector(xform, norm):
	xform.basis.y = norm
	xform.basis.x = -xform.basis.z.cross(norm)
	xform.basis = xform.basis.orthonormalized()
	return xform

func refreshtyres():
	grip = int(tyre_code.substr(0,4))
	tyrewidth = int(tyre_code.substr(5,3))
	tyrear = int(tyre_code.substr(9,4))
	rimsize = int(tyre_code.substr(13,2))
	tread = tyrear
	
	wheelsize =  (( float(tyrewidth)*((float(tyrear)*2.0)/100.0) + float(rimsize)*25.4 )*0.003269)/4.0
	
	q = ((wheelsize/((1.0*0.003269)/2.0))*0.003269)/2.0
	tyrelc = ((wheelsize/((1.0*0.003269)/2.0))*q)/125.0
	
	lateraldamp = float(tread)*0.0085
	wheelweight = wheelsize*2.0
	tyreprofile = float(tread)/10.0
	coefficiency = float(tyrewidth)*0.1
	contact = 8.0
	tyrecompressrate = 0.9
	tyrecompressiongripmultiply = float(tyrewidth)/500.0
	thread = 1.0
	
	if Connection>0:
		n_car.GearAssistant[2] = wheelsize

func _ready():
	add_exception(n_car)
	add_exception(n_velocity)
	add_exception(n_velocity2)
	n_geometry.translation = cast_to
	n_velocity.global_transform.origin = global_transform.origin
	n_velocity2.global_transform.origin = n_geometry.global_transform.origin
	
	if WheelScale:
		wheelsize = WheelScale
		grip = TyreGrip * 250

func _physics_process(delta):
	if not is_instance_valid(n_car):
		if get_parent().get_class() == "RigidBody":
			n_car = get_parent()
		return
	
#	print(n_car.get_node("test").linear_velocity)
#	print(n_velocity.linear_velocity)
	n_geometry.visible = n_car.visualisation
	n_car.wheels += 1
	n_velocity2.global_transform.basis = n_axis.global_transform.basis
	n_velocity.global_transform.basis = n_axis.global_transform.basis
	var rayvelocity = n_velocity.global_transform.basis.orthonormalized().xform_inv(n_velocity.get_linear_velocity())
	var rayvelocity2 = n_velocity2.global_transform.basis.orthonormalized().xform_inv(n_velocity2.get_linear_velocity())
	var rayvelocity2velocity = Vector2(rayvelocity2.x,rayvelocity2.z).length()
	
	n_velocity2.global_transform.basis = n_contact_axis.global_transform.basis
	n_velocity.global_transform.basis = n_contact_axis.global_transform.basis
	
	#var c_rayvelocity = n_velocity.global_transform.basis.orthonormalized().xform_inv(n_velocity.get_linear_velocity())
	var c_rayvelocity2 = n_velocity2.global_transform.basis.orthonormalized().xform_inv(n_velocity2.get_linear_velocity())
	
	n_geometry.global_transform.basis = n_contact_axis.global_transform.basis
	n_axis.rotation = Vector3(0,0,0)
	n_contact_axis.rotation_degrees.x = 0
	n_contact_axis.rotation_degrees.z = 0
	n_velocity.linear_velocity = -(n_velocity.global_transform.origin -  global_transform.origin)*rigidity
	n_velocity2.linear_velocity = -(n_velocity2.global_transform.origin -  n_geometry.global_transform.origin)*tyrerigidity
	n_contact_axis_force.translation = Vector3(0,0,0)
	
	var rotation_za = n_axis.rotation_degrees.z
	var w = -((dist-StrutOffset)*Suspension_Geometry)/(translation.x/deg2rad(90.0))
	
	var mcpherson = w/(abs(w)/90.0 +1)
	
	if translation.x>0:
		currentcamber = -mcpherson - Camber + Caster*n_car.steer
	else:
		currentcamber = -mcpherson + Camber + Caster*n_car.steer

	n_animation.rotation_degrees.z = currentcamber
	
	var wheelinclinement = clamp(rotation_za/90.0 +currentcamber/90.0, -0.25, 0.25)
	
	wheelangle = abs((wheelinclinement -scrub/90.0)/(tyreprofile/10.0))
	
	if wheelangle>1.0:
		wheelangle = 1.0
	
	compress = 0.0
	
	var st = 0.0
	
	if n_car.steer<0:
		st = n_car.steer*SteerAngle_Left
	else:
		st = n_car.steer*SteerAngle_Right
	
	var toe = Toe
	if translation.x<0:
		toe = -toe
	
	rotation_degrees.y = st -toe
	
	differentials()
	
#	print(currentconnection)
	
	var time_results:PoolRealArray = [0.0, 0.0, 0.0, 0.0]
	
	# ----
	if is_colliding():
		
		time_results[0] = Time.get_ticks_usec()
		#suspension
		n_geometry.global_transform.origin = get_collision_point()
		n_axis.global_transform = alignAxisToVector(n_axis.global_transform, get_collision_normal())
		n_contact_axis.global_transform = alignAxisToVector(n_contact_axis.global_transform, get_collision_normal())
		
		bumpfrequency = 0.0
		bumpy = 0.0
		bumpfrequencyrandomize = 0.0
		griploss = 0.0
		cgroundmaterial = 0.0
		n_car.wheelsonground += 1
		
		var collider:Object = get_collider()
		
		if is_instance_valid(collider) and not collider.get("groundmaterial") == null:
			bumpfrequency = collider.get("bumpfrequency")
			bumpy = collider.get("bumpy")
			bumpfrequencyrandomize = collider.get("bumpfrequencyrandomize")
			griploss = collider.get("griploss")
			cgroundmaterial = collider.get("groundmaterial")
		
		if bumpinverted:
			bumpycurrent -= (rayvelocity2velocity*(bumpfrequency*rand_range(1.0-bumpfrequencyrandomize,1.0+bumpfrequencyrandomize)))/100.0
		else:
			bumpycurrent += (rayvelocity2velocity*(bumpfrequency*rand_range(1.0-bumpfrequencyrandomize,1.0+bumpfrequencyrandomize)))/100.0
		if bumpycurrent<0:
			bumpinverted = false
			bumpycurrent = 0
		elif bumpycurrent>bumpy:
			bumpinverted = true
			bumpycurrent = bumpy
		
		n_car.cgroundmaterial += cgroundmaterial
		
		cast_current = cast_to.y +bumpycurrent
		
		var scvelo = rayvelocity.y
		if scvelo > 0.0:
			scvelo *= damp
		
		compress2 = (n_geometry.translation.y-cast_current -Rest)*currentelast
		if compress2<0:
			compress2 = 0
		
		if (n_geometry.translation.y-cast_current) > Rest:
			compress = (scvelo - compress2)*currentstif
		if compress>0:
			compress = 0
		
		wheelcompression = -compress
		n_contact_axis_force.translation.y = -compress
		
		var compressed_wheel:float = (wheelcompression / 1000.0) * tyrecompressrate
		
		tyrecompressed = compressed_wheel * (-(wheelangle) + 1)
		tyrecompressed *= 1.0 -wheelangle
		tyrecompressedscrub = max(compressed_wheel * 2.0, 0.0)

		if tyrecompressed<0.0:
			tyrecompressed = 0.0
#		elif tyrecompressed>tyrelc:
#			tyrecompressed = tyrelc
		
		#var decline = (tyrecompressed*tyrecompressed)*(0.8/tyrelc) - (0.43/tyrelc)
		
		tyrecompressedgrip = tyrecompressed*2.5
		
		var sliped = 1.0
		
		var predistz = c_rayvelocity2.z - (wv*wheelsize)
		
		if tyrecompressed>0.0:
			sliped = max(forcedata[0],forcedata[1])/(tyrecompressed*10.0)
		if predistz>0.0:
			if sliped>backward_grip_deficiency:
				sliped = backward_grip_deficiency
			else:
				sliped *= sliped/backward_grip_deficiency
		else:
			if sliped>forward_grip_deficiency:
				sliped = forward_grip_deficiency
			else:
				sliped *= sliped
		
#		sliped = 0.0
		
#		print(sliped)
		
#		currentgrip = (((( (float(grip)*((float(tyrewidth)/(float(tyrewidth)/2))/1.5))/(n_car.mass/150) )/(sliped*((roughness*(-(cgroundmaterial)+1))) +1))/(griploss+1))/1.1)*(tyrecompressedgrip*tyrecompressiongripmultiply) *0.825
		#var griplimit = ((tyrecompressedgrip * float(grip))*2.5)/(sliped*((roughness*(-(cgroundmaterial)+1))) +1)
		var griplimit = ((tyrecompressedgrip * grip) * 2.5) / (sliped * ((roughness*(-(cgroundmaterial)+1))) +1)
		
		griplimit /= griploss +1.0
		
		if RealisticTyres:
			var deficiency = float((n_car.mass*2.0)-float(tyrewidth))
			deficiency = clamp(deficiency / 500.0, 0.0 ,0.9999)
			
			currentgrip = griplimit*(1.0 - deficiency)
		else:
			currentgrip = griplimit
		
		gripscrub =  griplimit
#		gripscrub *= 0.0
		
		time_results[0] -= Time.get_ticks_usec()
		
		#-----
		
		# idfk
		time_results[1] = Time.get_ticks_usec()
		var limt = coefficiency*(currentgrip/1000.0)
		contactforce = clamp(contactforce, -limt, limt)
		
		if contactforce>0.0:
			contactforce += brakeforce
			if contactforce<0.0:
				contactforce = 0.0
		else:
			contactforce += brakeforce
			if contactforce>0.0:
				contactforce = 0.0
		
		var thectf = contactforce
		
		patch.x += c_rayvelocity2.x/5.0
		var maxtravel = currentgrip/1000.0
		patch.x = clamp(patch.x, -maxtravel, maxtravel)
		
		patch.y += (c_rayvelocity2.z - (wv*wheelsize))/5.0
		patch.y = clamp(patch.y, -maxtravel, maxtravel)
		
		patch /= abs(wv)/10.0 +1.0
		
		var br = (brakeforce/100.0)
		
		if br>0.0:
			var infl = max(1.0 / br, 1.0)
			
			patch.y /= infl
		else:
			patch.y *= 0.95
		
		var distz:float = c_rayvelocity2.z - ((wv+(thectf))*wheelsize) + patch.y
		var distzw:float = c_rayvelocity2.z - ((wv+(thectf))*wheelsize)
		var distx:float = c_rayvelocity2.x + patch.x
		var distz3:float = c_rayvelocity2.z - (wv*wheelsize)
		var distz4:float = c_rayvelocity2.z - ((wv+(thectf*-0.5))*wheelsize) + patch.y
		var slip:float = abs(distz) + abs(distx)
#		slipz = max(abs(distx),abs(distz))
		slipz = abs(distx*2.5) + abs(distz)
		var longimode:float = clamp(rayvelocity2velocity - abs(wv)*wheelsize -currentgrip/1000.0, 0.0, 1.0)
		
		var rolldirt:float = min(abs(wv), 10.0)
		
		var the_yes:float = min((abs(distz) + abs(distx))*2.5 + rolldirt, 100.0)
		
		n_car.wheelsforce += the_yes
		
		time_results[1] -= Time.get_ticks_usec()
		
		#----------
		
		#forces
		time_results[2] = Time.get_ticks_usec()
		var amountz:float = 0.0
		var amountzw:float = 0.0
		var amountx:float = 0.0
		skid = 0.0
		skid2 = 0.0
		var offsettedx:float = 1.0
		var offsettedz:float = 1.0
		var offsettedzw:float = 1.0
		
		if tyrecompressed>0.0:
			var slip2:float = max((abs(distz3) +abs(distx))*(0.1/tyrecompressed), 0.0)
			
			var thevelo:float = (abs(wv)/1.5)/(slip2 +1.0)
#			thevelo /= thevelo*0.001 +1.0
			var mass:float = n_car.mass/100
			var unita:float = 75.0*mass
			unita /= unita*0.0107 +1.0
			var unitb:float = float(tyrewidth)/8.0
			var unitc:float = 1.0
			var unitd:float = 0.0015/(mass/currentgrip)
			offsettedzw = (currentgrip*wheelweight)/unita
			
#			if contactforce == 0.0:
#				longimode = 1.0
			
			offsettedz = ((abs(wv)*(lateraldamp/unitb))/(1.0/4.0))*longimode + offsettedzw*(-(longimode)+1.0)
			offsettedx = ((thevelo*(lateraldamp/unitb))/(1.0/4.0))
			
			offsettedz *= abs(wv)/(abs(wv) +1.0)
			
			if offsettedz<unitc/wheelweight:
				offsettedz = unitc/wheelweight
			if offsettedx<unitd/wheelweight:
				offsettedx = unitd/wheelweight
			
			w = (tyrecompressed/contact)*2.0
			var ok:float = min(((tyrecompressed*w)/contact)*2.0, 1.0)
			
			brokencontactspin = abs(distz3)*(thread/(tyrecompressed/contact +1.0)) -(limt*0.1)
			brokencontact = Vector2(abs(distz3),abs(distx)).length()*(thread/(tyrecompressed/contact +1.0)) -(limt*0.1)
			skidspin = brokencontactspin
			skid = brokencontact
			skid2 = Vector2(abs(distz3),abs(distx)).length()
			
			brokencontact = clamp(brokencontact, 0.0, 1.0)
			brokencontactspin = clamp(brokencontactspin, 0.0, 1.0)
			
			var patchdistancefromcenter:float = (n_geometry.global_transform.origin - n_car.global_transform.origin).length()
			
			var farx:float = distx*patchdistancefromcenter
			var farz:float = distz*patchdistancefromcenter
			farx /= tyrecompressed+1.0
			farz /= tyrecompressed+1.0
			if translation.x>0:
				farx *= -1.0
			if translation.z>0:
				farz *= -1.0
			
			wsing = clamp(farz/2.5 -abs(farx)*0.1, 0.0, 0.75/(cgroundmaterial +1.0))
			
#			wsing = 0.0
				
#			print(wsing)
			var going:float = rayvelocity2velocity
			
			going /= wheelsize*100.0
			
			going -= 0.25
			#var going2:float = clamp(going*2.0, 0.0, 1.0)
			
			if going<1.0-(wsing/0.75):
				 going = 1.0-(wsing/0.75)
			going = clamp(going, 0.0, 1.0-wsing)
			
#			going = 1.0
			
#			wsing = 1.0

			var siding:float = abs(c_rayvelocity2.x/(rayvelocity2velocity +1.0))
			siding = min(siding * 5.0, 1.0)
			
			var declinet:float = min(siding + wsing, 1.0)
			var declinet2:float = min(siding + going, 1.0)
			var declinet3:float = max(going - siding, 0.0)
			
#			var AAAA:float = distz3*(1.0-wsing) + distz4*wsing
			
			var dapedz:float = abs(distz4*(1.0-declinet2) +distz3*declinet2) -(offsettedz*declinet3)*(1.0-declinet +wsing)
#			var dapedz = abs(distz3) -offsettedz*2.0
			if dapedz<0.0:
				 dapedz = 0.0
			
			var dapedx:float = max(abs(distx) -(offsettedx*going), 0.0)
#			var dapedx = abs(distx)
			
			var method1 = Vector2(Vector2(abs(distz),dapedx).length() -offsettedz,Vector2(dapedz,abs(distx)).length())
			var method2 = Vector2(abs(distz) + dapedx - offsettedz,dapedz + abs(distx))
			var methodw = Vector2(abs(distz),dapedx*2.0).length() -offsettedz
			
#			var methodtest = Vector2(abs(distz),abs(distx)).length()
			
#			wsing = 1.0
			
			var dampz:float = max(method1.x*(-(wsing)+1.0) + method2.x*wsing, 0.0)
			var dampw:float = max(methodw, 0.0)
			var dampx:float = max(method1.y*(-(wsing)+1.0) + method2.y*wsing, 0.0)
			
#			dampz = methodtest
#			dampx = methodtest
			
#			var dampw = (abs(distz) + abs(distx))*2.0 -offsettedz
#			var dampw = Vector2(abs(dampz),abs(distx)*2.0).length()*2.0 -offsettedz
			
			amountz = distz/(dampz/offsettedz +1.0) /offsettedz
			amountzw = distzw/(dampw/offsettedzw +1.0) /offsettedzw
			amountx = distx/(dampx/offsettedx +1.0) /offsettedx
			forcedata = [abs(distx),abs(distz)]
			n_contact_axis.rotation_degrees.y = 0
		
		var longitudinal:float = amountz*(currentgrip*2.0)
		var longitudinalw:float = amountzw*(currentgrip*2.0)
		var lateral:float = amountx*(currentgrip*2.0)
		var lateralscrub:float = rayvelocity2.x*(gripscrub*2.0)
		
		n_contact_axis_force.translation.x = -lateral/2.0
		n_contact_axis_force.translation.z = -longitudinal/2.0
		
		scrub = clamp(lateralscrub/(float(tyrewidth)*180.0), -90.0, 90.0)
		
		scrub /= (abs(scrub)*deg2rad(1.0) ) +1
		
		var wvok = min(abs(wv)/10.0, 1.0)
		
		scrub *= wvok
		
		wv += (longitudinalw/wheelweight)/50.0
		
		time_results[2] -= Time.get_ticks_usec()
		
		#------
		
		var h:float = clamp(skid*0.5 -abs(wv)*(lateraldamp/10.0) - 0.75, 0.0, 1.0)
		
		n_car.skidding = n_car.skidding+h
		
		var h2:float = max(skid2, 0.0)
		
		n_car.skidding2 = n_car.skidding2 + h2
		
		#debug
		n_geometry_compress.scale = Vector3(0.01,tyrecompressed/0.25,0.01)
		n_geometry_compress.translation.y = n_geometry_compress.scale.y/2
		n_geometry_longi.scale = Vector3(0.01,0.01,n_contact_axis_force.translation.z/500.0)
		n_geometry_longi.translation.z = n_geometry_longi.scale.z/2.0
		n_geometry_lateral.scale = Vector3(n_contact_axis_force.translation.x/500.0,0.01,0.01)
		n_geometry_lateral.translation.x = n_geometry_lateral.scale.x/2.0
		
		var half_wheel_size:float = wheelsize / 2.0
		
		n_geometry_compress.translation.y -= half_wheel_size
		n_geometry_lateral.translation.y = -half_wheel_size
		n_geometry_longi.translation.y = -half_wheel_size
	
	else:
		cast_current = cast_to.y
		wheelcompression = 0.0
		tyrecompressed = 0.0
		n_geometry.translation = cast_to
		n_contact_axis.rotation_degrees.y = 0
	
	dist = abs(cast_current-n_geometry.translation.y)
	
	n_geometry_compress.visible = is_colliding()
	n_geometry_lateral.visible = is_colliding()
	n_geometry_longi.visible = is_colliding()
	
	if is_instance_valid(n_swaybar_connection):
		var rolldist = max(dist - n_swaybar_connection.dist, 0.0)
		
		currentelast = elasticity*(rolldist*stiffness_swaybar +1)
		currentstif = stiffness*(rolldist*stiffness_swaybar +1)
	else:
		n_swaybar_connection = n_car.get_node(SwayBar_Connection)
		currentelast = elasticity
		currentstif = stiffness

	n_geometry.translation.y += wheelsize -(tyrecompressed*0.0025)
	
	if translation.x>0:
		n_geometry.translation.x += Offset
	else:
		n_geometry.translation.x -= Offset
	
	#wv manipulation
	contactforce = 0.0
	#brake
	
	time_results[3] = Time.get_ticks_usec()
	
	var brslip = slipz
	brslip -= n_car.ABS[1]
	if brslip<0:
		brslip = 0
	elif n_car.brake>0.5 and n_car.ABS[4] and rayvelocity2velocity>n_car.ABS[2]:
		n_car.set("absflashed", true)
	
	var absenabled = false
	
	if rayvelocity2velocity>n_car.ABS[2]:
		absenabled = n_car.ABS[4]
	
	var br:float = 0.0
	
	if absenabled:
		if n_car.ABS[5]:
			var brakae:float = n_car.brake
			if brakae>n_car.ABS[3]:
				 brakae = n_car.ABS[3]
			br = brakae*(-(brslip*(n_car.ABS[0]))+1)
		else:
			n_car.brakethreshold += brslip/50.0
			br = (n_car.brake*n_car.brake) - n_car.brakethresholdrun
			br *= n_car.ABS[3]
	else:
		br = n_car.brake*n_car.brake

	var brake:float = (br*BrakeInfluence)+((n_car.handbrake)*HandbrakeInfluence)
	#var brake2:float = brake
	
#	brake *= brake
	
	var espb:float = 0.0
	
	if translation.x>0.0:
		espb = (n_car.angular_velocity.y)*esp_strength -n_car.ESP[1]
	else:
		espb = (-n_car.angular_velocity.y)*esp_strength -n_car.ESP[1]

	if espb<0:
		espb = 0
	elif espb>(n_car.ESP[0]*n_car.ESP[2]):
		espb = (n_car.ESP[0]*n_car.ESP[2])
	elif espb>0 and n_car.ESP[2]:
		n_car.set("espflashed", true)

	var tcs:float = slipz*((n_car.TCS[0]*tcs_strength)*n_car.TCS[2]) -n_car.TCS[1]
	if tcs<0:
		tcs = 0
	elif tcs>0 and n_car.TCS[2]:
		n_car.set("tcsflashed", true)

	brake = clamp(brake + espb + tcs, 0.0, 1.0)
	
	
	time_results[3] -= Time.get_ticks_usec()
	
	if benchmarked:
		var sorted:PoolRealArray = time_results
		sorted.sort()
		print(time_results.find(sorted[0]))
		
	
	#-----
	
	driveshaft()
	#---------------

	# brake repositioned
	brakeforce = 0.0
	var brakeforcec:float = n_car.BrakeStrength*(brake*BrakeInfluence)
	if wv>brakeforcec:
		wv -= brakeforcec
		brakeforce -= brakeforcec
	elif wv<-brakeforcec:
		wv += brakeforcec
		brakeforce += brakeforcec
	else:
		wv *= (1.0-brake)*0.5
		brakeforce -= (rayvelocity2.z+contactforce)
	# -----------------
	
	var weight:float = n_car.mass
	
	n_car.apply_impulse((n_geometry.global_transform.origin-n_car.global_transform.origin),(n_contact_axis_force.global_transform.origin-global_transform.origin)/weight)
	
	animations()

func differentials() -> void:
	if is_instance_valid(n_diff_connection):
		var manipulate = 0.0
		if wv>n_diff_connection.wv:
			manipulate = (wv - n_diff_connection.wv)*n_car.Preload
		else:
			manipulate = (wv - n_diff_connection.wv)*n_car.CoastPreload
		wv -= manipulate
		n_diff_connection.wv += manipulate
		
		var the = (n_diff_connection.wv - wv)*2.0
		the -= 5.0
		the *= 0.05
		
		var ythe = currentgrip - n_diff_connection.currentgrip
		
		the = clamp(the + (ythe * 0.001), 0.0, 1.0/(n_car.Locking +1))
		
		currentconnection = 1/(the +1)
		
		currentconnection *= Connection
		n_car.dsweight += Connection
	
	else:
		n_diff_connection = n_car.get_node(Differential_Connection)
		currentconnection = Connection
		n_car.dsweight += Connection
		

func driveshaft() -> void:
	var tvd:float = (n_car.rpm/n_car.ratio - wv)

	#var tvd2:float = abs(tvd)

	var clutchon:float = n_car.clutchon*n_car.clutchon

	if not n_car.gear == 0 and n_car.dsweightrun>0	:
		var dist:float = 0.0
		if n_car.gear == -1:
			dist = abs(n_car.GearRatios[0] - n_car.ReverseRatio)
		else:
			dist = abs(n_car.GearRatios[0] - n_car.GearRatios[n_car.gear-1])
		var stab:float = n_car.ClutchStability -(dist*n_car.StabiliseGears)
		if stab<0:
			 stab = 0

		var css:float = stab*35.0
		var dss:float = stab*12.5
		
		var rat:float = abs(n_car.ratio*(n_car.ratio +1.0))
			
		if rat>n_car.StabilityThreshold:
			rat = n_car.StabilityThreshold
		
		css *= rat/100.0
		dss *= rat/100.0
			
		var bite1:float = clamp((tvd/css) / 2.0, -1.0, 1.0) * clutchon
		var bite2:float = clamp((tvd/dss) / 2.0, -3.0, 3.0) * clutchon
		
		var wforce:float = 0.0
		if n_car.dsweightrun>0:
			if n_car.ratio>0:
				n_car.resistance -= ((bite1*(600.0*n_car.ClutchGrip))*currentconnection)/n_car.dsweightrun
			else:
				n_car.resistance += ((bite1*(600.0*n_car.ClutchGrip))*currentconnection)/n_car.dsweightrun
			wforce = (bite2*(n_car.torquedrag*(n_car.DriveShaftGrip/wheelweight)))*((currentconnection*2.0)/n_car.dsweightrun )
		wforce *= 2.0
		wv += wforce
		contactforce += wforce
		var tvddebug:float = (n_car.speedrpm/n_car.ratio - wv)
		tvddebug = tvddebug*currentconnection
		var bitedebug:float = tvddebug*10.0
		if n_car.ratio>0.0:
			n_car.set("resistance2",n_car.resistance2 - bitedebug)
		else:
			n_car.set("resistance2",n_car.resistance2 + bitedebug)

func animations() -> void:
	n_animation.global_transform.origin = n_geometry.global_transform.origin
#	n_animation.global_transform.origin = orgin
	n_animation.translation.y += cast_to.y-cast_current
	n_animation_spinning.rotation_degrees.x += wv
