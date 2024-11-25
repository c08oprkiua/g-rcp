extends RayCast

# external (could be adjusted for individual wheels by another script)
var tyre_code = "0250-185-060-14" # the first 4-row is the grip amount. set to "0250" by default
var roughness = 0.25
var forward_grip_deficiency = 1.0
var backward_grip_deficiency = 4.0

var WheelScale = 0.5
var TyreGrip = 1.0
var RealisticTyres = false # uses tyre_code to determine grip and wheel scale

var stiffness = 475.0 # Spring stiffness
var elasticity = 10.0 # Spring rebound rate
var damp = 1.0 # Rebound Dampening

# placeholder
var stiffness_struf = 2.5
var elasticity_struf = 75.0
var StrutDistance = 1.0
#---

var stiffness_swaybar = 0.0 # Swaybar Stiffness


var Connection = 0.0 # Connection between the driveshaft and the wheel itself.
var BrakeInfluence = 0.0 # The amount of stopping torque taken from the "BrakeStrength" property when pressing the brake.
var HandbrakeInfluence = 0.0 # The amount of stopping torque taken from the "BrakeStrength" property when pulling the handbrake.
var Camber = 0.0 # The slant angle of the wheel.
var Caster = 0.0 # Steering caster angle.
var Toe = 0.0 # Toe-In Angle
var Rest = 0.5 # Suspension Rest Distance
var Offset = 0.0 # Hub Offset
var StrutOffset = 0.0 # (WIP)
var SteerAngle_Left = 0.0 # Left steering angle
var SteerAngle_Right = 0.0 # Right steering angle
var Suspension_Geometry = 25.0 # Higher numbers causes more negative camber upon compression.
var Differential_Connection = "" # (WIP) Connects the differential to another wheel.
var SwayBar_Connection = "" # Connects the sway bar to another wheel's axle.

var abs_strength = 1.0 # TCS Sensitivity
var tcs_strength = 1.0 # ABS Sensitivity
var esp_strength = 1.0 # ESP Sensitivity

# internal
var grip = int(tyre_code.substr(0,4))
var tyrewidth = int(tyre_code.substr(5,3))
var tyrear = int(tyre_code.substr(9,4))
var rimsize = int(tyre_code.substr(13,2))
var tread = tyrear


var wheelsize =  (( float(tyrewidth)*((float(tyrear)*2.0)/100.0) + float(rimsize)*25.4 )*0.003269)/4.0
#var wheelsize =  0.2

var q = ((wheelsize/((1.0*0.003269)/2.0))*0.003269)/2.0
var tyrelc = ((wheelsize/((1.0*0.003269)/2.0))*q)/125.0

var lateraldamp = float(tread)*0.0085
var wheelweight = wheelsize*2.0
var tyreprofile = float(tread)/10.0
var coefficiency = float(tyrewidth)*0.1
var contact = 8.0
var tyrecompressrate = 0.9
var tyrecompressiongripmultiply = float(tyrewidth)/500.0
var thread = 1.0
var compress = 0.0
var compress2 = 0.0


var rigidity = 60.0
var tyrerigidity = 60.0

# system
var forcedata = Vector2(0,0)
var wv = 0.0
var tyrecompressed = 0.0
var tyrecompressedgrip = 0.0
var tyrecompressedscrub = 0.0
var currentgrip = 0.0
var gripscrub = 0.0
var wheelangle = 0.0
var currentcamber = 0.0
var wheelcompression = 0.0
var patch = Vector2(0,0)
var scrub = 0.0
var dist = 0.0
var contactforce = 0.0
var brakeforce = 0.0
var currentconnection = 0.0
var slip = 0.0
var slipz = 0.0
var brokencontact = 0.0
var brokencontactspin = 0.0
var skidspin = 0.0
var skid = 0.0
var skid2 = 0.0

var currentstif = 0.0
var currentelast = 0.0

var wsing = 0.0


# effects
var cgroundmaterial = 0.0
var bumpy = 0.0
var bumpycurrent = 0.0
var bumpfrequency = 0.0
var bumpfrequencyrandomize = 0.0
var bumpinverted = false
var griploss = 0.0
var currentelasticity = 0.0
var currentstiffness = 0.0

var cast_current = 0.0

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
		get_parent().GearAssistant[2] = wheelsize


func _ready():
	add_exception(get_parent())
	add_exception(n_velocity)
	add_exception(n_velocity2)
	n_geometry.translation = cast_to
	n_velocity.global_transform.origin = global_transform.origin
	n_velocity2.global_transform.origin = n_geometry.global_transform.origin

	if WheelScale:
		wheelsize = WheelScale
		grip = TyreGrip*250

func _physics_process(delta):
#	print(get_parent().get_node("test").linear_velocity)
#	print(n_velocity.linear_velocity)
	n_geometry.visible = get_parent().get("visualisation")
	get_parent().wheels += 1
	n_velocity2.global_transform.basis = n_axis.global_transform.basis
	n_velocity.global_transform.basis = n_axis.global_transform.basis
	var rayvelocity = n_velocity.global_transform.basis.orthonormalized().xform_inv(n_velocity.get_linear_velocity())
	var rayvelocity2 = n_velocity2.global_transform.basis.orthonormalized().xform_inv(n_velocity2.get_linear_velocity())
	var rayvelocity2velocity = Vector2(rayvelocity2.x,rayvelocity2.z).length()
	
	n_velocity2.global_transform.basis = n_contact_axis.global_transform.basis
	n_velocity.global_transform.basis = n_contact_axis.global_transform.basis
	
	var c_rayvelocity = n_velocity.global_transform.basis.orthonormalized().xform_inv(n_velocity.get_linear_velocity())
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
		currentcamber = -mcpherson - Camber + Caster*get_parent().get("steer")
	else:
		currentcamber = -mcpherson + Camber + Caster*get_parent().get("steer")

	n_animation.rotation_degrees.z = currentcamber
	
	var wheelinclinement = clamp(rotation_za/90.0 +currentcamber/90.0, -0.25, 0.25)
	
	wheelangle = abs((wheelinclinement -scrub/90.0)/(tyreprofile/10.0))
	
	if wheelangle>1.0:
		wheelangle = 1.0

	compress = 0.0

	var st = 0.0

	if get_parent().get("steer")<0:
		st = get_parent().get("steer")*SteerAngle_Left
	else:
		st = get_parent().get("steer")*SteerAngle_Right
	
	var toe = Toe
	if translation.x<0:
		toe = -toe
	
	rotation_degrees.y = st -toe

	# differential
	if not Differential_Connection == "":
		var linkedwheel = get_parent().get_node(Differential_Connection)
		var manipulate = 0.0
		if wv>linkedwheel.wv:
			manipulate = (wv - linkedwheel.wv)*get_parent().Preload
		else:
			manipulate = (wv - linkedwheel.wv)*get_parent().CoastPreload
		wv -= manipulate
		linkedwheel.wv += manipulate


		var the = (linkedwheel.wv - wv)*2.0
		the -= 5.0
		the *= 0.05

		var ythe = currentgrip - linkedwheel.currentgrip
		the += ythe*0.001

		if the<0:
			 the = 0
		elif the>1.0/(get_parent().Locking +1):
			 the = 1.0/(get_parent().Locking +1)

		currentconnection = 1/(the +1)

		currentconnection *= Connection
		get_parent().dsweight += Connection
		
	else:
		currentconnection = Connection
		get_parent().dsweight += Connection
		
#	print(currentconnection)

	# ----
	if is_colliding():

		#suspension
		n_geometry.global_transform.origin = get_collision_point()
		n_axis.global_transform = alignAxisToVector(n_axis.global_transform,get_collision_normal())
		n_contact_axis.global_transform = alignAxisToVector(n_contact_axis.global_transform,get_collision_normal())

		bumpfrequency = 0.0
		bumpy = 0.0
		bumpfrequencyrandomize = 0.0
		griploss = 0.0
		cgroundmaterial = 0.0
		get_parent().wheelsonground += 1
		
		if not get_collider().get("groundmaterial") == null:
			bumpfrequency = get_collider().get("bumpfrequency")
			bumpy = get_collider().get("bumpy")
			bumpfrequencyrandomize = get_collider().get("bumpfrequencyrandomize")
			griploss = get_collider().get("griploss")
			cgroundmaterial = get_collider().get("groundmaterial")

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
		
		get_parent().cgroundmaterial += cgroundmaterial

		cast_current = cast_to.y +bumpycurrent

		var scvelo = rayvelocity.y
		if scvelo>0.0:
			scvelo *= damp

		compress2 = (n_geometry.translation.y-cast_current -Rest)*currentelast
#		compress2 *= 1.0
		if compress2<0:
			compress2 = 0
		
		if (n_geometry.translation.y-cast_current)>Rest :
			compress = (scvelo - compress2)*currentstif
		if compress>0:
			compress = 0
			
		wheelcompression = -compress
		n_contact_axis_force.translation.y = -compress
		
		tyrecompressed = (((wheelcompression)/1000.0)*tyrecompressrate)*(-(wheelangle)+1)
		tyrecompressed *= 1.0 -wheelangle
		tyrecompressedscrub = (wheelcompression/1000.0)*tyrecompressrate
		tyrecompressedscrub *= 2.0

		if tyrecompressed<0.0:
			tyrecompressed = 0.0
#		elif tyrecompressed>tyrelc:
#			tyrecompressed = tyrelc

		if tyrecompressedscrub<0.0:
			tyrecompressedscrub = 0.0

		var decline = (tyrecompressed*tyrecompressed)*(0.8/tyrelc) - (0.43/tyrelc)
		
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
		
			
#		currentgrip = (((( (float(grip)*((float(tyrewidth)/(float(tyrewidth)/2))/1.5))/(get_parent().mass/150) )/(sliped*((roughness*(-(cgroundmaterial)+1))) +1))/(griploss+1))/1.1)*(tyrecompressedgrip*tyrecompressiongripmultiply) *0.825
		var griplimit = ((tyrecompressedgrip*float(grip))*2.5)/(sliped*((roughness*(-(cgroundmaterial)+1))) +1)

		griplimit /= griploss +1.0

		var deficiency = float((get_parent().mass*2.0)-float(tyrewidth))
		deficiency /= 500.0
#		print(deficiency)
		if deficiency>0.9999:
			deficiency = 0.9999
		
		if deficiency<0.0 or RealisticTyres == false:
			deficiency = 0.0

		currentgrip = griplimit*(1.0 - deficiency)
			

		gripscrub =  griplimit
#		gripscrub *= 0.0
		#-----
		
		
		# idfk
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
			var infl = 1.0/br
			if infl<1.0:
				infl = 1.0
			patch.y /= infl
		else:
			patch.y *= 0.95
		
		var distz = c_rayvelocity2.z - ((wv+(thectf))*wheelsize) + patch.y
		var distzw = c_rayvelocity2.z - ((wv+(thectf))*wheelsize)
		var distx = c_rayvelocity2.x + patch.x
		var distz3 = c_rayvelocity2.z - (wv*wheelsize)
		var distz4 = c_rayvelocity2.z - ((wv+(thectf*-0.5))*wheelsize) + patch.y
		var slip = abs(distz) + abs(distx)
#		slipz = max(abs(distx),abs(distz))
		slipz = abs(distx*2.5) + abs(distz)
		var longimode = clamp(rayvelocity2velocity - abs(wv)*wheelsize -currentgrip/1000.0, 0.0, 1.0)
		
		var rolldirt = min(abs(wv), 10.0)
		
		var the_yes = min((abs(distz) + abs(distx))*2.5 + rolldirt, 100.0)
		
		get_parent().wheelsforce += the_yes
		
		#----------
		
		#forces
		var amountz = 0.0
		var amountzw = 0.0
		var amountx = 0.0
		skid = 0.0
		skid2 = 0.0
		var offsettedx = 1.0
		var offsettedz = 1.0
		var offsettedzw = 1.0
		
		if tyrecompressed>0.0:
			var slip2 = (abs(distz3) +abs(distx))*(0.1/tyrecompressed)
			if slip2<0.0:
				slip2 = 0.0
			var thevelo = (abs(wv)/1.5)/(slip2 +1.0)
#			thevelo /= thevelo*0.001 +1.0
			var mass = get_parent().mass/100
			var unita = 75.0*mass
			unita /= unita*0.0107 +1.0
			var unitb = float(tyrewidth)/8.0
			var unitc = 1.0
			var unitd = 0.0015/(mass/currentgrip)
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
			var ok = ((tyrecompressed*w)/contact)*2.0
			
			if ok>1.0:
				ok = 1.0

			brokencontactspin = abs(distz3)*(thread/(tyrecompressed/contact +1.0)) -(limt*0.1)
			brokencontact = Vector2(abs(distz3),abs(distx)).length()*(thread/(tyrecompressed/contact +1.0)) -(limt*0.1)
			skidspin = brokencontactspin
			skid = brokencontact
			skid2 = Vector2(abs(distz3),abs(distx)).length()
			
			brokencontact = clamp(brokencontact, 0.0, 1.0)
			brokencontactspin = clamp(brokencontactspin, 0.0, 1.0)
			
			var patchdistancefromcenter = (n_geometry.global_transform.origin - get_parent().global_transform.origin).length()
			
			var farx = distx*patchdistancefromcenter
			var farz = distz*patchdistancefromcenter
			farx /= tyrecompressed+1.0
			farz /= tyrecompressed+1.0
			if translation.x>0:
				farx *= -1.0
			if translation.z>0:
				farz *= -1.0
			
			wsing = clamp(farz/2.5 -abs(farx)*0.1, 0.0, 0.75/(cgroundmaterial +1.0))
			
#			wsing = 0.0
				
#			print(wsing)
			var going = rayvelocity2velocity
			
			going /= wheelsize*100.0
			
			going -= 0.25
			var going2 = clamp(going*2.0, 0.0, 1.0)
			
			if going<1.0-(wsing/0.75):
				 going = 1.0-(wsing/0.75)
			going = clamp(going, 0.0, 1.0-wsing)
			
#			going = 1.0
			
#			wsing = 1.0

			var siding = abs(c_rayvelocity2.x/(rayvelocity2velocity +1.0))
			siding *= 5.0
			if siding>1.0:
				siding = 1.0

			var declinet = siding + wsing
			if declinet>1.0:
				declinet = 1.0

			var declinet2 = siding + going
			if declinet2>1.0:
				declinet2 = 1.0

			var declinet3 = going - siding*1.0
			if declinet3<0.0:
				declinet3 = 0.0
			
#			var AAAA = distz3*(1.0-wsing) + distz4*wsing

			var dapedz = abs(distz4*(1.0-declinet2) +distz3*declinet2)*1.0 -(offsettedz*declinet3)*(1.0-declinet +wsing)
#			var dapedz = abs(distz3) -offsettedz*2.0
			if dapedz<0.0:
				 dapedz = 0.0

			var dapedx = abs(distx)*1.0 -(offsettedx*going)
#			var dapedx = abs(distx)
			if dapedx<0.0:
				 dapedx = 0.0
			
			var method1 = Vector2(Vector2(abs(distz),dapedx).length() -offsettedz,Vector2(dapedz*1.0,abs(distx)).length())
			var method2 = Vector2(abs(distz) + dapedx - offsettedz,dapedz*1.0 + abs(distx))
			var methodw = Vector2(abs(distz),dapedx*2.0).length() -offsettedz

#			var methodtest = Vector2(abs(distz),abs(distx)).length()

#			wsing = 1.0

			var dampz = method1.x*(-(wsing)+1.0) + method2.x*wsing
			var dampw = methodw*1.0
			var dampx = method1.y*(-(wsing)+1.0) + method2.y*wsing

#			dampz = methodtest
#			dampx = methodtest

			if dampx<0.0:
				dampx = 0.0
			if dampz<0.0:
				dampz = 0.0
			if dampw<0.0:
				dampw = 0.0
			
#			var dampw = (abs(distz) + abs(distx*1.0))*2.0 -offsettedz
#			var dampw = Vector2(abs(dampz),abs(distx)*2.0).length()*2.0 -offsettedz

			amountz = distz/(dampz/offsettedz +1.0) /offsettedz
			amountzw = distzw/(dampw/offsettedzw +1.0) /offsettedzw
			amountx = distx/(dampx/offsettedx +1.0) /offsettedx
			forcedata = [abs(distx),abs(distz)]
			n_contact_axis.rotation_degrees.y = 0

		var longitudinal = amountz*(currentgrip*2.0)
		var longitudinalw = amountzw*(currentgrip*2.0)
		var lateral = amountx*(currentgrip*2.0)
		var lateralscrub = rayvelocity2.x*(gripscrub*2.0)
		
		n_contact_axis_force.translation.x = -lateral/2.0
		n_contact_axis_force.translation.z = -longitudinal/2.0
		
		scrub = clamp(lateralscrub/(float(tyrewidth)*180.0), -90.0, 90.0)
		
		scrub /= (abs(scrub)*deg2rad(1.0) ) +1
		
		var wvok = min(abs(wv)/10.0, 1.0)
		
		scrub *= wvok
		
		wv += (longitudinalw/wheelweight)/50.0
		#------
		
		var h = clamp(skid*0.5 -abs(wv)*(lateraldamp/10.0) - 0.75, 0.0, 1.0)
		
		get_parent().set("skidding",get_parent().get("skidding")+h)

		var h2 = skid2
		if h2<0:
			h2 = 0
		get_parent().set("skidding2",get_parent().get("skidding2")+h2)

		#debug
		n_geometry_compress.scale = Vector3(0.01,tyrecompressed/0.25,0.01)
		n_geometry_compress.translation.y = n_geometry_compress.scale.y/2
		n_geometry_longi.scale = Vector3(0.01,0.01,n_contact_axis_force.translation.z/500.0)
		n_geometry_longi.translation.z = n_geometry_longi.scale.z/2.0
		n_geometry_lateral.scale = Vector3(n_contact_axis_force.translation.x/500.0,0.01,0.01)
		n_geometry_lateral.translation.x = n_geometry_lateral.scale.x/2.0
		
		n_geometry_compress.translation.y -= wheelsize/2.0
		n_geometry_lateral.translation.y = -wheelsize/2.0
		n_geometry_longi.translation.y = -wheelsize/2.0

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
	
	if not SwayBar_Connection == "":
		var linkedwheel = get_parent().get_node(SwayBar_Connection)
		var rolldist = dist - linkedwheel.get("dist")
		if rolldist<0.0:
			rolldist = 0.0
		currentelast = elasticity*(rolldist*stiffness_swaybar +1)
		currentstif = stiffness*(rolldist*stiffness_swaybar +1)
	else:
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
	var brslip = slipz
	brslip -= get_parent().get("ABS")[1]
	if brslip<0:
		brslip = 0
	elif get_parent().get("brake")>0.5 and get_parent().get("ABS")[4] and rayvelocity2velocity>get_parent().get("ABS")[2]:
		get_parent().set("absflashed", true)
			
	var absenabled = false
	
	if rayvelocity2velocity>get_parent().get("ABS")[2]:
		absenabled = get_parent().get("ABS")[4]
			
	var br = 0.0
			
	if absenabled:
		if get_parent().get("ABS")[5]:
			var brakae = get_parent().get("brake")
			if brakae>get_parent().get("ABS")[3]:
				 brakae = get_parent().get("ABS")[3]
			br = brakae*(-(brslip*(get_parent().get("ABS")[0]))+1)
		else:
			get_parent().brakethreshold += brslip/50.0
			br = (get_parent().get("brake")*get_parent().get("brake")) - get_parent().brakethresholdrun
			br *= get_parent().get("ABS")[3]
	else:
		br = get_parent().get("brake")*get_parent().get("brake")

	var brake = (br*BrakeInfluence)+((get_parent().get("handbrake"))*HandbrakeInfluence)
	var brake2 = brake
	
#	brake *= brake
		
	var espb = 0.0
		
	if translation.x>0.0:
		espb = (get_parent().angular_velocity.y)*esp_strength -get_parent().get("ESP")[1]
	else:
		espb = (-get_parent().angular_velocity.y)*esp_strength -get_parent().get("ESP")[1]

	if espb<0:
		espb = 0
	elif espb>(get_parent().get("ESP")[0]*get_parent().get("ESP")[2]):
		espb = (get_parent().get("ESP")[0]*get_parent().get("ESP")[2])
	elif espb>0 and get_parent().get("ESP")[2]:
		get_parent().set("espflashed", true)

	var tcs = slipz*((get_parent().get("TCS")[0]*tcs_strength)*get_parent().get("TCS")[2]) -get_parent().get("TCS")[1]
	if tcs<0:
		tcs = 0
	elif tcs>0 and get_parent().get("TCS")[2]:
		get_parent().set("tcsflashed", true)

	brake += espb +tcs

	if brake<0:
		brake = 0
	elif brake>1:
		brake = 1
	
	#-----
	
	#driveshaft
	var tvd = (get_parent().get("rpm")/get_parent().get("ratio") - wv)

	var tvd2 = abs(tvd)

	var clutchon = get_parent().get("clutchon")*get_parent().get("clutchon")

	if not get_parent().get("gear") == 0 and get_parent().dsweightrun>0	:
		var dist = 0.0
		if get_parent().gear == -1:
			dist = abs(get_parent().GearRatios[0] - get_parent().ReverseRatio)
		else:
			dist = abs(get_parent().GearRatios[0] - get_parent().GearRatios[get_parent().gear-1])
		var stab = get_parent().ClutchStability -(dist*get_parent().StabiliseGears)
		if stab<0:
			 stab = 0

		var css = stab*35.0
		var dss = stab*12.5
		
		var rat = abs(get_parent().get("ratio")*(get_parent().get("ratio") +1.0))
			
		if rat>get_parent().get("StabilityThreshold"):
			rat = get_parent().get("StabilityThreshold")
		
		css *= rat/100.0
		dss *= rat/100.0
			
		var bite1 = tvd/css
		var bite2 = tvd/dss
		bite1 /= 2.0
		bite2 /= 2.0
		if bite1>1.0:
			bite1 = 1.0
		elif bite1<-1.0:
			bite1 = -1.0
		if bite2>3.0:
			bite2 = 3.0
		elif bite2<-3.0:
			bite2 = -3.0
		bite1 *= clutchon
		bite2 *= clutchon
		var wforce = 0.0
		if get_parent().get("dsweightrun")>0:
			if get_parent().get("ratio")>0:
				get_parent().resistance -= ((bite1*(600.0*get_parent().get("ClutchGrip")))*currentconnection)/get_parent().get("dsweightrun")
			else:
				get_parent().resistance += ((bite1*(600.0*get_parent().get("ClutchGrip")))*currentconnection)/get_parent().get("dsweightrun")
			wforce = (bite2*(get_parent().get("torquedrag")*(get_parent().get("DriveShaftGrip")/wheelweight)))*((currentconnection*2.0)/get_parent().get("dsweightrun") )
		wforce *= 2.0
		wv += wforce
		contactforce += wforce
		var tvddebug = (get_parent().get("speedrpm")/get_parent().get("ratio") - wv)
		tvddebug = tvddebug*currentconnection
		var bitedebug = tvddebug*10.0
		if get_parent().get("ratio")>0.0:
			get_parent().set("resistance2",get_parent().get("resistance2") - bitedebug)
		else:
			get_parent().set("resistance2",get_parent().get("resistance2") + bitedebug)
	#---------------

	# brake repositioned
	brakeforce = 0.0
	var brakeforcec = get_parent().get("BrakeStrength")*(brake*BrakeInfluence)
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
		
	var weight = get_parent().mass

	get_parent().apply_impulse((n_geometry.global_transform.origin-get_parent().global_transform.origin)*1.0,(n_contact_axis_force.global_transform.origin-global_transform.origin)/weight)

	# animations
	n_animation.global_transform.origin = n_geometry.global_transform.origin
#	n_animation.global_transform.origin = orgin
	n_animation.translation.y += cast_to.y-cast_current
	n_animation_spinning.rotation_degrees.x += wv
	
