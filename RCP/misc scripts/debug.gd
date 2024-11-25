extends Control

onready var fps:Label = $monitors/fps
onready var rpm:Label = $monitors/rpm
onready var gear:Label = $monitors/gear
onready var kph:Label = $monitors/kph
onready var torque:Label = $monitors/torque
onready var rpm_speed:Label = $monitors/rpm_speed
onready var force:Label = $monitors/force
onready var turbo_psi:Label = $monitors/turbo_psi

onready var steering_wheel:TextureRect = $sw

onready var car:RigidBody = get_parent().get_node("car")

func _process(delta):
	if delta>0:
		fps.text = "fps: "+str(1.0/delta)
		rpm.text = "rpm: "+str(car.get("rpm"))
		gear.text = "gear: "+str(car.get("gear"))
		kph.text = "kph: "+str(int(car.linear_velocity.length()*2.2))
		torque.text = "torque: "+str(car.get("tq"))
		rpm_speed.text = "rpmspeed: "+str(car.get("speedrpm"))
		force.text = "force: "+str(car.get("gforce"))
		turbo_psi.text = "turbo psi: "+str(car.get("psi"))
		steering_wheel.rect_rotation = -car.get("steer")*400
