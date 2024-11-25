extends Control

export var enabled:bool = true

onready var fps:Label = $monitors/fps
onready var rpm:Label = $monitors/rpm
onready var gear:Label = $monitors/gear
onready var kph:Label = $monitors/kph
onready var torque:Label = $monitors/torque
onready var rpm_speed:Label = $monitors/rpm_speed
onready var force:Label = $monitors/force
onready var turbo_psi:Label = $monitors/turbo_psi

onready var steering_wheel:TextureRect = $sw

onready var car:SeVeCar = get_parent().get_node("car")

func _ready() -> void:
	if not enabled:
		set_process(false)
		hide()

func _process(delta:float) -> void:
	if delta > 0 and enabled:
		fps.text = "fps: "+String(1.0/delta)
		
		rpm.text = "rpm: "+String(car.rpm)
		gear.text = "gear: "+String(car.gear)
		kph.text = "kph: "+String(int(car.linear_velocity.length()*2.2))
		torque.text = "torque: "+String(car.tq)
		rpm_speed.text = "rpmspeed: "+String(car.speedrpm)
		force.text = "force: "+String(car.gforce)
		turbo_psi.text = "turbo psi: "+String(car.psi)
		steering_wheel.rect_rotation = -car.steer*400
