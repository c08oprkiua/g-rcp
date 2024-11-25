extends Camera

onready var car_node:RigidBody = get_parent().get_node("car")

func _process(_delta):
	if is_instance_valid(car_node):
		look_at(car_node.translation+Vector3(0,2,0),Vector3(0,1,0))
		translation = car_node.translation+Vector3(0,2,0)
		translate_object_local(Vector3(0,0,7))
		rotate_object_local(Vector3(1,0,0),-0.25)
