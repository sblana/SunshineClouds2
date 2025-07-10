@tool
@icon("res://addons/SunshineClouds2/CloudsDriverIcon.svg")
extends Node
class_name SunshineCloudsDriverGD

@export var update_continuously: bool = false:
	get:
		return update_continuously
	set(value):
		update_continuously = value
		retrieve_texture_data()

@export_tool_button("Generate Clouds Resource", "Add") var generate_action = build_new_clouds
#@export_tool_button("Test Clouds Position Sample", "Add") var position_sample = sample_clouds


@export_group("Compositor Resource")
@export var clouds_resource: SunshineCloudsGD
@export_group("Optional World Environment")
@export var ambience_sample_environment: Environment
@export var use_enviornment_ambience_for_fog: bool = false
@export_group("Light Controls")
@export var tracked_directional_lights: Array[DirectionalLight3D] = []:
	get:
		return tracked_directional_lights
	set(value):
		tracked_directional_lights = value
		retrieve_texture_data()

@export var tracked_directional_light_shadow_steps: Array[int] = []:
	get:
		return tracked_directional_light_shadow_steps
	set(value):
		tracked_directional_light_shadow_steps = value
		retrieve_texture_data()

@export var tracked_point_lights: Array[OmniLight3D] = []:
	get:
		return tracked_point_lights
	set(value):
		tracked_point_lights = value
		retrieve_texture_data()

@export var tracked_point_effectors: Array[SunshineCloudsEffector] = []:
	get:
		return tracked_point_effectors
	set(value):
		tracked_point_effectors = value
		retrieve_texture_data()

@export var directional_light_power_multiplier: float = 1.0
@export var point_light_power_multiplier: float = 1.0
@export_group("Wind Controls")
@export var origin_offset : Vector3 = Vector3.ZERO
@export var wind_direction: Vector3 = Vector3(1.0, 0.0, 1.0)
@export var extra_large_structures_wind_speed: float = 140.0
@export var large_structures_wind_speed: float = 100.0
@export var medium_structures_wind_speed: float = 40.0
@export var small_structures_wind_speed: float = 12.0
@export_group("Internal Use")
var extra_large_clouds_pos: Vector3 = Vector3.ZERO
var large_clouds_pos: Vector3 = Vector3.ZERO
var medium_clouds_pos: Vector3 = Vector3.ZERO
var small_clouds_pos: Vector3 = Vector3.ZERO

var _extralarge_clouds_domain: float = 0.0
var _large_clouds_domain: float = 0.0
var _medium_clouds_domain: float = 0.0
var _small_clouds_domain: float = 0.0

var _updating_settings: bool = false

func _ready():
	
	if update_continuously:
		
		if clouds_resource == null:
			update_continuously = false
			return
		call_deferred("retrieve_texture_data")


func _process(delta : float):
	if clouds_resource != null:
		clouds_resource.current_time = wrap(clouds_resource.current_time + delta * clouds_resource.dither_speed, 0.0, clouds_resource.dither_speed * 64.0)
		
		if update_continuously:
			_updating_settings = false
			extra_large_clouds_pos += wind_direction * extra_large_structures_wind_speed * delta
			extra_large_clouds_pos = wrap_vector(extra_large_clouds_pos, _extralarge_clouds_domain)
			large_clouds_pos += wind_direction * large_structures_wind_speed * delta
			large_clouds_pos = wrap_vector(large_clouds_pos, _large_clouds_domain)
			medium_clouds_pos += wind_direction * medium_structures_wind_speed * delta
			medium_clouds_pos = wrap_vector(medium_clouds_pos, _medium_clouds_domain)
			small_clouds_pos += ((wind_direction * small_structures_wind_speed) + (Vector3.UP * abs(small_structures_wind_speed))) * delta
			small_clouds_pos = wrap_vector(small_clouds_pos, _small_clouds_domain)
			
			clouds_resource.origin_offset = origin_offset
			clouds_resource.extra_large_scale_clouds_position = origin_offset + extra_large_clouds_pos
			clouds_resource.large_scale_clouds_position = origin_offset + large_clouds_pos
			clouds_resource.medium_scale_clouds_position = origin_offset + medium_clouds_pos
			clouds_resource.detail_clouds_position = origin_offset + small_clouds_pos
			
			clouds_resource.wind_direction = wind_direction
			
			if use_enviornment_ambience_for_fog and ambience_sample_environment != null:
				clouds_resource.atmosphere_color = ambience_sample_environment.fog_light_color
				clouds_resource.cloud_ambient_color = ambience_sample_environment.fog_light_color
			
			if (tracked_directional_lights.size() * 2.0 != clouds_resource.directional_lights_data.size() \
			or tracked_point_lights.size() * 2.0 != clouds_resource.point_lights_data.size()\
			or tracked_directional_lights.size() != tracked_directional_light_shadow_steps.size()):
				
				retrieve_texture_data()
				return
			
			for i in range(tracked_directional_lights.size()):
				if tracked_directional_lights[i] == null:
					continue
				if direction_light_data_changed(tracked_directional_lights[i], tracked_directional_light_shadow_steps[i], clouds_resource.directional_lights_data[i * 2], clouds_resource.directional_lights_data[i * 2 + 1]):
					retrieve_texture_data()
					return
			
			for i in range(tracked_point_lights.size()):
				if tracked_point_lights[i] == null:
					continue
				if point_light_data_changed(tracked_point_lights[i], clouds_resource.point_lights_data[i * 2], clouds_resource.point_lights_data[i * 2 + 1]):
					retrieve_texture_data()
					return
			
			for i in range(tracked_point_effectors.size()):
				if tracked_point_effectors[i] == null:
					continue
				if point_effector_data_changed(tracked_point_effectors[i], clouds_resource.point_effector_data[i * 2], clouds_resource.point_effector_data[i * 2 + 1]):
					retrieve_texture_data()
					return
			
	else:
		update_continuously = false

func sample_clouds():
	for i in range(64):
		clouds_resource.add_sample(return_data.bind(), Vector3(i * 1000, 6000.0, 0.0))

func return_data(position : Vector3, sampledensity : float):
	print(position, " ", sampledensity)
	pass

func build_new_clouds():
	if (is_inside_tree()):
		var env : WorldEnvironment = recursively_find_env(get_tree().root)
		if (env):
			if not ambience_sample_environment:
				ambience_sample_environment = env.environment
			
			var newClouds = SunshineCloudsGD.new()
			ResourceSaver.save(newClouds, "res://NewCloudsGD.tres")
			clouds_resource = ResourceLoader.load("res://NewCloudsGD.tres")
			
			if not env.compositor:
				env.compositor = Compositor.new()
			env.compositor.compositor_effects = [clouds_resource]
			
			update_continuously = true
		else:
			printerr("No world environment found.")


func recursively_find_env(thisNode: Node) -> WorldEnvironment:
	for child in thisNode.get_children():
		if child is WorldEnvironment:
			return child
		else:
			var result = recursively_find_env(child)
			if (result):
				return result
	
	return null

func retrieve_texture_data():
	if _updating_settings || !is_inside_tree():
		return
	_updating_settings = true
	if clouds_resource != null:
		_extralarge_clouds_domain = clouds_resource.extra_large_noise_scale / 2.0
		_large_clouds_domain = clouds_resource.large_noise_scale / 2.0
		_medium_clouds_domain = clouds_resource.medium_noise_scale / 2.0
		_small_clouds_domain = clouds_resource.small_noise_scale / 2.0
		
		clouds_resource.directional_lights_data.clear()
		clouds_resource.point_lights_data.clear()
		clouds_resource.point_effector_data.clear()
		
		if not tracked_directional_light_shadow_steps:
			tracked_directional_light_shadow_steps = []
		
		if tracked_directional_light_shadow_steps.size() < tracked_directional_lights.size():
			for i in range(tracked_directional_lights.size() - tracked_directional_light_shadow_steps.size()):
				tracked_directional_light_shadow_steps.append(12)
		
		for i in range(tracked_directional_lights.size()):
			if tracked_directional_lights[i] != null:
				var light = tracked_directional_lights[i]
				var look_dir = light.global_transform.basis.z.normalized()
				clouds_resource.directional_lights_data.append(Vector4(look_dir.x, look_dir.y, look_dir.z, tracked_directional_light_shadow_steps[i]))
				clouds_resource.directional_lights_data.append(Vector4(light.light_color.r, light.light_color.g, light.light_color.b, round(light.light_color.a * light.light_energy * directional_light_power_multiplier * 10.0) / 10.0))
		
		for i in range(tracked_point_lights.size()):
			if tracked_point_lights[i] != null:
				var light = tracked_point_lights[i]
				var light_pos = light.global_position
				clouds_resource.point_lights_data.append(Vector4(light_pos.x, light_pos.y, light_pos.z, light.omni_range))
				clouds_resource.point_lights_data.append(Vector4(light.light_color.r, light.light_color.g, light.light_color.b, round(light.light_color.a * light.light_energy * point_light_power_multiplier * 10.0) / 10.0))
		
		
		for i in range(tracked_point_effectors.size()):
			if tracked_point_effectors[i] != null:
				var node = tracked_point_effectors[i]
				var node_pos = node.global_position
				clouds_resource.point_effector_data.append(Vector4(node_pos.x, node_pos.y, node_pos.z, node.Radius))
				clouds_resource.point_effector_data.append(Vector4(node.Power, 0.0, 0.0, 0.0))
		
		
		clouds_resource.lights_updated = true
	
	_updating_settings = false

func wrap_vector(target, domain_size):
	if target.x > domain_size:
		target.x -= domain_size * 2.0
	elif target.x < -domain_size:
		target.x += domain_size * 2.0

	if target.y > domain_size:
		target.y -= domain_size * 2.0
	elif target.y < -domain_size:
		target.y += domain_size * 2.0

	if target.z > domain_size:
		target.z -= domain_size * 2.0
	elif target.z < -domain_size:
		target.z += domain_size * 2.0

	return target

func direction_light_data_changed(light : DirectionalLight3D, shadowCount : int, dirData : Vector4, colorData : Vector4):
	return light.global_transform.basis.z.x != dirData.x \
		or light.global_transform.basis.z.y != dirData.y \
		or light.global_transform.basis.z.z != dirData.z \
		or float(shadowCount) != dirData.w \
		or light.light_color.r != colorData.x \
		or light.light_color.g != colorData.y \
		or light.light_color.b != colorData.z \
		or round(light.light_color.a * light.light_energy * directional_light_power_multiplier * 10.0) / 10.0 != round(colorData.w * 10.0) / 10.0


func point_light_data_changed(light : OmniLight3D, dirData : Vector4, colorData : Vector4):
	return light.global_position.x != dirData.x \
		or light.global_position.y != dirData.y \
		or light.global_position.z != dirData.z \
		or light.omni_range != dirData.w \
		or light.light_color.r != colorData.x \
		or light.light_color.g != colorData.y \
		or light.light_color.b != colorData.z \
		or round(light.light_color.a * light.light_energy * point_light_power_multiplier * 10.0) / 10.0 != round(colorData.w * 10.0) / 10.0

func point_effector_data_changed(node : SunshineCloudsEffector, dirData : Vector4, colorData : Vector4):
	return node.global_position.x != dirData.x \
		or node.global_position.y != dirData.y \
		or node.global_position.z != dirData.z \
		or node.Radius != dirData.w \
		or node.Power != colorData.x
