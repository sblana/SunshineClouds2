class_name TreeTestEffect
extends CompositorEffect

var update_octree : bool = false


@export_group("Basic Settings")
@export_range(0, 1) var clouds_coverage : float = 0.6:
	set(f):
		clouds_coverage = f
		update_octree = true
@export_range(0, 20) var clouds_density : float = 1.0
@export_range(0, 2) var atmospheric_density : float = 0.5
@export_range(0, 10) var lighting_density : float = 0.55
@export_range(0, 1) var fog_effect_ground : float = 1.0

@export_subgroup("Colors")
@export_range(0, 1) var clouds_anisotropy : float = 0.3
@export_range(0, 1) var clouds_powder : float = 0.5
@export var cloud_ambient_color : Color = Color(0.352, 0.624, 0.784, 1.0)
@export var cloud_ambient_tint : Color = Color(0.352, 0.624, 0.784, 1.0)
@export var atmosphere_color : Color = Color(0.801, 0.893, 0.962, 1.0)
@export var ambient_occlusion_color : Color = Color(0.17, 0.044, 0.027, 0.549)

@export_subgroup("Structure")
@export_range(0, 1) var accumulation_decay : float = 0.5
@export_range(100, 1000000) var extra_large_noise_scale : float = 320000.0
@export_range(100, 500000) var large_noise_scale : float = 50000.0
@export_range(100, 100000) var medium_noise_scale : float = 6000.0
@export_range(100, 10000) var small_noise_scale : float = 2500.0
@export_range(0, 2) var clouds_sharpness : float = 1.0
@export_range(0, 3) var clouds_detail_power : float = 0.9
@export_range(0, 50000) var curl_noise_strength : float = 5000.0
@export_range(0, 2) var lighting_sharpness : float = 0.05
@export_range(0, 1) var wind_swept_range : float = 0.5
@export_range(0, 5000) var wind_swept_strength : float = 500.0

@export var cloud_floor : float = 1500.0
@export var cloud_ceiling : float = 25000.0

@export_subgroup("Performance")
@export var max_step_count : float = 50
@export var max_lighting_steps : float = 32
@export_enum("Native","Half","Quarter","Eighth") var resolution_scale = 0:
	get:
		return resolution_scale
	set(value):
		resolution_scale = value
		last_size = Vector2i(0, 0)
		lights_updated = true
@export_range(0, 2) var lod_bias : float = 1.0

@export_subgroup("Noise Textures")
@export var dither_noise : Texture3D
@export var height_gradient : Texture2D
@export var extra_large_noise_patterns : Texture2D
@export var large_scale_noise : Texture3D
@export var medium_scale_noise : Texture3D
@export var small_scale_noise : Texture3D
@export var curl_noise : Texture3D

@export_group("Advanced Settings")
@export_subgroup("Visuals")
@export_range(0, 1000) var dither_speed : float = 100.8254
@export_range(0, 20) var blur_power : float = 2.0
@export_range(0, 6) var blur_quality : float = 1.0

@export_subgroup("Reflections")
@export var reflections_globalshaderparam : String = ""

@export_subgroup("Performance")
@export var min_step_distance : float = 100.0
@export var max_step_distance : float = 600.0
@export var lighting_travel_distance : float = 5000.0

@export_subgroup("Mask")
@export var extra_large_used_as_mask : bool = false
@export var mask_width_km : float = 32.0;

@export_group("Compute Shaders")
@export var pre_pass_compute_shader : RDShaderFile
@export var compute_shader : RDShaderFile
@export var post_pass_compute_shader : RDShaderFile
#region mine
@export var tree_create_compute_shader : RDShaderFile
@export var tree_march_compute_shader : RDShaderFile
#region

@export_group("Internal Use")
@export var origin_offset : Vector3 = Vector3.ZERO
@export_subgroup("Positions")
@export var wind_direction : Vector3 = Vector3.ZERO
@export var extra_large_scale_clouds_position : Vector3 = Vector3.ZERO
@export var large_scale_clouds_position : Vector3 = Vector3.ZERO
@export var medium_scale_clouds_position : Vector3 = Vector3.ZERO
@export var detail_clouds_position : Vector3 = Vector3.ZERO
@export var current_time : float = 0.0

@export_subgroup("Lights")
@export var directional_lights_data : Array[Vector4] = []
@export var point_lights_data : Array[Vector4] = []
@export var point_effector_data : Array[Vector4] = []

var positionQueries : Array[Vector3] = []
var positionQueryCallables : Array[Callable] = []
var positionQuerying : bool = false
var positionResetting : bool = false

var lights_updated = false

var maskDrawnRid : RID = RID()

var rd : RenderingDevice
var shader : RID = RID()
var pipeline : RID = RID()

var prepass_shader : RID = RID()
var prepass_pipeline : RID = RID()

var postpass_shader : RID = RID()
var postpass_pipeline : RID = RID()

#region mine
var treecreate_shader : RID = RID()
var treecreate_pipeline : RID = RID()

var treemarch_shader : RID = RID()
var treemarch_pipeline : RID = RID()

var tree_node_data_byte_size : int = (4+4)*4
var tree_buffer_byte_size : int = tree_node_data_byte_size * 65536*8
var tree_buffer : RID = RID()
var uniform_set1 : RID = RID()
#endregion

var nearest_sampler : RID = RID()
var linear_sampler : RID = RID()
var linear_sampler_no_repeat : RID = RID()

var general_data_buffer : RID = RID()
var light_data_buffer : RID = RID()
var point_sample_data_buffer : RID = RID()
var accumulation_textures : Array[RID] = []
var resized_depth : RID = RID()
var push_constants : PackedByteArray
var prepass_push_constants : PackedByteArray
var postpass_push_constants : PackedByteArray
var last_size : Vector2i = Vector2i(0, 0)
var color_images : Array[RID] = []
var msaa_color_images : Array[RID] = []

var buffers : RenderSceneBuffersRD


var uniform_sets : Array[RID] = []
var general_data : PackedByteArray

var light_data : PackedByteArray

var accumulation_is_a : bool = false

var last_view_mat : Transform3D
var last_projection_mat : Projection
var first_run : bool = true
var filter_index = 0

func refresh_compute():
	maskDrawnRid = RID()
	last_size = Vector2i.ZERO

func update_mask(newMask : RID):
	maskDrawnRid = newMask
	last_size = Vector2i.ZERO

func add_sample(callable : Callable, position : Vector3):
	#if (positionQueries.size() == 32):
		#print("Max cloud position sample queue reached (32), query failed.")
		#return
	positionQueries.append(position)
	positionQueryCallables.append(callable)

func _init():
	effect_callback_type = CompositorEffect.EFFECT_CALLBACK_TYPE_PRE_TRANSPARENT
	access_resolved_depth = true
	access_resolved_color = true
	needs_motion_vectors = true
	RenderingServer.call_on_render_thread(initialize_compute)

func _notification(what):
	if what == NOTIFICATION_PREDELETE and is_instance_valid(self):
		RenderingServer.call_on_render_thread(clear_compute)

func clear_compute():
	if rd:
		if shader.is_valid():
			rd.free_rid(shader)
		shader = RID()
		
		if prepass_shader.is_valid():
			rd.free_rid(prepass_shader)
		prepass_shader = RID()
		
		if postpass_shader.is_valid():
			rd.free_rid(postpass_shader)
		postpass_shader = RID()
		
		#region mine
		if treecreate_shader.is_valid():
			rd.free_rid(treecreate_shader)
		treecreate_shader = RID()

		if treemarch_shader.is_valid():
			rd.free_rid(treemarch_shader)
		treemarch_shader = RID()
		
		if tree_buffer.is_valid():
			rd.free_rid(tree_buffer)
		tree_buffer = RID()
		#endregion
		
		if nearest_sampler.is_valid():
			rd.free_rid(nearest_sampler)
		nearest_sampler = RID()
		
		if linear_sampler.is_valid():
			rd.free_rid(linear_sampler)
		linear_sampler = RID()
		
		if linear_sampler_no_repeat.is_valid():
			rd.free_rid(linear_sampler_no_repeat)
		linear_sampler_no_repeat = RID()
		
		if general_data_buffer.is_valid():
			rd.free_rid(general_data_buffer)
		general_data_buffer = RID()
		
		if light_data_buffer.is_valid():
			rd.free_rid(light_data_buffer)
		light_data_buffer = RID()
		
		if point_sample_data_buffer.is_valid():
			rd.free_rid(point_sample_data_buffer)
		point_sample_data_buffer = RID()
		
		if resized_depth.is_valid():
			rd.free_rid(resized_depth)
		resized_depth = RID()
		
		if accumulation_textures.size() > 0:
			for item in accumulation_textures:
				if item.is_valid():
					rd.free_rid(item)
			accumulation_textures.clear()
		
		if msaa_color_images.size() > 0:
			for item in msaa_color_images:
				if item.is_valid():
					rd.free_rid(item)
			msaa_color_images.clear()


func initialize_compute():
	first_run = true
	if not rd:
		rd = RenderingServer.get_rendering_device()
		if not rd:
			enabled = false
			printerr("No rendering device on load.")
			return
	clear_compute()
	
	var sampler_state = RDSamplerState.new()
	sampler_state.min_filter = RenderingDevice.SAMPLER_FILTER_NEAREST
	sampler_state.mag_filter = RenderingDevice.SAMPLER_FILTER_NEAREST
	sampler_state.repeat_u = RenderingDevice.SAMPLER_REPEAT_MODE_REPEAT
	sampler_state.repeat_v = RenderingDevice.SAMPLER_REPEAT_MODE_REPEAT
	sampler_state.repeat_w = RenderingDevice.SAMPLER_REPEAT_MODE_REPEAT
	nearest_sampler = rd.sampler_create(sampler_state)
	
	var linear_sampler_state = RDSamplerState.new()
	linear_sampler_state.min_filter = RenderingDevice.SAMPLER_FILTER_LINEAR
	linear_sampler_state.mag_filter = RenderingDevice.SAMPLER_FILTER_LINEAR
	linear_sampler_state.repeat_u = RenderingDevice.SAMPLER_REPEAT_MODE_REPEAT
	linear_sampler_state.repeat_v = RenderingDevice.SAMPLER_REPEAT_MODE_REPEAT
	linear_sampler_state.repeat_w = RenderingDevice.SAMPLER_REPEAT_MODE_REPEAT
	linear_sampler = rd.sampler_create(linear_sampler_state)
	
	var linear_sampler_state_no_repeat = RDSamplerState.new()
	linear_sampler_state_no_repeat.min_filter = RenderingDevice.SAMPLER_FILTER_LINEAR
	linear_sampler_state_no_repeat.mag_filter = RenderingDevice.SAMPLER_FILTER_LINEAR
	linear_sampler_state_no_repeat.repeat_u = RenderingDevice.SAMPLER_REPEAT_MODE_CLAMP_TO_EDGE
	linear_sampler_state_no_repeat.repeat_v = RenderingDevice.SAMPLER_REPEAT_MODE_CLAMP_TO_EDGE
	linear_sampler_state_no_repeat.repeat_w = RenderingDevice.SAMPLER_REPEAT_MODE_CLAMP_TO_EDGE
	linear_sampler_no_repeat = rd.sampler_create(linear_sampler_state_no_repeat)
	
	if not dither_noise:
		dither_noise = ResourceLoader.load("res://addons/SunshineClouds2/NoiseTextures/bluenoise_Dither.png")
	if not height_gradient:
		height_gradient = ResourceLoader.load("res://addons/SunshineClouds2/NoiseTextures/HeightGradient.tres")
	if not extra_large_noise_patterns:
		extra_large_noise_patterns = ResourceLoader.load("res://addons/SunshineClouds2/NoiseTextures/ExtraLargeScaleNoise.tres")
	if not large_scale_noise:
		large_scale_noise = ResourceLoader.load("res://addons/SunshineClouds2/NoiseTextures/LargeScaleNoise.tres")
	if not medium_scale_noise:
		medium_scale_noise = ResourceLoader.load("res://addons/SunshineClouds2/NoiseTextures/MediumScaleNoise.tres")
	if not small_scale_noise:
		small_scale_noise = ResourceLoader.load("res://addons/SunshineClouds2/NoiseTextures/SmallScaleNoise.tres")
	if not curl_noise:
		curl_noise = ResourceLoader.load("res://addons/SunshineClouds2/NoiseTextures/curl_noise_varied.tga")
	
	if not compute_shader:
		compute_shader = ResourceLoader.load("res://addons/SunshineClouds2/SunshineCloudsCompute.glsl")
	if not pre_pass_compute_shader:
		pre_pass_compute_shader = ResourceLoader.load("res://addons/SunshineClouds2/SunshineCloudsPreCompute.glsl")
	if not post_pass_compute_shader:
		post_pass_compute_shader = ResourceLoader.load("res://addons/SunshineClouds2/SunshineCloudsPostCompute.glsl")
	if not compute_shader or not pre_pass_compute_shader or not post_pass_compute_shader:
		enabled = false
		printerr("No Shader found on load.")
		clear_compute()
		return
	
	#region mine
	if not tree_create_compute_shader:
		tree_create_compute_shader = ResourceLoader.load("res://treetest/tree_create.glsl")
	if not tree_march_compute_shader:
		tree_march_compute_shader = ResourceLoader.load("res://treetest/tree_march.glsl")
	#endregion
	
	var prepass_shader_spirv = pre_pass_compute_shader.get_spirv()
	prepass_shader = rd.shader_create_from_spirv(prepass_shader_spirv)
	if prepass_shader.is_valid():
		prepass_pipeline = rd.compute_pipeline_create(prepass_shader)
	else:
		enabled = false
		printerr("Prepass Shader failed to compile.")
		clear_compute()
		return
	
	
	var shader_spirv = compute_shader.get_spirv()
	shader = rd.shader_create_from_spirv(shader_spirv)
	if shader.is_valid():
		pipeline = rd.compute_pipeline_create(shader)
	else:
		enabled = false
		printerr("Shader failed to compile.")
		clear_compute()
		return
	
	
	var postpass_shader_spirv = post_pass_compute_shader.get_spirv()
	postpass_shader = rd.shader_create_from_spirv(postpass_shader_spirv)
	if postpass_shader.is_valid():
		postpass_pipeline = rd.compute_pipeline_create(postpass_shader)
	else:
		enabled = false
		printerr("Post pass Shader failed to compile.")
		clear_compute()
		return
	
	#region mine
	var treecreate_shader_spirv = tree_create_compute_shader.get_spirv()
	treecreate_shader = rd.shader_create_from_spirv(treecreate_shader_spirv)
	if treecreate_shader.is_valid():
		treecreate_pipeline = rd.compute_pipeline_create(treecreate_shader)
	else:
		enabled = false
		printerr("Tree create Shader failed to compile.")
		clear_compute()
		return
	var treemarch_shader_spirv = tree_march_compute_shader.get_spirv()
	treemarch_shader = rd.shader_create_from_spirv(treemarch_shader_spirv)
	if treemarch_shader.is_valid():
		treemarch_pipeline = rd.compute_pipeline_create(treemarch_shader)
	else:
		enabled = false
		printerr("Tree march Shader failed to compile.")
		clear_compute()
		return

	var treebuffer_bytes := PackedByteArray()
	treebuffer_bytes.resize(tree_buffer_byte_size)
	treebuffer_bytes.fill(0)
	if tree_buffer.is_valid():
		rd.free_rid(tree_buffer)
	tree_buffer = rd.storage_buffer_create(tree_buffer_byte_size, treebuffer_bytes)
	#endregion

func _render_callback(effect_callback_type, render_data):
	if rd == null:
		initialize_compute()
	elif pipeline.is_valid() and height_gradient and extra_large_noise_patterns and large_scale_noise and medium_scale_noise and small_scale_noise and dither_noise and curl_noise:
		buffers = render_data.get_render_scene_buffers() as RenderSceneBuffersRD
		if buffers:
			var msaa = buffers.get_msaa_3d() != 0
			if msaa:
				return
			
			var size = buffers.get_internal_size()
			if size.x == 0 and size.y == 0:
				return
			
			var resscale = 1
			match resolution_scale:
				0: 
					resscale = 1
				1: 
					resscale = 2
				2: 
					resscale = 4
				3: 
					resscale = 8
			
			var new_size = size / resscale
			var view_count = buffers.get_view_count()
			if size != last_size or uniform_sets == null or uniform_sets.size() != view_count * 3 or color_images.size() == 0 or color_images[0] != buffers.get_color_layer(0):
				update_octree = true
				initialize_compute()
				
				accumulation_textures.clear()
				uniform_sets.clear()
				
				var prepass_data_ms = StreamPeerBuffer.new()
				prepass_data_ms.put_float(size.x)
				prepass_data_ms.put_float(size.y)
				prepass_data_ms.put_float(resscale)
				prepass_data_ms.put_float(0.0)
				
				prepass_push_constants = prepass_data_ms.data_array
				#print("prepass_push_constants",prepass_push_constants.size())
				
				var postpass_data_ms = StreamPeerBuffer.new()
				postpass_data_ms.put_float(new_size.x)
				postpass_data_ms.put_float(new_size.y)
				postpass_data_ms.put_float(resscale)
				postpass_data_ms.put_float(0.0)
				
				postpass_push_constants = postpass_data_ms.data_array
				color_images.clear()
				msaa_color_images.clear()
				
				#print("postpass_push_constants",postpass_push_constants.size())
				for view in range(view_count):
					color_images.append(buffers.get_color_layer(view, msaa))
					
					var depth_image : RID = buffers.get_depth_layer(view, msaa)
					
					var blankImageData : PackedByteArray = []
					blankImageData.resize(new_size.x * new_size.y * 4 * 4)
					
					var base_colorformat : RDTextureFormat = rd.texture_get_format(color_images[view])
					
					
					if (msaa):
						base_colorformat.usage_bits = RenderingDevice.TEXTURE_USAGE_STORAGE_BIT | RenderingDevice.TEXTURE_USAGE_SAMPLING_BIT | RenderingDevice.TEXTURE_USAGE_CAN_COPY_TO_BIT | RenderingDevice.TEXTURE_USAGE_CAN_COPY_FROM_BIT
						
						msaa_color_images.append(rd.texture_create(base_colorformat, RDTextureView.new(), []))
					
					base_colorformat.format = RenderingDevice.DATA_FORMAT_R32G32B32A32_SFLOAT
					base_colorformat.width = new_size.x
					base_colorformat.height = new_size.y
					
					accumulation_textures.append(rd.texture_create(base_colorformat, RDTextureView.new(), [blankImageData]))
					accumulation_textures.append(rd.texture_create(base_colorformat, RDTextureView.new(), [blankImageData]))
					accumulation_textures.append(rd.texture_create(base_colorformat, RDTextureView.new(), [blankImageData]))
					accumulation_textures.append(rd.texture_create(base_colorformat, RDTextureView.new(), [blankImageData]))
					accumulation_textures.append(rd.texture_create(base_colorformat, RDTextureView.new(), [blankImageData]))
					accumulation_textures.append(rd.texture_create(base_colorformat, RDTextureView.new(), [blankImageData]))
					
					#reflections
					accumulation_textures.append(rd.texture_create(base_colorformat, RDTextureView.new(), [blankImageData]))
					
					
					var depthformat : RDTextureFormat = rd.texture_get_format(depth_image)
					depthformat.width = new_size.x
					depthformat.height = new_size.y
					depthformat.format = RenderingDevice.DATA_FORMAT_R32_SFLOAT
					depthformat.usage_bits = RenderingDevice.TEXTURE_USAGE_STORAGE_BIT | RenderingDevice.TEXTURE_USAGE_SAMPLING_BIT
					resized_depth = rd.texture_create(depthformat, RDTextureView.new(), [])
					
					#Prepass Compute Shader
					var prepass_uniforms_array : Array[RDUniform] = []
					var prepass_depth_uniform = RDUniform.new()
					prepass_depth_uniform.uniform_type = RenderingDevice.UNIFORM_TYPE_SAMPLER_WITH_TEXTURE
					prepass_depth_uniform.binding = 0
					prepass_depth_uniform.add_id(nearest_sampler)
					prepass_depth_uniform.add_id(depth_image)
					prepass_uniforms_array.append(prepass_depth_uniform)
					
					var prepass_depth_output_uniform = RDUniform.new()
					prepass_depth_output_uniform.uniform_type = RenderingDevice.UNIFORM_TYPE_IMAGE
					prepass_depth_output_uniform.binding = 1
					prepass_depth_output_uniform.add_id(resized_depth)
					prepass_uniforms_array.append(prepass_depth_output_uniform)
					uniform_sets.append(rd.uniform_set_create(prepass_uniforms_array, prepass_shader, 0))
					
					#Base Compute Shader
					var uniforms_array : Array[RDUniform] = []
					var output_data_uniform = RDUniform.new()
					output_data_uniform.uniform_type = RenderingDevice.UNIFORM_TYPE_IMAGE
					output_data_uniform.binding = 0
					output_data_uniform.add_id(accumulation_textures[view * 7])
					uniforms_array.append(output_data_uniform)
					
					var output_color_uniform = RDUniform.new()
					output_color_uniform.uniform_type = RenderingDevice.UNIFORM_TYPE_IMAGE
					output_color_uniform.binding = 1
					output_color_uniform.add_id(accumulation_textures[view * 7 + 1])
					uniforms_array.append(output_color_uniform)
					
					var accum1A_uniform = RDUniform.new()
					accum1A_uniform.uniform_type = RenderingDevice.UNIFORM_TYPE_IMAGE
					accum1A_uniform.binding = 2
					accum1A_uniform.add_id(accumulation_textures[view * 7 + 2])
					uniforms_array.append(accum1A_uniform)
					
					var accum1B_uniform = RDUniform.new()
					accum1B_uniform.uniform_type = RenderingDevice.UNIFORM_TYPE_IMAGE
					accum1B_uniform.binding = 3
					accum1B_uniform.add_id(accumulation_textures[view * 7 + 3])
					uniforms_array.append(accum1B_uniform)
					
					var accum2A_uniform = RDUniform.new()
					accum2A_uniform.uniform_type = RenderingDevice.UNIFORM_TYPE_IMAGE
					accum2A_uniform.binding = 4
					accum2A_uniform.add_id(accumulation_textures[view * 7 + 4])
					uniforms_array.append(accum2A_uniform)
					
					var accum2B_uniform = RDUniform.new()
					accum2B_uniform.uniform_type = RenderingDevice.UNIFORM_TYPE_IMAGE
					accum2B_uniform.binding = 5
					accum2B_uniform.add_id(accumulation_textures[view * 7 + 5])
					uniforms_array.append(accum2B_uniform)
					
					var depth_uniform = RDUniform.new()
					depth_uniform.uniform_type = RenderingDevice.UNIFORM_TYPE_SAMPLER_WITH_TEXTURE
					depth_uniform.binding = 6
					depth_uniform.add_id(nearest_sampler)
					depth_uniform.add_id(resized_depth)
					uniforms_array.append(depth_uniform)
					
					var extra_noise_uniform = RDUniform.new()
					extra_noise_uniform.uniform_type = RenderingDevice.UNIFORM_TYPE_SAMPLER_WITH_TEXTURE
					extra_noise_uniform.binding = 7
					extra_noise_uniform.add_id(linear_sampler)
					extra_noise_uniform.add_id(maskDrawnRid if extra_large_used_as_mask && maskDrawnRid.is_valid() else RenderingServer.texture_get_rd_texture(extra_large_noise_patterns.get_rid()))
					uniforms_array.append(extra_noise_uniform)
					
					var noise_uniform = RDUniform.new()
					noise_uniform.uniform_type = RenderingDevice.UNIFORM_TYPE_SAMPLER_WITH_TEXTURE
					noise_uniform.binding = 8
					noise_uniform.add_id(linear_sampler)
					noise_uniform.add_id(RenderingServer.texture_get_rd_texture(large_scale_noise.get_rid()))
					uniforms_array.append(noise_uniform)
					
					var medium_noise_uniform = RDUniform.new()
					medium_noise_uniform.uniform_type = RenderingDevice.UNIFORM_TYPE_SAMPLER_WITH_TEXTURE
					medium_noise_uniform.binding = 9
					medium_noise_uniform.add_id(linear_sampler)
					medium_noise_uniform.add_id(RenderingServer.texture_get_rd_texture(medium_scale_noise.get_rid()))
					uniforms_array.append(medium_noise_uniform)
					
					var small_noise_uniform = RDUniform.new()
					small_noise_uniform.uniform_type = RenderingDevice.UNIFORM_TYPE_SAMPLER_WITH_TEXTURE
					small_noise_uniform.binding = 10
					small_noise_uniform.add_id(linear_sampler)
					small_noise_uniform.add_id(RenderingServer.texture_get_rd_texture(small_scale_noise.get_rid()))
					uniforms_array.append(small_noise_uniform)
					
					var curl_noise_uniform = RDUniform.new()
					curl_noise_uniform.uniform_type = RenderingDevice.UNIFORM_TYPE_SAMPLER_WITH_TEXTURE
					curl_noise_uniform.binding = 11
					curl_noise_uniform.add_id(linear_sampler)
					curl_noise_uniform.add_id(RenderingServer.texture_get_rd_texture(curl_noise.get_rid()))
					uniforms_array.append(curl_noise_uniform)
					
					var dither_noise_uniform = RDUniform.new()
					dither_noise_uniform.uniform_type = RenderingDevice.UNIFORM_TYPE_SAMPLER_WITH_TEXTURE
					dither_noise_uniform.binding = 12
					dither_noise_uniform.add_id(nearest_sampler)
					dither_noise_uniform.add_id(RenderingServer.texture_get_rd_texture(dither_noise.get_rid()))
					uniforms_array.append(dither_noise_uniform)
					
					var height_gradient_uniform = RDUniform.new()
					height_gradient_uniform.uniform_type = RenderingDevice.UNIFORM_TYPE_SAMPLER_WITH_TEXTURE
					height_gradient_uniform.binding = 13
					height_gradient_uniform.add_id(linear_sampler_no_repeat)
					height_gradient_uniform.add_id(RenderingServer.texture_get_rd_texture(height_gradient.get_rid()))
					uniforms_array.append(height_gradient_uniform)
					
					general_data_buffer = rd.uniform_buffer_create(464)
					var camera_uniform = RDUniform.new()
					camera_uniform.uniform_type = RenderingDevice.UNIFORM_TYPE_UNIFORM_BUFFER
					camera_uniform.binding = 14
					camera_uniform.add_id(general_data_buffer)
					uniforms_array.append(camera_uniform)
					
					light_data_buffer = rd.uniform_buffer_create(6272)
					var light_data_uniform = RDUniform.new()
					light_data_uniform.uniform_type = RenderingDevice.UNIFORM_TYPE_UNIFORM_BUFFER
					light_data_uniform.binding = 15
					light_data_uniform.add_id(light_data_buffer)
					uniforms_array.append(light_data_uniform)
					
					var sampleData : PackedByteArray = []
					sampleData.resize(512)
					point_sample_data_buffer = rd.storage_buffer_create(512, sampleData)
					var point_sample_data_uniform = RDUniform.new()
					point_sample_data_uniform.uniform_type = RenderingDevice.UNIFORM_TYPE_STORAGE_BUFFER
					point_sample_data_uniform.binding = 16
					point_sample_data_uniform.add_id(point_sample_data_buffer)
					uniforms_array.append(point_sample_data_uniform)
					
					uniform_sets.append(rd.uniform_set_create(uniforms_array, shader, 0))
					
					#Post Pass Compute Shader
					var postpass_uniforms_array : Array[RDUniform] = []
					var prepass_color_data_uniform = RDUniform.new()
					prepass_color_data_uniform.uniform_type = RenderingDevice.UNIFORM_TYPE_SAMPLER_WITH_TEXTURE
					prepass_color_data_uniform.binding = 0
					prepass_color_data_uniform.add_id(linear_sampler_no_repeat)
					prepass_color_data_uniform.add_id(accumulation_textures[view * 7])
					postpass_uniforms_array.append(prepass_color_data_uniform)
					
					var prepass_color_uniform = RDUniform.new()
					prepass_color_uniform.uniform_type = RenderingDevice.UNIFORM_TYPE_SAMPLER_WITH_TEXTURE
					prepass_color_uniform.binding = 1
					prepass_color_uniform.add_id(linear_sampler_no_repeat)
					prepass_color_uniform.add_id(accumulation_textures[view * 7 + 1])
					postpass_uniforms_array.append(prepass_color_uniform)
					
					var postpass_reflections_uniform = RDUniform.new()
					postpass_reflections_uniform.uniform_type = RenderingDevice.UNIFORM_TYPE_IMAGE
					postpass_reflections_uniform.binding = 2
					postpass_reflections_uniform.add_id(accumulation_textures[view * 7 + 6])
					postpass_uniforms_array.append(postpass_reflections_uniform)
					
					if (reflections_globalshaderparam != ""):
						var newTexture = Texture2DRD.new()
						newTexture.texture_rd_rid = accumulation_textures[view * 7 + 6]
						RenderingServer.global_shader_parameter_set(reflections_globalshaderparam, newTexture)
					
					var postpass_color_uniform = RDUniform.new()
					postpass_color_uniform.uniform_type = RenderingDevice.UNIFORM_TYPE_IMAGE
					postpass_color_uniform.binding = 3
					postpass_color_uniform.add_id(msaa_color_images[view] if msaa else color_images[view])
					postpass_uniforms_array.append(postpass_color_uniform)
					
					var postpass_depth_uniform = RDUniform.new()
					postpass_depth_uniform.uniform_type = RenderingDevice.UNIFORM_TYPE_SAMPLER_WITH_TEXTURE
					postpass_depth_uniform.binding = 4
					postpass_depth_uniform.add_id(nearest_sampler)
					postpass_depth_uniform.add_id(depth_image)
					postpass_uniforms_array.append(postpass_depth_uniform)
					
					var postpass_camera_uniform = RDUniform.new()
					postpass_camera_uniform.uniform_type = RenderingDevice.UNIFORM_TYPE_UNIFORM_BUFFER
					postpass_camera_uniform.binding = 5
					postpass_camera_uniform.add_id(general_data_buffer)
					postpass_uniforms_array.append(postpass_camera_uniform)
					
					var postpass_light_data_uniform = RDUniform.new()
					postpass_light_data_uniform.uniform_type = RenderingDevice.UNIFORM_TYPE_UNIFORM_BUFFER
					postpass_light_data_uniform.binding = 6
					postpass_light_data_uniform.add_id(light_data_buffer)
					postpass_uniforms_array.append(postpass_light_data_uniform)
					
					uniform_sets.append(rd.uniform_set_create(postpass_uniforms_array, postpass_shader, 0))

					#region mine
					var tree_buffer_uniform = RDUniform.new()
					tree_buffer_uniform.uniform_type = RenderingDevice.UNIFORM_TYPE_STORAGE_BUFFER
					tree_buffer_uniform.binding = 0
					tree_buffer_uniform.add_id(tree_buffer)
					uniform_set1 = rd.uniform_set_create([tree_buffer_uniform], treecreate_shader, 1)
					#endregion
				
				lights_updated = true
			
			# Push constants and matrix updates
			var ms = StreamPeerBuffer.new()
			ms.put_float(new_size.x)
			ms.put_float(new_size.y)
			ms.put_float(large_noise_scale)
			ms.put_float(medium_noise_scale)
			
			ms.put_float(current_time)
			ms.put_float(clouds_coverage)
			ms.put_float(clouds_density)
			ms.put_float(clouds_detail_power)
			
			ms.put_float(lighting_density)
			ms.put_float(accumulation_decay)
			if (accumulation_is_a):
				ms.put_float(1.0)
			else:
				ms.put_float(0.0)
			ms.put_float(0.0)
			push_constants = ms.get_data_array()
			
			
			var rendersceneData : RenderSceneData = render_data.get_render_scene_data();
			var cameraTR : Transform3D = rendersceneData.get_cam_transform();
			var viewProj : Projection = rendersceneData.get_cam_projection();
			
			last_size = size
			
			update_matrices(cameraTR, viewProj)
			if lights_updated or directional_lights_data.size() == 0:
				update_lights()
			
			if (!positionQuerying && !positionResetting && positionQueries.size() > 0):
				encode_sample_points()
			
			# rd.buffer_clear(tree_buffer, 0, tree_buffer_byte_size)
			
			var prepass_x_groups = ((size.x - 1) / 8) + 1
			var prepass_y_groups = ((size.y - 1) / 8) + 1
			var x_groups = ((size.x - 1) / 8 / resscale) + 1
			var y_groups = ((size.y - 1) / 8 / resscale) + 1
			
			for view in view_count:
				if (msaa):
					rd.texture_copy(color_images[view], msaa_color_images[view], Vector3.ZERO, Vector3.ZERO, Vector3(size.x, size.y, 0.0),0,0,0,0)
				
				rd.draw_command_begin_label("SunshineClouds", Color.POWDER_BLUE)
				# var prepass_list = rd.compute_list_begin()
				# rd.compute_list_bind_compute_pipeline(prepass_list, prepass_pipeline)
				# rd.compute_list_bind_uniform_set(prepass_list, uniform_sets[view * 3], 0)
				# rd.compute_list_set_push_constant(prepass_list, prepass_push_constants, prepass_push_constants.size())
				# rd.compute_list_dispatch(prepass_list, x_groups, y_groups, 1)
				# rd.compute_list_end()

				if update_octree:
					var treecreate_list := rd.compute_list_begin()
					rd.compute_list_bind_compute_pipeline(treecreate_list, treecreate_pipeline)
					rd.compute_list_bind_uniform_set(treecreate_list, uniform_sets[view * 3 + 1], 0)
					rd.compute_list_bind_uniform_set(treecreate_list, uniform_set1, 1);
					rd.compute_list_set_push_constant(treecreate_list, push_constants, push_constants.size())
					rd.compute_list_dispatch(treecreate_list, 1, 1, 1)
					rd.compute_list_end()
					print("just ran")
					update_octree = false

				var treemarch_list := rd.compute_list_begin()
				rd.compute_list_bind_compute_pipeline(treemarch_list, treemarch_pipeline)
				rd.compute_list_bind_uniform_set(treemarch_list, uniform_sets[view * 3 + 1], 0)
				rd.compute_list_bind_uniform_set(treemarch_list, uniform_set1, 1);
				rd.compute_list_set_push_constant(treemarch_list, push_constants, push_constants.size())
				rd.compute_list_dispatch(treemarch_list, x_groups, y_groups, 1)
				rd.compute_list_end()

				# var compute_list = rd.compute_list_begin()
				# rd.compute_list_bind_compute_pipeline(compute_list, pipeline)
				# rd.compute_list_bind_uniform_set(compute_list, uniform_sets[view * 3 + 1], 0)
				# rd.compute_list_set_push_constant(compute_list, push_constants, push_constants.size())
				# rd.compute_list_dispatch(compute_list, x_groups, y_groups, 1)
				# rd.compute_list_end()

				var postpass_list = rd.compute_list_begin()
				rd.compute_list_bind_compute_pipeline(postpass_list, postpass_pipeline)
				rd.compute_list_bind_uniform_set(postpass_list, uniform_sets[view * 3 + 2], 0)
				rd.compute_list_set_push_constant(postpass_list, postpass_push_constants, postpass_push_constants.size())
				rd.compute_list_dispatch(postpass_list, prepass_x_groups, prepass_y_groups, 1)
				rd.compute_list_end()
				rd.draw_command_end_label()
				
				if (msaa):
					rd.texture_copy(msaa_color_images[view], color_images[view], Vector3.ZERO, Vector3.ZERO, Vector3(size.x, size.y, 0.0),0,0,0,0)
			
			if (!positionResetting && positionQuerying):
				positionResetting = true
				rd.buffer_get_data_async(point_sample_data_buffer, retrieve_position_queries.bind())
			#call_deferred("update_callbacktype", cameraTR.origin.y)
			#if (cameraTR.origin.y > cloud_floor):
				#if (self.effect_callback_type != CompositorEffect.EFFECT_CALLBACK_TYPE_POST_TRANSPARENT):
					#self.effect_callback_type = CompositorEffect.EFFECT_CALLBACK_TYPE_POST_TRANSPARENT
			#else:
				#if (self.effect_callback_type != CompositorEffect.EFFECT_CALLBACK_TYPE_PRE_TRANSPARENT):
					#self.effect_callback_type = CompositorEffect.EFFECT_CALLBACK_TYPE_PRE_TRANSPARENT

func retrieve_position_queries(data : PackedByteArray):
	
	var idx = 0
	while idx < 512 && positionQueryCallables.size() > 0:
		var position : Vector3 = Vector3.ZERO
		position.x = data.decode_float(idx)
		idx += 4
		position.y = data.decode_float(idx)
		idx += 4
		position.z = data.decode_float(idx)
		idx += 4
		var density = data.decode_float(idx)
		idx += 4
		
		positionQueryCallables[0].call(position, density)
		positionQueryCallables.remove_at(0)
		
	
	positionQuerying = false
	positionResetting = false


func update_callbacktype(lastY : float):
	if (lastY > cloud_floor):
		if (self.effect_callback_type != CompositorEffect.EFFECT_CALLBACK_TYPE_POST_TRANSPARENT):
			self.effect_callback_type = CompositorEffect.EFFECT_CALLBACK_TYPE_POST_TRANSPARENT
	else:
		if (self.effect_callback_type != CompositorEffect.EFFECT_CALLBACK_TYPE_PRE_TRANSPARENT):
			self.effect_callback_type = CompositorEffect.EFFECT_CALLBACK_TYPE_PRE_TRANSPARENT

func update_matrices(camera_tr, view_proj):
	if general_data.size() != 464: #32+32+44 * 4 bytes for each float = 432.
		general_data.resize(464)
	
	var idx = 0
	filter_index += 1
	if filter_index > 16:
		filter_index = 0
		# Camera matrix (16 floats)
	general_data.encode_float(idx, camera_tr.basis.x.x); idx += 4
	general_data.encode_float(idx, camera_tr.basis.x.y); idx += 4
	general_data.encode_float(idx, camera_tr.basis.x.z); idx += 4
	general_data.encode_float(idx, 0); idx += 4
	
	general_data.encode_float(idx, camera_tr.basis.y.x); idx += 4
	general_data.encode_float(idx, camera_tr.basis.y.y); idx += 4
	general_data.encode_float(idx, camera_tr.basis.y.z); idx += 4
	general_data.encode_float(idx, 0); idx += 4
	
	general_data.encode_float(idx, camera_tr.basis.z.x); idx += 4
	general_data.encode_float(idx, camera_tr.basis.z.y); idx += 4
	general_data.encode_float(idx, camera_tr.basis.z.z); idx += 4
	general_data.encode_float(idx, 0); idx += 4
	
	general_data.encode_float(idx, camera_tr.origin.x); idx += 4
	general_data.encode_float(idx, camera_tr.origin.y); idx += 4
	general_data.encode_float(idx, camera_tr.origin.z); idx += 4
	general_data.encode_float(idx, 1.0); idx += 4

	# Previous or current camera matrix
	var mat = camera_tr if first_run else last_view_mat
	general_data.encode_float(idx, mat.basis.x.x); idx += 4
	general_data.encode_float(idx, mat.basis.x.y); idx += 4
	general_data.encode_float(idx, mat.basis.x.z); idx += 4
	general_data.encode_float(idx, 0); idx += 4
	
	general_data.encode_float(idx, mat.basis.y.x); idx += 4
	general_data.encode_float(idx, mat.basis.y.y); idx += 4
	general_data.encode_float(idx, mat.basis.y.z); idx += 4
	general_data.encode_float(idx, 0); idx += 4
	
	general_data.encode_float(idx, mat.basis.z.x); idx += 4
	general_data.encode_float(idx, mat.basis.z.y); idx += 4
	general_data.encode_float(idx, mat.basis.z.z); idx += 4
	general_data.encode_float(idx, 0); idx += 4
	
	general_data.encode_float(idx, mat.origin.x); idx += 4
	general_data.encode_float(idx, mat.origin.y); idx += 4
	general_data.encode_float(idx, mat.origin.z); idx += 4
	general_data.encode_float(idx, 1.0); idx += 4

	# Projection matrix (16 floats)
	general_data.encode_float(idx, view_proj.x.x); idx += 4
	general_data.encode_float(idx, view_proj.x.y); idx += 4
	general_data.encode_float(idx, view_proj.x.z); idx += 4
	general_data.encode_float(idx, view_proj.x.w); idx += 4
	
	general_data.encode_float(idx, view_proj.y.x); idx += 4
	general_data.encode_float(idx, view_proj.y.y); idx += 4
	general_data.encode_float(idx, view_proj.y.z); idx += 4
	general_data.encode_float(idx, view_proj.y.w); idx += 4
	
	general_data.encode_float(idx, view_proj.z.x); idx += 4
	general_data.encode_float(idx, view_proj.z.y); idx += 4
	general_data.encode_float(idx, view_proj.z.z); idx += 4
	general_data.encode_float(idx, view_proj.z.w); idx += 4
	
	general_data.encode_float(idx, view_proj.w.x); idx += 4
	general_data.encode_float(idx, view_proj.w.y); idx += 4
	general_data.encode_float(idx, view_proj.w.z); idx += 4
	general_data.encode_float(idx, view_proj.w.w); idx += 4

	# Previous or current camera matrix
	var proj = view_proj if first_run else last_projection_mat
	general_data.encode_float(idx, proj.x.x); idx += 4
	general_data.encode_float(idx, proj.x.y); idx += 4
	general_data.encode_float(idx, proj.x.z); idx += 4
	general_data.encode_float(idx, proj.x.w); idx += 4
	
	general_data.encode_float(idx, proj.y.x); idx += 4
	general_data.encode_float(idx, proj.y.y); idx += 4
	general_data.encode_float(idx, proj.y.z); idx += 4
	general_data.encode_float(idx, proj.y.w); idx += 4
	
	general_data.encode_float(idx, proj.z.x); idx += 4
	general_data.encode_float(idx, proj.z.y); idx += 4
	general_data.encode_float(idx, proj.z.z); idx += 4
	general_data.encode_float(idx, proj.z.w); idx += 4
	
	general_data.encode_float(idx, proj.w.x); idx += 4
	general_data.encode_float(idx, proj.w.y); idx += 4
	general_data.encode_float(idx, proj.w.z); idx += 4
	general_data.encode_float(idx, proj.w.w); idx += 4

	last_projection_mat = view_proj
	last_view_mat = camera_tr
	accumulation_is_a = not accumulation_is_a
	first_run = false
	
	# Additional data (44 floats)
	var width = mask_width_km * 1000.0
	
	if (extra_large_used_as_mask):
		general_data.encode_float(idx, origin_offset.x + (width * 0.5) * -1.0); idx += 4
		general_data.encode_float(idx, origin_offset.y + (width * 0.5) * -1.0); idx += 4
		general_data.encode_float(idx, origin_offset.z + (width * 0.5) * -1.0); idx += 4
		general_data.encode_float(idx, width); idx += 4
	else:
		general_data.encode_float(idx, extra_large_scale_clouds_position.x); idx += 4
		general_data.encode_float(idx, extra_large_scale_clouds_position.y); idx += 4
		general_data.encode_float(idx, extra_large_scale_clouds_position.z); idx += 4
		general_data.encode_float(idx, extra_large_noise_scale); idx += 4
	
	#general_data.encode_float(idx, extra_large_scale_clouds_position.x); idx += 4
	#general_data.encode_float(idx, extra_large_scale_clouds_position.y); idx += 4
	#general_data.encode_float(idx, extra_large_scale_clouds_position.z); idx += 4
	#general_data.encode_float(idx, extra_large_noise_scale); idx += 4
	#
	general_data.encode_float(idx, large_scale_clouds_position.x); idx += 4
	general_data.encode_float(idx, large_scale_clouds_position.y); idx += 4
	general_data.encode_float(idx, large_scale_clouds_position.z); idx += 4
	general_data.encode_float(idx, lighting_sharpness); idx += 4

	general_data.encode_float(idx, medium_scale_clouds_position.x); idx += 4
	general_data.encode_float(idx, medium_scale_clouds_position.y); idx += 4
	general_data.encode_float(idx, medium_scale_clouds_position.z); idx += 4
	general_data.encode_float(idx, lighting_travel_distance); idx += 4

	general_data.encode_float(idx, detail_clouds_position.x); idx += 4
	general_data.encode_float(idx, detail_clouds_position.y); idx += 4
	general_data.encode_float(idx, detail_clouds_position.z); idx += 4
	general_data.encode_float(idx, atmospheric_density); idx += 4

	general_data.encode_float(idx, cloud_ambient_color.r * cloud_ambient_tint.r); idx += 4
	general_data.encode_float(idx, cloud_ambient_color.g * cloud_ambient_tint.g); idx += 4
	general_data.encode_float(idx, cloud_ambient_color.b * cloud_ambient_tint.b); idx += 4
	general_data.encode_float(idx, cloud_ambient_color.a * cloud_ambient_tint.a); idx += 4

	general_data.encode_float(idx, ambient_occlusion_color.r); idx += 4
	general_data.encode_float(idx, ambient_occlusion_color.g); idx += 4
	general_data.encode_float(idx, ambient_occlusion_color.b); idx += 4
	general_data.encode_float(idx, ambient_occlusion_color.a); idx += 4

	general_data.encode_float(idx, atmosphere_color.r); idx += 4
	general_data.encode_float(idx, atmosphere_color.g); idx += 4
	general_data.encode_float(idx, atmosphere_color.b); idx += 4
	general_data.encode_float(idx, atmosphere_color.a); idx += 4

	general_data.encode_float(idx, small_noise_scale); idx += 4
	general_data.encode_float(idx, min_step_distance); idx += 4
	general_data.encode_float(idx, max_step_distance); idx += 4
	general_data.encode_float(idx, lod_bias); idx += 4

	general_data.encode_float(idx, clouds_sharpness); idx += 4
	general_data.encode_float(idx, float(directional_lights_data.size()) / 2.0); idx += 4
	general_data.encode_float(idx, clouds_powder); idx += 4
	general_data.encode_float(idx, clouds_anisotropy); idx += 4

	general_data.encode_float(idx, cloud_floor); idx += 4
	general_data.encode_float(idx, cloud_ceiling); idx += 4
	general_data.encode_float(idx, float(max_step_count)); idx += 4
	general_data.encode_float(idx, float(max_lighting_steps)); idx += 4

	general_data.encode_float(idx, float(filter_index)); idx += 4
	general_data.encode_float(idx, float(blur_power)); idx += 4
	general_data.encode_float(idx, float(blur_quality)); idx += 4
	general_data.encode_float(idx, float(curl_noise_strength)); idx += 4
	
	general_data.encode_float(idx, wind_direction.x); idx += 4
	general_data.encode_float(idx, wind_direction.z); idx += 4
	general_data.encode_float(idx, fog_effect_ground); idx += 4
	general_data.encode_float(idx, positionQueries.size()); idx += 4
	
	general_data.encode_float(idx, float(point_lights_data.size()) / 2.0); idx += 4
	general_data.encode_float(idx, float(point_effector_data.size()) / 2.0); idx += 4
	general_data.encode_float(idx, wind_swept_range); idx += 4
	general_data.encode_float(idx, wind_swept_strength); idx += 4
	
	# Copy to byte buffer
	rd.buffer_update(general_data_buffer, 0, general_data.size(), general_data)


func update_lights():
	lights_updated = false
	
	if light_data.size() != 6272: #32 + 1024 + 512 * 4 bytes for each float = 6272.
		light_data.resize(6272)
	
	if (directional_lights_data.size() == 0): #defaults to having a default light.
		directional_lights_data.append(Vector4(0.5, 1.0, 0.5, 16.0))
		directional_lights_data.append(Vector4(1.0, 1.0, 1.0, 1.0))
	
	var idx = 0
	for i in range(min(directional_lights_data.size(), 8)):
		light_data.encode_float(idx, directional_lights_data[i].x)
		idx += 4
		light_data.encode_float(idx, directional_lights_data[i].y)
		idx += 4
		light_data.encode_float(idx, directional_lights_data[i].z)
		idx += 4
		light_data.encode_float(idx, directional_lights_data[i].w)
		idx += 4
	
	
	idx = 128
	for i in range(min(point_lights_data.size(), 256)):
		light_data.encode_float(idx, point_lights_data[i].x)
		idx += 4
		light_data.encode_float(idx, point_lights_data[i].y)
		idx += 4
		light_data.encode_float(idx, point_lights_data[i].z)
		idx += 4
		light_data.encode_float(idx, point_lights_data[i].w)
		idx += 4
	
	idx = 4224
	for i in range(min(point_effector_data.size(), 128)):
		light_data.encode_float(idx, point_effector_data[i].x)
		idx += 4
		light_data.encode_float(idx, point_effector_data[i].y)
		idx += 4
		light_data.encode_float(idx, point_effector_data[i].z)
		idx += 4
		light_data.encode_float(idx, point_effector_data[i].w)
		idx += 4
	
	rd.buffer_update(light_data_buffer, 0, light_data.size(), light_data)

func encode_sample_points():
	positionQuerying = true
	var sample_points_data_floats : PackedByteArray = []
	sample_points_data_floats.resize(512)
	
	var idx = 0
	while idx < 512 && positionQueries.size() > 0:
		
		sample_points_data_floats.encode_float(idx, positionQueries[0].x)
		idx += 4
		sample_points_data_floats.encode_float(idx, positionQueries[0].y)
		idx += 4
		sample_points_data_floats.encode_float(idx, positionQueries[0].z)
		idx += 4
		sample_points_data_floats.encode_float(idx, 0.0)
		idx += 4
		positionQueries.remove_at(0)
	
	
	rd.buffer_update(point_sample_data_buffer, 0, sample_points_data_floats.size(), sample_points_data_floats)
