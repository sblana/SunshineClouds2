#[compute]
#version 450

layout(local_size_x = 8, local_size_y = 8, local_size_z = 1) in;

#extension GL_ARB_shading_language_include : enable
// dependency for GL_EXT_shader_realtime_clock
// see https://registry.khronos.org/OpenGL/extensions/ARB/ARB_gpu_shader_int64.txt
#extension GL_ARB_gpu_shader_int64 : enable
// see https://github.com/KhronosGroup/GLSL/blob/main/extensions/ext/GL_EXT_shader_realtime_clock.txt
#extension GL_EXT_shader_realtime_clock : enable

#include "common.glsli"
#include "colormaps.glsli"

#define MAX_NUM_STEPS 1024

void main() {
		//SETTING UP UVS/RAY DATA
	ivec2 iuv = ivec2(gl_GlobalInvocationID.xy);
	ivec2 size = ivec2(params.raster_size);

	// Prevent reading/writing out of bounds.
	if (iuv.x >= size.x || iuv.y >= size.y) {
		return;
	}

	vec2 screen_uv = vec2(iuv) / vec2(size);

	// Convert screen coordinates to normalized device coordinates
	vec2 ndc = screen_uv * 2.0 - 1.0;
	// Convert NDC to view space coordinates
	vec4 clipPos = vec4(ndc, 0.0, 1.0);
	vec4 viewPos = inverse(genericData.proj) * clipPos;
	viewPos.xyz /= viewPos.w;

	vec3 rd_world = normalize(viewPos.xyz);
	rd_world = mat3(genericData.view) * rd_world;
	// Define the ray properties

	vec3 ray_dir = normalize(rd_world);
	vec3 inv_ray_dir = 1.0 / ray_dir;
	// local to tree root
	vec3 ray_origin = genericData.view[3].xyz - TREE_ROOT_ORIGIN;
	vec3 ray_pos = ray_origin - TREE_ROOT_ORIGIN;

	float density = 0.0;

	uint cur_layer = 0;
	TreeNodeIdx_t cur_node_idxs[TREE_NUM_MAX_LAYERS];
	cur_node_idxs[0] = 0;

	RayAABBIntersection next_intersection = intersect_ray_with_aabb(ray_dir, ray_origin, AABB3(vec3(0.0), TREE_ROOT_SIZE));

	uint max_layer = 0;
	uint n_iters = 0;
	uint64_t clock_before_rt = clockRealtimeEXT();
	if (will_ray_exit_aabb(next_intersection)) {
		ray_pos = ray_origin + ray_dir * max(next_intersection.entry_t + 0.001, 0.0);
		vec3 size = tree_layer_cell_size(cur_layer);
		for (int i = 0; i < MAX_NUM_STEPS; ++i) {
			n_iters++;
			AABB3 tn_aabb = AABB3(vec3(floor(ray_pos / size) * size), vec3(0.0));
			tn_aabb.max = tn_aabb.min + size;
			// go down
			while (!tree_buffer.nodes[cur_node_idxs[cur_layer]].is_leaf_node) {
				uint closest_child_j = which_is_the_closest_child_of_tree_node_to_pos(ray_pos, tn_aabb.min, size);
				TreeNodeIdx_t closest_child_idx = tree_buffer.nodes[cur_node_idxs[cur_layer]].child_nodes_start_idx + closest_child_j;
				size /= vec3(TREE_NUM_DIVISIONS_PER_NODE);
				tn_aabb.min = floor(ray_pos / size) * size;
				tn_aabb.max = tn_aabb.min + size;
				cur_node_idxs[cur_layer + 1] = closest_child_idx;
				cur_layer += 1;
				// enter node
			}
			max_layer = uint(max(cur_layer, max_layer));

			next_intersection = intersect_inv_ray_with_aabb(inv_ray_dir, ray_pos, tn_aabb);

			// leaf node
			if (tree_buffer.nodes[cur_node_idxs[cur_layer]].is_leaf_node) {
				float data = tree_buffer.nodes[cur_node_idxs[cur_layer]].data;
				float total_t = next_intersection.exit_t;
				if (cur_layer == (TREE_NUM_MAX_LAYERS - 1)) {
					data = 0.0;
					const uint num_samples = 2;
					for (uint i = 0; i < num_samples; ++i) {
						data += sample_scene(ray_pos + ray_dir * (total_t / float(num_samples) * (i + 0.5)));
					}
					data /= float(num_samples);
				}
				density += data * total_t;
				if (density > 2.0) {
					density = 2.0;
					break;
				}
			}

			// clamping. we need to be 1 bit lower than the maximum value. sucks to do unless we're already doing tons of bit manipulation.
			vec3 next_min = tn_aabb.min + size * sign(ray_dir) * vec3(equal(next_intersection.exit_t.xxx, next_intersection.t_far_i));
			vec3 next_max = tn_aabb.max + size * sign(ray_dir) * vec3(equal(next_intersection.exit_t.xxx, next_intersection.t_far_i));
			ray_pos = clamp(ray_pos + ray_dir * (next_intersection.exit_t), next_min, next_max - vec3(0.0001) * (abs(next_max) + 1.0));

			// go up
			// need to find common ancestor of the current node and the next node.
			// we know where the two nodes are so we can infer based on their bounding boxes
			int j = int(cur_layer);
			for (; j >= 0; --j) {
				if (floor(ray_pos / size) == floor(tn_aabb.min / size)) {
					break;
				}
				size *= vec3(TREE_NUM_DIVISIONS_PER_NODE);
			}

			// exit node(s)
			if (j < 0)
				break;
			cur_layer = j;
		}
	}

	uint64_t clock_after_rt = clockRealtimeEXT();

	uint64_t clock_diff_rt = clock_after_rt - clock_before_rt;
	// imageStore(output_color_image, iuv, vec4(colormap_inferno(float(clock_diff_rt)/1000.0/1000.0), 1.0));

	imageStore(output_color_image, iuv, vec4(vec3(density / 2.0), 1.0));

	// imageStore(output_color_image, iuv, vec4(vec2(density / 2.0), pow(float(max_layer)/float(TREE_NUM_MAX_LAYERS - 1), 2.0), 1.0));
	// imageStore(output_color_image, iuv, vec4(vec2(density / 2.0), pow(float(n_iters)/float(MAX_NUM_STEPS), 1.0), 1.0));
	// imageStore(output_color_image, iuv, vec4(colormap_inferno(float(n_iters)/MAX_NUM_STEPS), 1.0));

	// imageStore(output_color_image, iuv, vec4(vec3(local_xyz), 1.0));
}