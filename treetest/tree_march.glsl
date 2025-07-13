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

	vec3 global_ray_dir = normalize(rd_world);
	vec3 ray_dir = global_ray_dir;
	vec3 inv_ray_dir = 1.0 / ray_dir;
	// local to tree root
	vec3 ray_origin = genericData.view[3].xyz - TREE_ROOT_ORIGIN;
	vec3 ray_pos = ray_origin - TREE_ROOT_ORIGIN;

	float density = 0.0;

	uint cur_layer = 0;
	TreeNodeIdx_t cur_node_idxs[TREE_NUM_MAX_LAYERS];
	cur_node_idxs[0] = 0;

	RayAABBIntersection root_intersection = intersect_ray_with_aabb(ray_dir, ray_origin, AABB3(vec3(0.0), TREE_ROOT_SIZE));

	uint n_iters = 0;
	uint64_t clock_before_rt = clockRealtimeEXT();
	// uses the idea of a fractional tree and its float bit manipulation
	// see https://dubiousconst282.github.io/2024/10/03/voxel-ray-tracing/
	if (will_ray_exit_aabb(root_intersection)) {
		float largest_tree_root_axis = max(max(TREE_ROOT_SIZE.x, TREE_ROOT_SIZE.y), TREE_ROOT_SIZE.z);
		ray_pos = ray_origin + ray_dir * max(root_intersection.entry_t + 0.001, 0.0);
		ray_origin = clamp(ray_origin / TREE_ROOT_SIZE + 1.0, vec3(1.0), vec3(1.9999999));
		ray_pos    = clamp(ray_pos    / TREE_ROOT_SIZE + 1.0, vec3(1.0), vec3(1.9999999));
		ray_dir = normalize(ray_dir / (TREE_ROOT_SIZE / largest_tree_root_axis));
		inv_ray_dir = 1.0 / ray_dir;
		uint size_exp = 23u; // (mantissa bit width -> root)
		AABB3 tn_aabb = AABB3(vec3(1.0), vec3(1.9999999));
		for (int i = 0; i < MAX_NUM_STEPS; ++i) {
			n_iters++;

			// go down
			while (!tree_buffer.nodes[cur_node_idxs[cur_layer]].is_leaf_node) {
				size_exp -= 2u;
				uint closest_child_j = FPM_octree_closest_child(ray_pos, size_exp);
				TreeNodeIdx_t closest_child_idx = tree_buffer.nodes[cur_node_idxs[cur_layer]].child_nodes_start_idx + closest_child_j;
				cur_node_idxs[cur_layer + 1] = closest_child_idx;
				cur_layer += 1;
			}

			vec3 size = vec3(uintBitsToFloat((size_exp + 127u - 23u) << 23u));
			tn_aabb.min = FPM_floor_size(ray_pos, size_exp);
			tn_aabb.max = tn_aabb.min + size;
			RayAABBIntersectionExitOnly next_intersection = intersect_inv_ray_with_aabb_exit_only(inv_ray_dir, ray_pos, tn_aabb);

			// leaf node
			if (tree_buffer.nodes[cur_node_idxs[cur_layer]].is_leaf_node) {
				float data = tree_buffer.nodes[cur_node_idxs[cur_layer]].data;
				float total_t = length(next_intersection.exit_t * ray_dir * TREE_ROOT_SIZE);
				if (cur_layer == (TREE_NUM_MAX_LAYERS - 1)) {
					vec3 global_pos = (ray_pos - 1.0) * TREE_ROOT_SIZE + TREE_ROOT_ORIGIN;
					data = 0.0;
					const uint num_samples = 2;
					for (uint k = 0; k < num_samples; ++k) {
						data += sample_scene(global_pos + global_ray_dir * (total_t / num_samples * (k + 0.5)));
					}
					data = data / num_samples;
				}
				density += data * total_t;
				if (density > 2.0) {
					density = 2.0;
					break;
				}
			}

			// clamping
			vec3 neighbour_min = tn_aabb.min + uvec3(equal(next_intersection.exit_t.xxx, next_intersection.t_far_i)) * sign(ray_dir) * size;
			vec3 neighbour_max = intBitsToFloat(floatBitsToInt(neighbour_min) + ((1 << size_exp) - 1));
			ray_pos = clamp(ray_pos + ray_dir * next_intersection.exit_t, neighbour_min, neighbour_max);

			// need to find common ancestor of the current node and the next node.
			uvec3 diff_pos = floatBitsToUint(ray_pos) ^ floatBitsToUint(tn_aabb.min);
			int diff_exp = findMSB((diff_pos.x | diff_pos.y | diff_pos.z) & 0xFFAAAAAA) + 2;
			// go up
			if (diff_exp > size_exp) {
				size_exp = diff_exp;
				// exit root
				if (diff_exp > 23u) {
					break;
				}
				cur_layer = (23 - diff_exp) >> 1;
			}
		}
	}

	uint64_t clock_after_rt = clockRealtimeEXT();

	uint64_t clock_diff_rt = clock_after_rt - clock_before_rt;
	// imageStore(output_color_image, iuv, vec4(colormap_inferno(float(clock_diff_rt)/1000.0/1000.0), 1.0));

	imageStore(output_color_image, iuv, vec4(vec3(density / 2.0), 1.0));

	// imageStore(output_color_image, iuv, vec4(vec2(density / 2.0), pow(float(max_layer)/float(TREE_NUM_MAX_LAYERS - 1), 2.0), 1.0));
	// imageStore(output_color_image, iuv, vec4(vec2(density / 2.0), pow(float(n_iters)/float(MAX_NUM_STEPS), 1.0), 1.0));
	// imageStore(output_color_image, iuv, vec4(colormap_inferno(float(n_iters)/MAX_NUM_STEPS), 1.0));
}