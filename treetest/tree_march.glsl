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
	vec3 ray_origin = genericData.view[3].xyz;
	vec3 ray_pos = ray_origin;

	float density = 0.0;

	uint cur_layer = 0;
	TreeNodeIdx_t cur_node_idxs[TREE_NUM_MAX_LAYERS];
	cur_node_idxs[0] = 0;

	RayAABBIntersection next_intersection = intersect_ray_with_aabb(ray_dir, ray_origin, tree_buffer.nodes[cur_node_idxs[0]].aabb);

	uint max_layer = 0;
	uint n_iters = 0;
	uint64_t clock_before_rt = clockRealtimeEXT();
	if (will_ray_exit_aabb(next_intersection)) {
		ray_pos = ray_origin + ray_dir * max(next_intersection.entry_t + 0.001, 0.0);
		for (int i = 0; i < MAX_NUM_STEPS; ++i) {
			n_iters++;
			if (!tree_buffer.nodes[cur_node_idxs[cur_layer]].is_leaf_node) {
				AABB3 tn_aabb = tree_node_aabb_to_aabb(tree_buffer.nodes[cur_node_idxs[cur_layer]].aabb);

				if (is_pos_inside_aabb(ray_pos, tn_aabb)) {
					uint closest_child_j = which_is_the_closest_child_of_tree_node_to_pos(ray_pos, tn_aabb);
					TreeNodeIdx_t closest_child_idx = tree_buffer.nodes[cur_node_idxs[cur_layer]].child_nodes_start_idx + closest_child_j;
					cur_node_idxs[cur_layer + 1] = closest_child_idx;
					cur_layer += 1;
					max_layer = uint(max(cur_layer, max_layer));
					// enter node
					continue;
				}
			}
			// leaf node or no child collided (exiting node)
			next_intersection = intersect_ray_with_aabb(ray_dir, ray_origin, tree_buffer.nodes[cur_node_idxs[cur_layer]].aabb);

			// leaf node
			if (tree_buffer.nodes[cur_node_idxs[cur_layer]].is_leaf_node) {
				if (cur_layer == (TREE_NUM_MAX_LAYERS - 1)) {
					float start_t = max(0.0, next_intersection.entry_t);
					float total_t = max(next_intersection.exit_t - start_t, 0.0);
					// if (total_t > 0.0) {
					float data = 0.0;
					const uint num_samples = 2;
					for (uint i = 0; i < num_samples; ++i) {
						data += sample_scene(ray_origin + ray_dir * (start_t + (total_t / float(num_samples)) * (i + 0.5)));
					}
					data /= float(num_samples);
					density += data * pow(total_t, 1.0);
					// }
					if (density > 2.0) {
						density = 2.0;
						break;
					}
				}
			}
			// exit node
			ray_pos = ray_origin + ray_dir * (next_intersection.exit_t + 0.001);

			if (cur_layer == 0)
				break;
			--cur_layer;
		}
		// n_iters--;
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