#[compute]
#version 450

layout(local_size_x = 8, local_size_y = 8, local_size_z = 1) in;

#extension GL_ARB_shading_language_include : enable
#include "common.glsli"

#define MAX_NUM_STEPS 1000

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

	float density = 0.0;

	uint cur_layer = 0;
	TreeNodeIdx_t cur_node_idxs[TREE_NUM_MAX_LAYERS];
	cur_node_idxs[0] = 0;
	// bitflag, one bit for each node. bit 0 == child =, bit 1 == child 1, etc;
	uint visited_child_nodes_of_cur_nodes[TREE_NUM_MAX_LAYERS];
	for (uint i = 0; i < (TREE_NUM_MAX_LAYERS); ++i) {
		visited_child_nodes_of_cur_nodes[i] = 0u;
	}

	RayAABBIntersection next_intersection = RayAABBIntersection(0.0,0.0);

	uint max_layer = 0;
	if (will_ray_exit_aabb(intersect_ray_with_aabb(ray_dir, ray_origin, tree_buffer.nodes[cur_node_idxs[0]].aabb)))
		for (int i = 0; i < MAX_NUM_STEPS; ++i) {
			if (!tree_buffer.nodes[cur_node_idxs[cur_layer]].is_leaf_node) {
				bool will_exit_child = false;
				TreeNodeIdx_t first_exiting_child_idx;
				for (uint j = 0; j < TREE_NUM_CHILDREN_PER_NODE; ++j) {
					bool already_visited = 0 < (visited_child_nodes_of_cur_nodes[cur_layer] & (1u << j));
					if (already_visited)
						continue;
					TreeNodeIdx_t this_idx = tree_buffer.nodes[cur_node_idxs[cur_layer]].child_nodes_start_idx + j;
					RayAABBIntersection this_intersection = intersect_ray_with_aabb(ray_dir, ray_origin, tree_buffer.nodes[this_idx].aabb);
					if (will_ray_exit_aabb(this_intersection) && ((!will_exit_child) || (next_intersection.exit_t > this_intersection.exit_t))) {
						will_exit_child = true;
						first_exiting_child_idx = this_idx;
						next_intersection = this_intersection;
						// break;
					}
				}
				if (will_exit_child) {
					cur_node_idxs[cur_layer + 1] = first_exiting_child_idx;
					visited_child_nodes_of_cur_nodes[cur_layer] |= (1u << (first_exiting_child_idx - tree_buffer.nodes[cur_node_idxs[cur_layer]].child_nodes_start_idx));
					cur_layer += 1;
					max_layer = uint(max(cur_layer, max_layer));
					// enter node
					continue;
				}
			}
				// leaf node
			else {
				if (cur_layer == (TREE_NUM_MAX_LAYERS - 1)) {
					float start_t = max(0.0, next_intersection.entry_t);
					float total_t = max(next_intersection.exit_t - start_t, 0.0);
					if (total_t > 0.0) {
							float data = 0.0;
							for (uint i = 0; i < 2; ++i) {
								data += sample_scene(ray_origin + ray_dir * (start_t + (total_t / 2.0) * (i + 0.5)));
							}
							data /= 2.0;
							density += data * pow(total_t, 1.0);
					}
				}
				if (density > 2.0) {
					density = 2.0;
					break;
				}
			}
			visited_child_nodes_of_cur_nodes[cur_layer] = 0u;
			// exit node
			if (cur_layer == 0)
				break;
			--cur_layer;
		}
	

	imageStore(output_color_image, iuv, vec4(vec3(density / 2.0), 1.0));
	// imageStore(output_color_image, iuv, vec4(vec2(density / 2.0), empty_density, 1.0));
	imageStore(output_color_image, iuv, vec4(vec2(density / 2.0), pow(float(max_layer)/float(TREE_NUM_MAX_LAYERS - 1), 2.0), 1.0));
}