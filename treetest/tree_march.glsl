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
	// vec3 ray_pos = ray_origin;
	float step_size = 100.0;
	// if (TREE_MAX_NODES_IN_LAYER_5 + TREE_BUFFER_START_IDX_LAYER_5 > TREE_MAX_NODES)
		// return;

	if (false) {
		TreeNodeIdx_t prev_within_node = 0xFFFFFFFF;
		for (int i = 0; i < MAX_NUM_STEPS; i++) {

			TreeNodeIdx_t root = 0;
			TreeNodeData root_node = tree_buffer.nodes[root];
			AABB3 root_aabb = tree_node_idx_to_aabb(0,0);
			RayAABBIntersection intersection = intersect_ray_with_aabb(ray_dir, ray_origin, root_aabb);
			bool will_enter = will_ray_enter_aabb(intersection);
			bool will_exit = will_ray_exit_aabb(intersection);
			// bool is_inside = (intersection.entry_t < 0.0) && (!isinf(intersection.entry_t));

			density += uint(will_enter) * 0.01;
			if ((will_enter || will_exit) && !root_node.is_leaf_node) {
				TreeNodeIdx_t closest_child;
				TreeNodeData closest_child_node;
				bool will_enter_child = false;
				float child_distance;
				for (uint j = 0; j < TREE_NUM_CHILDREN_PER_NODE; ++j) {
					TreeNodeIdx_t this_idx = root_node.child_nodes_start_idx + j;
					RayAABBIntersection this_intersection = intersect_ray_with_aabb(ray_dir, ray_origin, tree_node_idx_to_aabb(this_idx, 1));
					if (will_ray_enter_aabb(this_intersection) && ((!will_enter_child) || (will_enter_child && child_distance > this_intersection.entry_t))) {
						will_enter_child = true;
						child_distance = this_intersection.entry_t;
						closest_child = this_idx;
					}
				}
				if (will_enter_child) {
					closest_child_node = tree_buffer.nodes[closest_child];
					if (!closest_child_node.is_leaf_node) {
						density += uint(will_enter) * 0.01;
						TreeNodeIdx_t closest_grandchild;
						bool will_enter_grandchild = false;
						float grandchild_distance;
						for (uint j = 0; j < TREE_NUM_CHILDREN_PER_NODE; ++j) {
							TreeNodeIdx_t this_idx = closest_child_node.child_nodes_start_idx + j;
							RayAABBIntersection this_intersection = intersect_ray_with_aabb(ray_dir, ray_origin, tree_node_idx_to_aabb(this_idx, 2));
							if (will_ray_enter_aabb(this_intersection) && ((!will_enter_grandchild) || (will_enter_grandchild && grandchild_distance > this_intersection.entry_t))) {
								will_enter_grandchild = true;
								grandchild_distance = this_intersection.entry_t;
								closest_grandchild = this_idx;
							}
						}
						if (will_enter_grandchild) {
							density += tree_buffer.nodes[closest_grandchild].data * 0.01;
						}
					}
					else {
						density += closest_child_node.data * 0.1;
					}
				}
			}
			break;
			// ray_pos += ray_dir * step_size;
		}
	}


	density = 0.0;

	uint cur_layer = 0;
	TreeNodeIdx_t cur_node_idxs[TREE_NUM_MAX_LAYERS];
	cur_node_idxs[0] = 0;
	TreeNodeData cur_nodes[TREE_NUM_MAX_LAYERS];
	TreeNodeIdx_t visited_child_nodes_of_cur_nodes[TREE_NUM_MAX_LAYERS][TREE_NUM_CHILDREN_PER_NODE];
	uint num_visited_child_nodes_of_cur_nodes[TREE_NUM_MAX_LAYERS];
	for (uint i = 0; i < (TREE_NUM_MAX_LAYERS); ++i) {
		num_visited_child_nodes_of_cur_nodes[i] = 0u;
	}
	cur_nodes[0] = tree_buffer.nodes[0];

	uint max_layer = 0;
	// float empty_density = 0.0;
	if (will_ray_exit_aabb(intersect_ray_with_aabb(ray_dir, ray_origin, tree_node_idx_to_aabb(0, 0))))
		for (int i = 0; i < MAX_NUM_STEPS; ++i) {
			TreeNodeIdx_t first_exiting_child_idx;
			float first_exiting_child_t;
			bool will_exit_child = false;
			if (!cur_nodes[cur_layer].is_leaf_node) {
				for (uint j = 0; j < TREE_NUM_CHILDREN_PER_NODE; ++j) {
					TreeNodeIdx_t this_idx = cur_nodes[cur_layer].child_nodes_start_idx + j;
					RayAABBIntersection this_intersection = intersect_ray_with_aabb(ray_dir, ray_origin, tree_node_idx_to_aabb(this_idx, cur_layer + 1));
					bool already_visited = false;
					for (uint k = 0; k < num_visited_child_nodes_of_cur_nodes[cur_layer]; ++k) {
						already_visited = already_visited || (this_idx == visited_child_nodes_of_cur_nodes[cur_layer][k]);
					}
					if ((!already_visited) && will_ray_exit_aabb(this_intersection) && ((!will_exit_child) || (will_exit_child && first_exiting_child_t > this_intersection.exit_t))) {
						will_exit_child = true;
						first_exiting_child_t = this_intersection.exit_t;
						first_exiting_child_idx = this_idx;
					}
				}
			}
			if ((!cur_nodes[cur_layer].is_leaf_node) && will_exit_child) {
				TreeNodeData first_exiting_child_node = tree_buffer.nodes[first_exiting_child_idx];
				cur_nodes[cur_layer + 1] = first_exiting_child_node;
				cur_node_idxs[cur_layer + 1] = first_exiting_child_idx;
				visited_child_nodes_of_cur_nodes[cur_layer][num_visited_child_nodes_of_cur_nodes[cur_layer]] = first_exiting_child_idx;
				num_visited_child_nodes_of_cur_nodes[cur_layer] += 1;
				cur_layer += 1;
				max_layer = uint(max(cur_layer, max_layer));
				// enter node
			}
			else {
				// leaf node
				if (cur_nodes[cur_layer].is_leaf_node) {
					if (cur_layer == (TREE_NUM_MAX_LAYERS - 1)) {
						RayAABBIntersection intersection = intersect_ray_with_aabb(ray_dir, ray_origin, tree_node_idx_to_aabb(cur_node_idxs[cur_layer], cur_layer));
						float start_t = max(0.0, intersection.entry_t);
						float end_t = intersection.exit_t;
						float total_t = max(end_t - start_t, 0.0);
						if (total_t > 0.0) {
								float data = 0.0;
								for (uint i = 0; i < 2; ++i) {
									data += sample_scene(ray_origin + ray_dir * (start_t + (total_t / 2.0) * (i + 0.5)));
								}
								data /= 2.0;
								density += data * pow(total_t, 1.0);
						}
							// empty_density += 0.05 * total_t / pow(4.0, (TREE_NUM_MAX_LAYERS - 1.0 - cur_layer));
					}
					if (density > 2.0) {
						density = 2.0;
						break;
					}
				}
				// if (cur_layer < (TREE_NUM_MAX_LAYERS - 1))
				num_visited_child_nodes_of_cur_nodes[cur_layer] = 0;
				// exit node
				// density += 0.01;
				if (cur_layer == 0)
					break;
				--cur_layer;
			}
		}
	

	imageStore(output_color_image, iuv, vec4(vec3(density / 2.0), 1.0));
	// imageStore(output_color_image, iuv, vec4(vec2(density / 2.0), empty_density, 1.0));
	imageStore(output_color_image, iuv, vec4(vec2(density / 2.0), pow(float(max_layer)/float(TREE_NUM_MAX_LAYERS - 1), 2.0), 1.0));
}