#[compute]
#version 450
#define NUM_INVOCATIONS (1024)
layout(local_size_x = NUM_INVOCATIONS, local_size_y = 1, local_size_z = 1) in;

#extension GL_ARB_shading_language_include : enable
#include "common.glsli"

void main() {
	// bottom layer
	uint cells_per_invocation = uint(ceil(float(TREE_MAX_NODES_IN_LAYER_N[TREE_NUM_MAX_LAYERS-1]) / float(NUM_INVOCATIONS)));
	for (uint p = gl_LocalInvocationIndex * cells_per_invocation; p < ((gl_LocalInvocationIndex + 1) * cells_per_invocation); ++p) {
		if (p < TREE_MAX_NODES_IN_LAYER_N[TREE_NUM_MAX_LAYERS-1]) {
			TreeNodeData cur = tree_node_data_init();
			TreeNodeIdx_t idx = TREE_BUFFER_START_IDX_LAYER_N[TREE_NUM_MAX_LAYERS-1] + p;
			cur.parent_node_idx = TREE_BUFFER_START_IDX_LAYER_N[TREE_NUM_MAX_LAYERS-2] + p / TREE_NUM_CHILDREN_PER_NODE;
			cur.is_leaf_node = true;
			AABB3 aabb = tree_node_idx_to_aabb(idx, TREE_NUM_MAX_LAYERS-1);
			cur.data = 0.0;
			vec3 aabb_size = aabb.max - aabb.min;
			const vec3 num_samples_per_axis = vec3(2,1,2);
			for (uint z = 0; z < num_samples_per_axis.z; ++z) {
				for (uint y = 0; y < num_samples_per_axis.y; ++y)
					for (uint x = 0; x < num_samples_per_axis.x; ++x)
						cur.data += sample_scene(aabb.min + aabb_size * (vec3(x,y,z) + 0.5) / num_samples_per_axis);
			}
			cur.data = cur.data / (num_samples_per_axis.x * num_samples_per_axis.y * num_samples_per_axis.z);
			tree_buffer.nodes[idx] = cur;
		}
	}
	barrier();
	// middle layers
	for (uint i = TREE_NUM_MAX_LAYERS-2; i > 0; --i) {
		cells_per_invocation = uint(ceil(float(TREE_MAX_NODES_IN_LAYER_N[i]) / float(NUM_INVOCATIONS)));
		for (uint p = gl_LocalInvocationIndex * cells_per_invocation; p < ((gl_LocalInvocationIndex + 1) * cells_per_invocation); ++p) {
			if (p < TREE_MAX_NODES_IN_LAYER_N[i]) {
				TreeNodeData cur = tree_node_data_init();
				TreeNodeIdx_t idx = TREE_BUFFER_START_IDX_LAYER_N[i] + p;
				cur.parent_node_idx = TREE_BUFFER_START_IDX_LAYER_N[i-1] + p / TREE_NUM_CHILDREN_PER_NODE;
				cur.child_nodes_start_idx = TREE_BUFFER_START_IDX_LAYER_N[i+1] + p * TREE_NUM_CHILDREN_PER_NODE;

				// reference
				cur.data = 0.0;
				cur.is_leaf_node = true;
				for (uint j = 0; j < TREE_NUM_CHILDREN_PER_NODE; ++j) {
					float this_data = tree_buffer.nodes[cur.child_nodes_start_idx + j].data;
					cur.data += this_data;
				}
				cur.data /= TREE_NUM_CHILDREN_PER_NODE;
				for (uint j = 0; j < TREE_NUM_CHILDREN_PER_NODE; ++j) {
					float this_data = tree_buffer.nodes[cur.child_nodes_start_idx + j].data;
					cur.is_leaf_node = !(!cur.is_leaf_node || ((0.0001) < this_data));
					// cur.is_leaf_node = !(!cur.is_leaf_node || ((cur.data + 0.01) < this_data || this_data < (cur.data - 0.01)));
				}

				tree_buffer.nodes[idx] = cur;
			}
		}
		barrier();
	}
	barrier();
	// root layer
	if (gl_LocalInvocationIndex == 0) {
		TreeNodeData cur = tree_node_data_init();
		cur.child_nodes_start_idx = TREE_BUFFER_START_IDX_LAYER_1;
		tree_buffer.nodes[0] = cur;
	}
}