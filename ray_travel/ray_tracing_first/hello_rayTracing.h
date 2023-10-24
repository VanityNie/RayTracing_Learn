//
// Created by win111 on 2023/10/24.
//

#ifndef VK_RAYTRACING_TUTORIAL_HELLO_RAYTRACING_H
#define VK_RAYTRACING_TUTORIAL_HELLO_RAYTRACING_H


#include <sstream>


#define STB_IMAGE_IMPLEMENTATION

#include "nvvkhl/appbase_vk.hpp"
#include "nvvk/debug_util_vk.hpp"
#include "nvvk/descriptorsets_vk.hpp"
#include "nvvk/memallocator_dma_vk.hpp"
#include "nvvk/resourceallocator_vk.hpp"



// The OBJ model
struct ObjModel
{
    uint32_t     nbIndices{0};
    uint32_t     nbVertices{0};
    nvvk::Buffer vertexBuffer;    // Device buffer of all 'Vertex'
    nvvk::Buffer indexBuffer;     // Device buffer of the indices forming triangles
    nvvk::Buffer matColorBuffer;  // Device buffer of array of 'Wavefront material'
    nvvk::Buffer matIndexBuffer;  // Device buffer of array of 'Wavefront material'
};




#endif //VK_RAYTRACING_TUTORIAL_HELLO_RAYTRACING_H
