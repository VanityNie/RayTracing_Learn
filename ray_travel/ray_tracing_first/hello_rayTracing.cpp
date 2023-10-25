//
// Created by win111 on 2023/10/24.
//

#ifndef VK_RAYTRACING_TUTORIAL_HELLO_RAYTRACING_CPP
#define VK_RAYTRACING_TUTORIAL_HELLO_RAYTRACING_CPP


#include <sstream>
#include <nvvk/raytraceKHR_vk.hpp>
#include <nvvk/buffers_vk.hpp>


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



class HelloRayTracing
{

private:
    VkDevice m_device;

    nvvk::RaytracingBuilderKHR m_rtBuilder;
    std::vector<ObjModel> m_objModel;
public:


    auto objectToVkGemortyKHR(const ObjModel& model);
    // create bottom Acceleration Structure
    void createBottomLevelAS();

};

/**
 *
 * @brief read obj Model and translate to vk Gemorty info to build the BLAS
 * @param ObjModel
 * @return nvvk::RaytracingBuilderKHR::BlasInput
 */

auto HelloRayTracing::objectToVkGemortyKHR(const ObjModel &model) {

    // BLAS builder requires raw device addresses.

    VkDeviceAddress vertexAddress = nvvk::getBufferDeviceAddress(m_device, model.vertexBuffer.buffer);
    VkDeviceAddress indexAddress = nvvk::getBufferDeviceAddress(m_device,model.indexBuffer.buffer);

    uint32_t maxPrimitiveCount = model.nbIndices / 3;

    // Describe buffer as array of VertexObj.
    VkAccelerationStructureGeometryTrianglesDataKHR triangles{
        VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_GEOMETRY_TRIANGLES_DATA_KHR
    };

    triangles.vertexFormat = VK_FORMAT_R32G32B32_SFLOAT;    //vec3 vertex postion data
    triangles.vertexData.deviceAddress = vertexAddress;

    // Describe index data (32-bit unsigned int)
    triangles.indexType = VK_INDEX_TYPE_UINT32;
    triangles.indexData.deviceAddress = indexAddress;

    // now the tranform data is empty
    //triangles.transformData = {};
    triangles.maxVertex = model.nbVertices - 1;

    // Identify the above data as containing opaque triangles.

    VkAccelerationStructureGeometryKHR asGeom{VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_GEOMETRY_KHR};

    asGeom.geometryType = VK_GEOMETRY_TYPE_TRIANGLES_KHR;
    asGeom.flags = VK_GEOMETRY_OPAQUE_BIT_KHR;
    asGeom.geometry.triangles = triangles;


    VkAccelerationStructureBuildRangeInfoKHR offset;
    offset.firstVertex     = 0;
    offset.primitiveCount  = maxPrimitiveCount;
    offset.primitiveOffset = 0;
    offset.transformOffset = 0;

    nvvk::RaytracingBuilderKHR::BlasInput input;

    //this blas only has one geometry
    input.asGeometry.emplace_back(asGeom);
    input.asBuildOffsetInfo.emplace_back(offset);

    return input;
}

/**
 *  @brief use fast trace bit optimize ray traing
 *
 */
void HelloRayTracing::createBottomLevelAS()
{

    std::vector<nvvk::RaytracingBuilderKHR::BlasInput> allBlas;

    allBlas.resize(m_objModel.size());

    for(const auto & obj: m_objModel)
    {
        auto  blas = objectToVkGemortyKHR(obj);

        allBlas.emplace_back(blas);
    }

    //VK_BUILD_ACCELERATION_STRUCTURE_PREFER_FAST_TRACE_BIT_KHR optimize ray tracing
    m_rtBuilder.buildBlas(allBlas,VK_BUILD_ACCELERATION_STRUCTURE_PREFER_FAST_TRACE_BIT_KHR);
}


#endif //VK_RAYTRACING_TUTORIAL_HELLO_RAYTRACING_CPP
