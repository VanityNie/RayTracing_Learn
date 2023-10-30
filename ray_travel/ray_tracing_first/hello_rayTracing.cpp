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
#include "nvvk/raytraceNV_vk.hpp"
#include "host_device.h"

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


struct ObjInstance
{
    nvmath::mat4f transform;    // Matrix of the instance
    uint32_t      objIndex{0};  // Model index reference
};

class HelloRayTracing
{

private:
    VkDevice m_device;
    VkPhysicalDevice m_physicalDevice;
    VkPhysicalDeviceProperties2 m_rtProperties;
    nvvk::RaytracingBuilderKHR m_rtBuilder;
    nvvk::ResourceAllocator m_alloc;
    uint32_t m_graphicsQueueIndex;


    //Array of objects and instances
    std::vector<ObjModel> m_objModel;
    std::vector<ObjInstance>m_instances;



    //Descriptor set
    nvvk::DescriptorSetBindings m_rtDescSetLayoutBind;
    VkDescriptorPool  m_rtDescPool;
    VkDescriptorSetLayout m_rtDescSetLayout;
    VkDescriptorSet m_rtDescSet;

    void createRtDescriptorSet();


    nvvk::Texture  m_offscreenColor;


public:


    void initRayTracing();

    auto objectToVkGemortyKHR(const ObjModel& model);
    // create bottom Acceleration Structure
    void createBottomLevelAS();
    void createTopLevelAS();


};


void HelloRayTracing::initRayTracing() {
    VkPhysicalDeviceProperties2 prop2{VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_PROPERTIES_2};
    prop2.pNext = &m_rtProperties;
    vkGetPhysicalDeviceProperties2(m_physicalDevice, &prop2);
    m_rtBuilder.setup(m_device, &m_alloc, m_graphicsQueueIndex);
}



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



/**
 * @brief create ray tracing descriptor set to hold the Acceleration structure and the output image
 */

void HelloRayTracing::createRtDescriptorSet() {

    m_rtDescSetLayoutBind.addBinding(RtxBindings::eTlas, VK_DESCRIPTOR_TYPE_ACCELERATION_STRUCTURE_KHR, 1,
                                     VK_SHADER_STAGE_RAYGEN_BIT_KHR | VK_SHADER_STAGE_CLOSEST_HIT_BIT_KHR);  // TLAS
    m_rtDescSetLayoutBind.addBinding(RtxBindings::eOutImage, VK_DESCRIPTOR_TYPE_STORAGE_IMAGE, 1,
                                     VK_SHADER_STAGE_RAYGEN_BIT_KHR);  // Output image

    m_rtDescPool      = m_rtDescSetLayoutBind.createPool(m_device);
    m_rtDescSetLayout = m_rtDescSetLayoutBind.createLayout(m_device);

    VkDescriptorSetAllocateInfo allocateInfo{VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO};
    allocateInfo.descriptorPool     = m_rtDescPool;
    allocateInfo.descriptorSetCount = 1;
    allocateInfo.pSetLayouts        = &m_rtDescSetLayout;
    vkAllocateDescriptorSets(m_device, &allocateInfo, &m_rtDescSet);


    VkAccelerationStructureKHR  tlas = m_rtBuilder.getAccelerationStructure();
    VkWriteDescriptorSetAccelerationStructureKHR descASInfo{ VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET_ACCELERATION_STRUCTURE_KHR};
    descASInfo.accelerationStructureCount = 1;
    descASInfo.pAccelerationStructures    = &tlas;
    VkDescriptorImageInfo imageInfo{{}, m_offscreenColor.descriptor.imageView, VK_IMAGE_LAYOUT_GENERAL};


    std::vector<VkWriteDescriptorSet> writes;
    writes.emplace_back(m_rtDescSetLayoutBind.makeWrite(m_rtDescSet, RtxBindings::eTlas, &descASInfo));
    writes.emplace_back(m_rtDescSetLayoutBind.makeWrite(m_rtDescSet, RtxBindings::eOutImage, &imageInfo));
    vkUpdateDescriptorSets(m_device, static_cast<uint32_t>(writes.size()), writes.data(), 0, nullptr);

}


void HelloRayTracing::createTopLevelAS()
{
    std::vector<VkAccelerationStructureInstanceKHR> tlas;
    tlas.resize(m_instances.size());

    for(const auto & inst : m_instances)
    {
        VkAccelerationStructureInstanceKHR rayInst{};
        rayInst.transform = nvvk::toTransformMatrixKHR(inst.transform);  // Position of the instance
        rayInst.instanceCustomIndex = inst.objIndex;
        rayInst.accelerationStructureReference = m_rtBuilder.getBlasDeviceAddress(inst.objIndex);
        rayInst.flags                          = VK_GEOMETRY_INSTANCE_TRIANGLE_FACING_CULL_DISABLE_BIT_KHR;
        rayInst.mask                           = 0xFF;       //  Only be hit if rayMask & instance.mask != 0
        rayInst.instanceShaderBindingTableRecordOffset = 0;  // We will use the same hit group for all objects
        tlas.emplace_back(rayInst);
        m_rtBuilder.buildTlas(tlas, VK_BUILD_ACCELERATION_STRUCTURE_PREFER_FAST_TRACE_BIT_KHR);
    }
}

#endif //VK_RAYTRACING_TUTORIAL_HELLO_RAYTRACING_CPP
