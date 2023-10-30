//
// Created by win111 on 2023/10/30.
//

#ifndef VK_RAYTRACING_TUTORIAL_HOST_DEVICE_H
#define VK_RAYTRACING_TUTORIAL_HOST_DEVICE_H


#ifdef __cplusplus
#include "nvmath/nvmath.h"

using vec2 = nvmath::vec2f;
using vec3 = nvmath::vec3f;
using vec4 = nvmath::vec4f;
using mat4 = nvmath::mat4f;
using uint = unsigned int;
#endif


#ifdef __cplusplus // Descriptor binding helper for C++ and GLSL
#define START_BINDING(a) enum a {
#define END_BINDING() }
#else
#define START_BINDING(a)  const uint
 #define END_BINDING()
#endif

START_BINDING(SceneBindings)
    eGlobals  = 0,  // Global uniform containing camera matrices
    eObjDescs = 1,  // Access to the object descriptions
    eTextures = 2   // Access to textures
END_BINDING();

START_BINDING(RtxBindings)
    eTlas     = 0,  // Top-level acceleration structure
    eOutImage = 1   // Ray tracer output image
END_BINDING();

#endif //VK_RAYTRACING_TUTORIAL_HOST_DEVICE_H
