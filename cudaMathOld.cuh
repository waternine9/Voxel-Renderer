#include "cuda_runtime.h"

#include "device_launch_parameters.h"

#include <iostream>
#include <fstream>
#include <sstream>
#include <stdio.h>
#include <string>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <math.h>
__device__ float dot(float3 v1, float3 v2)

{
    return v1.x * v2.x + v1.y * v2.y + v1.z * v2.z;
}
__device__ float3 cross(float3 v1, float3 v2)
{
    float3 rez = { 0, 0, 0 };
    rez.x = v1.y * v2.z - v1.z * v2.y;
    rez.y = v1.z * v2.x - v1.x * v2.z;
    rez.z = v1.x * v2.y - v1.y * v2.x;
    return rez;
}
__device__ float3 sub(float3 v1, float3 v2)
{
    return { v1.x - v2.x, v1.y - v2.y, v1.z - v2.z };
}
__device__ float3 add(float3 v1, float3 v2)
{
    return { v1.x + v2.x, v1.y + v2.y, v1.z + v2.z };
}
__device__ float length(float3 v)
{
    return (sqrtf(v.x * v.x + v.y * v.y + v.z * v.z));
}
__device__ float3 normalize(float3 v)
{
    float3 rez = { 0, 0, 0 };
    float len = length(v);
    rez.x = v.x / len;
    rez.y = v.y / len;
    rez.z = v.z / len;

    return rez;
}