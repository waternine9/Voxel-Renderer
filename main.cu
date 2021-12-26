#include <iostream>	
#include "opencv2/opencv.hpp"

#include "glm/glm.hpp"

#include "glm/ext.hpp"
#include <chrono>
#include <fstream>
#include <strstream>
#include <math.h>
#include <Windows.h>
#include "cuda_runtime.h"
#include "device_launch_parameters.h"
#include "cudamathOld.cuh"
// #include <Windows.h>
#define NUMTRIS 10000
#define RESOLUTION 512
using namespace cv;
using namespace std;
struct Wrapper
{
public:
	int* r;
	int* g;
	int* b;
};
__global__ void getColAtLoc(int y, float* triangles, int* outputR, int* outputG, int* outputB, float* mousePos, float degreesXZ, float degreesYZ)
{
	float i = threadIdx.x;
	float j = blockIdx.x;
	int colR = 255;
	int colG = 255;
	int colB = 255;
	float3 rayVec = { (i - (RESOLUTION * 0.5)) * 0.004, ((j + y * (RESOLUTION * 0.25)) - (RESOLUTION * 0.5)) * 0.004, 1 };
	float3 rayPos = { (mousePos[0] / RESOLUTION) * 4 - 2, (mousePos[1] / RESOLUTION) * 4 - 2, mousePos[2] };
	float3 rayOrig = rayPos;
	float3 rayOrigVec = rayVec;
	float SINF = sinf(degreesXZ);
	float COSF = cosf(degreesXZ);
	float output2[1][3] = { { 0, 0, 0 } };
	float input1[1][3];
	float input2[3][3];
	input2[0][0] = 1;
	input2[0][1] = 0;
	input2[0][2] = 0;
	input2[1][0] = 0;
	input2[2][0] = 0;
	input2[1][1] = COSF;
	input2[1][2] = -SINF;
	input2[2][1] = SINF;
	input2[2][2] = COSF;
	input1[0][0] = rayVec.x;
	input1[0][1] = rayVec.y;
	input1[0][2] = rayVec.z;
	for (int _ = 0;_ < 1;_++)
		for (int Y = 0;Y < 3;Y++)
			for (int k = 0;k < 3;k++)
			{
				output2[_][Y] += input1[_][k] * input2[k][Y];
			}
	rayVec = { (float)output2[0][0], (float)output2[0][1], (float)output2[0][2] };
	SINF = sinf(degreesYZ);
	COSF = cosf(degreesYZ);
	float output22[1][3] = { { 0, 0, 0 } };
	input2[0][0] = COSF;
	input2[0][1] = 0;
	input2[0][2] = SINF;
	input2[1][0] = 0;
	input2[2][0] = -SINF;
	input2[1][1] = 1;
	input2[1][2] = 0;
	input2[2][1] = 0;
	input2[2][2] = COSF;
	input1[0][0] = rayVec.x;
	input1[0][1] = rayVec.y;
	input1[0][2] = rayVec.z;
	for (int _ = 0;_ < 1;_++)
		for (int Y = 0;Y < 3;Y++)
			for (int k = 0;k < 3;k++)
			{
				output22[_][Y] += input1[_][k] * input2[k][Y];
			}
	rayVec = { (float)output22[0][0], (float)output22[0][1], (float)output22[0][2] };
	rayVec = normalize(rayVec);
	float totalDist = 1000;
	float t;
	float3 curCenter;
	for (int x = 0;x < NUMTRIS;x++)
	{
		float3 center = { triangles[x * 3], triangles[x * 3 + 1], triangles[x * 3 + 2] };
		if (center.x == 0.0f) break;
		else 
		{
			float3 dirfrac;
			dirfrac.x = 1.0f / rayVec.x;
			dirfrac.y = 1.0f / rayVec.y;
			dirfrac.z = 1.0f / rayVec.z;
			// lb is the corner of AABB with minimal coordinates - left bottom, rt is maximal corner
			// rayPos is origin of ray
			float3 lb;
			float3 rt;
			
			lb.x = center.x - 0.5f;
			lb.y = center.y - 0.5f;
			lb.z = center.z - 0.5f;
			rt.x = center.x + 0.5f;
			rt.y = center.y + 0.5f;
			rt.z = center.z + 0.5f;
			
			float t1 = (lb.x - rayPos.x) * dirfrac.x;
			float t2 = (rt.x - rayPos.x) * dirfrac.x;
			float t3 = (lb.y - rayPos.y) * dirfrac.y;
			float t4 = (rt.y - rayPos.y) * dirfrac.y;
			float t5 = (lb.z - rayPos.z) * dirfrac.z;
			float t6 = (rt.z - rayPos.z) * dirfrac.z;
			
			float tmin = max(max(min(t1, t2), min(t3, t4)), min(t5, t6));
			float tmax = min(min(max(t1, t2), max(t3, t4)), max(t5, t6));
			
			// if tmax < 0, ray (line) is intersecting AABB, but the whole AABB is behind us
			if (tmax >= 0)
			{
				// if tmin > tmax, ray doesn't intersect AABB
				if (tmin <= tmax)
				{
					t = tmin;
					if (t < totalDist)
					{
						// col = abs(add(rayPos, { rayVec.x * t, rayVec.y * t, rayVec.z * t }).y * 10);// fmaxf(dot(normalize(sub({ mousePos[0], mousePos[1], mousePos[2] }, add(rayPos, { rayVec.x * t, rayVec.y * t, rayVec.z * t }))), normalize(sub(add(rayPos, { rayVec.x * t, rayVec.y * t, rayVec.z * t }), center))), 0) * 255;
						totalDist = t;
						curCenter = center;
						colR = add(rayPos, { rayVec.x * t, rayVec.y * t, rayVec.z * t }).x;
						colG = sin(add(rayPos, { rayVec.x * t, rayVec.y * t, rayVec.z * t }).y / 100) * 255;
						colB = cos(add(rayPos, { rayVec.x * t, rayVec.y * t, rayVec.z * t }).z / 100) * 255;
					}
				}
			}
		}
	}
	/* -------------------------------
		This is the reflection code. I commented it out because it looks bad.
	------------------------------- */
	/* if (totalDist < 1000)
	{
		totalDist = 1000;
		float3 norm = normalize(sub(add(rayPos, { rayVec.x * t, rayVec.y * t, rayVec.z * t }), curCenter));
		float DOTPROD = dot(rayVec, norm);
		rayVec.x = rayVec.x - 2 * DOTPROD * norm.x;
		rayVec.y = rayVec.y - 2 * DOTPROD * norm.y;
		rayVec.z = rayVec.z - 2 * DOTPROD * norm.z;
		rayPos = add(add(rayPos, { rayVec.x * t, rayVec.y * t, rayVec.z * t }), rayVec);
		for (int x = 0;x < NUMTRIS;x++)
		{
			float3 center = { triangles[x * 3], triangles[x * 3 + 1], triangles[x * 3 + 2] };
			if (center.x == 0.0f) break;
			else
			{
				float3 dirfrac;
				dirfrac.x = 1.0f / rayVec.x;
				dirfrac.y = 1.0f / rayVec.y;
				dirfrac.z = 1.0f / rayVec.z;
				// lb is the corner of AABB with minimal coordinates - left bottom, rt is maximal corner
				// rayPos is origin of ray
				float3 lb;
				float3 rt;
				lb.x = center.x - 0.5f;
				lb.y = center.y - 0.5f;
				lb.z = center.z - 0.5f;
				rt.x = center.x + 0.5f;
				rt.y = center.y + 0.5f;
				rt.z = center.z + 0.5f;
				float t1 = (lb.x - rayPos.x) * dirfrac.x;
				float t2 = (rt.x - rayPos.x) * dirfrac.x;
				float t3 = (lb.y - rayPos.y) * dirfrac.y;
				float t4 = (rt.y - rayPos.y) * dirfrac.y;
				float t5 = (lb.z - rayPos.z) * dirfrac.z;
				float t6 = (rt.z - rayPos.z) * dirfrac.z;

				float tmin = max(max(min(t1, t2), min(t3, t4)), min(t5, t6));
				float tmax = min(min(max(t1, t2), max(t3, t4)), max(t5, t6));
				float t;
				// if tmax < 0, ray (line) is intersecting AABB, but the whole AABB is behind us
				if (tmax >= 0)
				{
					// if tmin > tmax, ray doesn't intersect AABB
					if (tmin <= tmax)
					{
						t = tmin;
						if (t < totalDist)
						{
							colR = add(rayPos, { rayVec.x * t, rayVec.y * t, rayVec.z * t }).x; // fmaxf(dot(normalize(sub({ mousePos[0], mousePos[1], mousePos[2] }, add(rayPos, { rayVec.x * t, rayVec.y * t, rayVec.z * t }))), normalize(sub(add(rayPos, { rayVec.x * t, rayVec.y * t, rayVec.z * t }), center))), 0) * 255;
							colG = sin(add(rayPos, { rayVec.x * t, rayVec.y * t, rayVec.z * t }).y / 100) * 255;
							colB = cos(add(rayPos, { rayVec.x * t, rayVec.y * t, rayVec.z * t }).z / 100) * 255;
							totalDist = t;
						}
					}
				}
			}
		}
	} */
	outputR[((int)i + (int)j * RESOLUTION)] = colR;
	outputG[((int)i + (int)j * RESOLUTION)] = colG;
	outputB[((int)i + (int)j * RESOLUTION)] = colB;
}

Wrapper helper(int y, float* triangles, float* mousePos, float degreesXZ, float degreesYZ)
{
	float* dev_triangles = nullptr;
	cudaMalloc(&dev_triangles, NUMTRIS * 3 * sizeof(float));
	cudaMemcpy(dev_triangles, triangles, NUMTRIS * 3 * sizeof(float), cudaMemcpyHostToDevice);
	int* dev_outputR = nullptr;
	int outputR[(RESOLUTION * (RESOLUTION / 4))] = { 255 };
	cudaMalloc(&dev_outputR, (RESOLUTION * (RESOLUTION / 4)) * sizeof(int));
	cudaMemcpy(dev_outputR, outputR, (RESOLUTION * (RESOLUTION / 4)) * sizeof(int), cudaMemcpyHostToDevice);
	int* dev_outputG = nullptr;
	int outputG[(RESOLUTION * (RESOLUTION / 4))] = { 255 };
	cudaMalloc(&dev_outputG, (RESOLUTION * (RESOLUTION / 4)) * sizeof(int));
	cudaMemcpy(dev_outputG, outputG, (RESOLUTION * (RESOLUTION / 4)) * sizeof(int), cudaMemcpyHostToDevice);
	int* dev_outputB = nullptr;
	int outputB[(RESOLUTION * (RESOLUTION / 4))] = { 255 };
	cudaMalloc(&dev_outputB, (RESOLUTION * (RESOLUTION / 4)) * sizeof(int));
	cudaMemcpy(dev_outputB, outputB, (RESOLUTION * (RESOLUTION / 4)) * sizeof(int), cudaMemcpyHostToDevice);

	float* dev_mousepos = nullptr;
	cudaMalloc(&dev_mousepos, 3 * sizeof(float));
	cudaMemcpy(dev_mousepos, mousePos, 3 * sizeof(float), cudaMemcpyHostToDevice);
	getColAtLoc<<<(RESOLUTION / 4), RESOLUTION>>>(y, dev_triangles, dev_outputR, dev_outputG, dev_outputB, dev_mousepos, degreesXZ, degreesYZ);
	cudaDeviceSynchronize();
	cudaMemcpy(outputR, dev_outputR, RESOLUTION * (RESOLUTION / 4) * sizeof(int), cudaMemcpyDeviceToHost);
	cudaMemcpy(outputG, dev_outputG, RESOLUTION * (RESOLUTION / 4) * sizeof(int), cudaMemcpyDeviceToHost);
	cudaMemcpy(outputB, dev_outputB, RESOLUTION * (RESOLUTION / 4) * sizeof(int), cudaMemcpyDeviceToHost);
	cudaFree(dev_triangles);
	cudaFree(dev_outputR);
	cudaFree(dev_outputG);
	cudaFree(dev_outputB);
	cudaFree(dev_mousepos);
	Wrapper wrapper;
	wrapper.r = outputR;
	wrapper.g = outputG;
	wrapper.b = outputB;
	return wrapper;
}
float mousePos[3] = { 0.0f, 0.0f, 0.0f };
float degreesXZ = 0;
float degreesYZ = 0;
void mouseCallback(int event, int x, int y, int flags, void* userData)
{

}
bool loadFromObjectFile(string sFilename, float *triangles)
{
	ifstream f(sFilename);
	if (!f.is_open())
		return false;

	// Local cache of verts
	vector<Vec3f> verts;
	int i = -1;
	while (!f.eof())
	{
		char line[128];
		f.getline(line, 128);

		strstream s;
		s << line;

		char junk;

		if (line[0] == 'v')
		{
			Vec3f v;
			s >> junk >> v[0] >> v[1] >> v[2];
			verts.push_back(v);
		}

		if (line[0] == 'f')
		{
			i++;
			int f[3];
			s >> junk >> f[0] >> f[1] >> f[2];
			if (i * 3 < NUMTRIS)
			{
				triangles[i * 3] = verts[f[0] - 1][0];
				triangles[i * 3 + 1] = verts[f[0] - 1][1];
				triangles[i * 3 + 2] = verts[f[0] - 1][2] + 2;
			}
		}
	}

	return true;
}
int main()
{
	Mat canvas;
	float triangles[NUMTRIS * 3];
	srand(time(NULL));
	// loadFromObjectFile("C:/Users/arthu/ObjFiles/ground.obj", triangles);
	for (float x = 0;x < 100;x++)
	{
		for (float y = 0;y < 100;y++)
		{
			triangles[((int)y + (int)x * 100) * 3] = x + 0.1f;
			triangles[((int)y + (int)x * 100) * 3 + 1] = (int)(sin(x / 10) * cos(y / 10) * 10);
			triangles[((int)y + (int)x * 100) * 3 + 2] = y + 0.1f;
		}
	}
	while (true)
	{
		float oT = clock();
		canvas = Mat::zeros(Size2i(RESOLUTION, RESOLUTION), CV_8UC3);
		for (int y = 0;y < 4;y++)
		{
			
			Wrapper col = helper(y, triangles, mousePos, degreesXZ, degreesYZ);
			for (int y2 = 0;y2 < RESOLUTION / 4;y2++)
			{
				for (int x = 0;x < RESOLUTION;x++)
				{
				
					Vec3b& at = canvas.at<Vec3b>(y2 + y * (RESOLUTION / 4), x);
					at.val[0] = col.r[(x + y2 * (RESOLUTION))];
					at.val[1] = col.g[(x + y2 * (RESOLUTION))];
					at.val[2] = col.b[(x + y2 * (RESOLUTION))];
					
				}
			}
			
			
		}
		/* for (int i = 0;i < NUMTRIS;i++)
		{
			triangles[i * 3 + 1] += 1;
		} */ 

		resize(canvas, canvas, { 1000, 1000 });
		printf("%f\n", (clock() - oT) / CLOCKS_PER_SEC);
		imshow("Output:", canvas);
		setMouseCallback("Output:", mouseCallback);
		waitKey(1);
		float3 forward = { cosf(degreesYZ + (3.14159) / 2), 0, sinf(degreesYZ + (3.14159) / 2) };
		if (GetKeyState('S') & 0x8000)
		{
			mousePos[0] += forward.x * -32;
			mousePos[2] += forward.z * -0.32f;
		}
		if (GetKeyState('W') & 0x8000)
		{
			mousePos[0] += forward.x * 32;
			mousePos[2] += forward.z * 0.32f;
		}
		if (GetKeyState('A') & 0x8000)
		{
			mousePos[0] -= 40;
		}
		if (GetKeyState('D') & 0x8000)
		{
			mousePos[0] += 40;
		}
		if (GetKeyState('E') & 0x8000)
		{
			mousePos[1] += 40;
		}
		if (GetKeyState('Q') & 0x8000)
		{
			mousePos[1] -= 40;
		}

		if (GetKeyState('R') & 0x8000)
		{
			degreesXZ -= 0.05f;
		}
		if (GetKeyState('T') & 0x8000)
		{
			degreesXZ += 0.05f;
		}
		if (GetKeyState('F') & 0x8000)
		{
			degreesYZ -= 0.1f;
		}
		if (GetKeyState('G') & 0x8000)
		{
			degreesYZ += 0.1f;
		}
	}
}
