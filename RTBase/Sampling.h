#pragma once

#include "Core.h"
#include <random>
#include <algorithm>
#include "Imaging.h"

class Sampler
{
public:
	virtual float next() = 0;
};

class MTRandom : public Sampler
{
public:
	std::mt19937 generator;
	std::uniform_real_distribution<float> dist;
	MTRandom(unsigned int seed = 1) : dist(0.0f, 1.0f)
	{
		generator.seed(seed);
	}
	float next()
	{
		return dist(generator);
	}
};

// Note all of these distributions assume z-up coordinate system
class SamplingDistributions
{
public:
	static Vec3 uniformSampleHemisphere(float r1, float r2)
	{
		// Add code here
		float theta = acos(r1);
		float phi = 2 * M_PI * r2;

		return SphericalCoordinates::sphericalToWorld(theta, phi);
	}
	static float uniformHemispherePDF(const Vec3 wi)
	{

		// Add code here
		return 1.0f / (2 * M_PI);
	}
	static Vec3 cosineSampleHemisphere(float r1, float r2)
	{
		// Add code here
		float theta = acos(sqrtf(r1));
		float phi = 2 * M_PI * r2;

		return SphericalCoordinates::sphericalToWorld(theta, phi);
	}
	static float cosineHemispherePDF(const Vec3 wi)
	{

		// Add code here
		return wi.z / M_PI;
	}
	static Vec3 uniformSampleSphere(float r1, float r2)
	{

		// Add code here
		float theta = acos(1 - 2 * r1);
		float phi = 2 * M_PI * r2;

		return SphericalCoordinates::sphericalToWorld(theta, phi);
	}
	static float uniformSpherePDF(const Vec3& wi)
	{
		// Add code here
		return 1.0f / (4 * M_PI);
	}
};

class TabulatedDistributionEnv {
public:
	int width, height;
	std::vector<float> weightedF;
	float totalSum;
	std::vector<float> conditionalCDF;
	std::vector<float> marginalCDF;

	TabulatedDistributionEnv(Texture* env) {
		width = env->width;
		height = env->height;
		weightedF.resize(width * height, 0.0f);
		conditionalCDF.resize(width * height, 0.0f);
		marginalCDF.resize(height, 0.0f);
		float rowSum = 0.0f;
		totalSum = 0.0f;

		// go through all pixel
		for (int i = 0; i < height; i++) {
			// get pixel center as v to get sin theta
			float v = (i + 0.5f) / float(height);
			float sinTheta = sinf(v * M_PI);

			rowSum = 0.0f;

			for (int j = 0; j < width; j++) {
				int index = i * width + j;

				//incorporate F(u,v)
				float lum = env->texels[index].Lum();
				float wF = lum * sinTheta;

				weightedF[index] = wF;
				// accumulate cdf
				rowSum += wF;
				conditionalCDF[index] = rowSum;
			}

			// accumulate cdf
			totalSum += rowSum;
			marginalCDF[i] = totalSum;

			//normalize
			if (rowSum > 0) {
				for (int j = 0; j < width; j++) {
					conditionalCDF[i * width + j] /= rowSum;
				}
			}
		}

		//normalize
		for (int i = 0; i < height; i++) {
			marginalCDF[i] /= totalSum;
		}
	}

	void sample(float r1, float r2, float& u, float& v, float& pdf) const {

		int row = 0;
		// binary search first CDF >= r1
		auto binSR = std::lower_bound(marginalCDF.begin(), marginalCDF.end(), r1);
		row = int(std::distance(marginalCDF.begin(), binSR));
		if (row >= height) row = height - 1;

		int column = 0;
		// binary search first CDF >= r2 (within the row)
		auto start = conditionalCDF.begin() + row * width;
		auto end = start + width;
		auto binSC = std::lower_bound(start, end, r2);
		column = int(std::distance(start, binSC));
		if (column >= width) column = width - 1;

		u = (column + 0.5f) / float(width);
		v = (row + 0.5f) / float(height);

		float sinTheta = sinf(v * M_PI);
		//float deno = 2 * M_PI * M_PI * sinTheta;
		float deno = (2.0f * M_PI / float(width)) * (M_PI / float(height)) * sinTheta;

		if (deno > 0.0f) {
			float wF = weightedF[row * width + column];
			if (wF > 0.0f) {
				pdf = wF / (totalSum * deno);
			}
			else {
				pdf = 0.0f;
			}
		}
		else {
			pdf = 0.0f;
		}


	}


	float getPDF(float u, float v)const {

		int row = std::min(int(v * height), height - 1);
		int column = std::min(int(u * width), width - 1);

		float sinTheta = sinf(v * M_PI);
		//float deno = 2 * M_PI * M_PI * sinTheta;
		float deno = (2.0f * M_PI / float(width))* (M_PI / float(height))* sinTheta;

		float pdf = 0.0f;

		if (deno > 0) {
			float wF = weightedF[row * width + column];
			if (wF > 0) {
				pdf = wF / (totalSum * deno);
			}
		}

		return pdf;
	}

};
