#pragma once

#include "Core.h"
#include "Geometry.h"
#include "Materials.h"
#include "Sampling.h"

#pragma warning( disable : 4244)


class Light
{
public:
	virtual Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& emittedColour, float& pdf) = 0;
	virtual Colour evaluate(const ShadingData& shadingData, const Vec3& wi) = 0;
	virtual float PDF(const ShadingData& shadingData, const Vec3& wi) = 0;
	virtual bool isArea() = 0;
	virtual Vec3 normal(const ShadingData& shadingData, const Vec3& wi) = 0;
	virtual float totalIntegratedPower() = 0;
};

class AreaLight : public Light
{
public:
	Triangle* triangle = NULL;
	Colour emission;
	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& emittedColour, float& pdf)
	{
		emittedColour = emission;
		return triangle->sample(sampler, pdf);
	}
	Colour evaluate(const ShadingData& shadingData, const Vec3& wi)
	{
		if (Dot(wi, triangle->gNormal()) < 0)
		{
			return emission;
		}
		return Colour(0.0f, 0.0f, 0.0f);
	}
	float PDF(const ShadingData& shadingData, const Vec3& wi)
	{
		return 1.0f / triangle->area;
	}
	bool isArea()
	{
		return true;
	}
	Vec3 normal(const ShadingData& shadingData, const Vec3& wi)
	{
		return triangle->gNormal();
	}
	float totalIntegratedPower()
	{
		return (triangle->area * emission.Lum());
	}
};

class BackgroundColour : public Light
{
public:
	Colour emission;
	BackgroundColour(Colour _emission)
	{
		emission = _emission;
	}
	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf)
	{
		Vec3 wi = SamplingDistributions::uniformSampleSphere(sampler->next(), sampler->next());
		pdf = SamplingDistributions::uniformSpherePDF(wi);
		reflectedColour = emission;
		return wi;
	}
	Colour evaluate(const ShadingData& shadingData, const Vec3& wi)
	{
		return emission;
	}
	float PDF(const ShadingData& shadingData, const Vec3& wi)
	{
		return SamplingDistributions::uniformSpherePDF(wi);
	}
	bool isArea()
	{
		return false;
	}
	Vec3 normal(const ShadingData& shadingData, const Vec3& wi)
	{
		return -wi;
	}
	float totalIntegratedPower()
	{
		return emission.Lum() * 4.0f * M_PI;
	}
};


class EnvironmentMap : public Light
{
public:
	Texture* env;
	TabulatedDistributionEnv* tabuDist;


	EnvironmentMap(Texture* _env)
	{
		env = _env;
		tabuDist = new TabulatedDistributionEnv(_env);
	}

	Vec3 sample3(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf)
	{
		// Assignment: Update this code to importance sampling lighting based on luminance of each pixel


		Vec3 wi1 = SamplingDistributions::uniformSampleSphere(sampler->next(), sampler->next());
		float pdf1 = SamplingDistributions::uniformSpherePDF(wi1);
		Colour reflectedColour1 = evaluate(shadingData, wi1);


		float u, v;
		float pdf2 = 0;
		tabuDist->sample(sampler->next(), sampler->next(), u, v, pdf2);
		float theta = v * M_PI;
		float phi = u * 2.0f * M_PI;
		float sinTheta = sinf(theta);
		Vec3 wi2(sinTheta * cosf(phi), cosf(theta), sinTheta * sinf(phi));
		Colour reflectedColour2 = evaluate(shadingData, wi2);

		float pTotal1 = pdf1 + pdf2;

		pdf = pTotal1 / 2.0f;

		if (pTotal1 > 0.0f) {
			reflectedColour = (reflectedColour1 * pdf1 / pTotal1) + (reflectedColour2 * pdf2 / pTotal1);
		}
		return wi1;
	

	}
	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf)
	{
		// Assignment: Update this code to importance sampling lighting based on luminance of each pixel

		float rand = sampler->next();

		if (rand < 0.5f) {
			Vec3 wi1 = SamplingDistributions::uniformSampleSphere(sampler->next(), sampler->next());
			float pdf11 = SamplingDistributions::uniformSpherePDF(wi1);
			Colour reflectedColour1 = evaluate(shadingData, wi1);

			float y = wi1.y;
			if (y < -1) {
				y = -1;
			}
			else if (y > 1) {
				y = 1;
			}

			float theta = acosf(y);
			float phi = atan2f(wi1.z, wi1.x);

			// atan2f range[ -pi, pi]
			if (phi < 0.0f) phi += 2.0f * M_PI;

			float u = phi / (2.0f * M_PI);
			float v = theta / M_PI;
			float pdf12 = tabuDist->getPDF(u, v);

			float pTotal1 = pdf11 + pdf12;

			pdf = pTotal1 / 2.0f;

			if (pTotal1 > 0.0f) {
				reflectedColour = reflectedColour1 * pdf11 / pTotal1;
			}
			return wi1;
		}
		else {
			float u, v;
			float pdf21 = 0;
			tabuDist->sample(sampler->next(), sampler->next(), u, v, pdf21);
			float theta = v * M_PI;
			float phi = u * 2.0f * M_PI;

			float sinTheta = sinf(theta);
			Vec3 wi2(sinTheta * cosf(phi), cosf(theta), sinTheta * sinf(phi));
			Colour reflectedColour2 = evaluate(shadingData, wi2);

			float pdf22 = SamplingDistributions::uniformSpherePDF(wi2);

			float pTotal2 = pdf21 + pdf22;

			pdf = pTotal2 / 2.0f;


			if (pTotal2 > 0.0f) {
				reflectedColour = reflectedColour2 * pdf21 / pTotal2;
			}
			return wi2;
		}

	}
	Vec3 sample2(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf)
	{
		// Assignment: Update this code to importance sampling lighting based on luminance of each pixel

		//Vec3 wi = SamplingDistributions::uniformSampleSphere(sampler->next(), sampler->next());
		//float pdf = SamplingDistributions::uniformSpherePDF(wi);
		//reflectedColour = evaluate(shadingData, wi);

		float u, v;
		tabuDist->sample(sampler->next(), sampler->next(), u, v, pdf);
		float theta = v * M_PI;
		float phi = u * 2.0f * M_PI;

		float sinTheta = sinf(theta);
		Vec3 wi(sinTheta * cosf(phi), cosf(theta), sinTheta * sinf(phi));
		reflectedColour = evaluate(shadingData, wi);

		return wi;
	}
	Vec3 sample1(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf)
	{
		// Assignment: Update this code to importance sampling lighting based on luminance of each pixel

		Vec3 wi = SamplingDistributions::uniformSampleSphere(sampler->next(), sampler->next());
		pdf = SamplingDistributions::uniformSpherePDF(wi);
		reflectedColour = evaluate(shadingData, wi);


		return wi;
	}
	Colour evaluate(const ShadingData& shadingData, const Vec3& wi)
	{
		float u = atan2f(wi.z, wi.x);
		u = (u < 0.0f) ? u + (2.0f * M_PI) : u;
		u = u / (2.0f * M_PI);
		float v = acosf(wi.y) / M_PI;
		return env->sample(u, v);
	}
	float PDF(const ShadingData& shadingData, const Vec3& wi)
	{
		return SamplingDistributions::uniformSpherePDF(wi);
	}
	float PDF1(const ShadingData& shadingData, const Vec3& wi)
	{
		// Assignment: Update this code to return the correct PDF of luminance weighted importance sampling

		// transfer back to u,v

		// acos range [-1,1]
		float y = wi.y;
		if (y < -1) {
			y = -1;
		}
		else if (y > 1) {
			y = 1;
		}

		float theta = acosf(y);

		float phi = atan2f(wi.z, wi.x);

		// atan2f range[ -pi, pi]
		if (phi < 0.0f) phi += 2.0f * M_PI;

		float u = phi / (2.0f * M_PI);
		float v = theta / M_PI;

		float pdf = SamplingDistributions::uniformSpherePDF(wi);

		return tabuDist->getPDF(u, v) + pdf;
	}

	bool isArea()
	{
		return false;
	}
	Vec3 normal(const ShadingData& shadingData, const Vec3& wi)
	{
		return -wi;
	}
	float totalIntegratedPower()
	{
		float total = 0;
		for (int i = 0; i < env->height; i++)
		{
			float st = sinf(((float)i / (float)env->height) * M_PI);
			for (int n = 0; n < env->width; n++)
			{
				total += (env->texels[(i * env->width) + n].Lum() * st);
			}
		}
		total = total / (float)(env->width * env->height);
		return total * 4.0f * M_PI;
	}
};