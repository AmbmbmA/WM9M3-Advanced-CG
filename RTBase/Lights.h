#pragma once

#include "Core.h"
#include "Geometry.h"
#include "Materials.h"
#include "Sampling.h"

#pragma warning( disable : 4244)


class SceneBounds
{
public:
	Vec3 sceneCentre;
	float sceneRadius;
};


class Light
{
public:
	virtual Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& emittedColour, float& pdf) = 0;
	//virtual Colour evaluate(const ShadingData& shadingData, const Vec3& wi) = 0;
	virtual Colour evaluate(const Vec3& wi) = 0;
	virtual float PDF(const ShadingData& shadingData, const Vec3& wi) = 0;
	virtual bool isArea() = 0;
	virtual Vec3 normal(const Vec3& wi) = 0;
	virtual float totalIntegratedPower() = 0;
	virtual Vec3 samplePositionFromLight(Sampler* sampler, float& pdf) = 0;
	virtual Vec3 sampleDirectionFromLight(Sampler* sampler, float& pdf) = 0;
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
	Colour evaluate(const Vec3& wi)
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
	Vec3 normal(const Vec3& wi)
	{
		return triangle->gNormal();
	}
	float totalIntegratedPower()
	{
		return (triangle->area * emission.Lum());
	}
	Vec3 samplePositionFromLight(Sampler* sampler, float& pdf)
	{
		return triangle->sample(sampler, pdf);
	}
	Vec3 sampleDirectionFromLight(Sampler* sampler, float& pdf)
	{
		// Add code to sample a direction from the light
		//Vec3 wi = Vec3(0, 0, 1);
		Vec3 wi = SamplingDistributions::cosineSampleHemisphere(sampler->next(), sampler->next());
		pdf = SamplingDistributions::cosineHemispherePDF(wi);
		Frame frame;
		frame.fromVector(triangle->gNormal());
		return frame.toWorld(wi);
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
	Colour evaluate(const Vec3& wi)
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
	Vec3 normal(const Vec3& wi)
	{
		return -wi;
	}
	float totalIntegratedPower()
	{
		return emission.Lum() * 4.0f * M_PI;
	}
	Vec3 samplePositionFromLight(Sampler* sampler, float& pdf)
	{
		Vec3 p = SamplingDistributions::uniformSampleSphere(sampler->next(), sampler->next());
		p = p * use<SceneBounds>().sceneRadius;
		p = p + use<SceneBounds>().sceneCentre;
		pdf = 4 * M_PI * use<SceneBounds>().sceneRadius * use<SceneBounds>().sceneRadius;
		return p;
	}
	Vec3 sampleDirectionFromLight(Sampler* sampler, float& pdf)
	{
		Vec3 wi = SamplingDistributions::uniformSampleSphere(sampler->next(), sampler->next());
		pdf = SamplingDistributions::uniformSpherePDF(wi);
		return wi;
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

	
	Vec3 sampleOld(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf)
	{
		// Assignment: Update this code to importance sampling lighting based on luminance of each pixel

		Vec3 wi = SamplingDistributions::uniformSampleSphere(sampler->next(), sampler->next());
		pdf = SamplingDistributions::uniformSpherePDF(wi);
		reflectedColour = evaluate(wi);

		return wi;
	}

	Vec3 sampleMISWrong2(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf)
	{
		// Assignment: Update this code to importance sampling lighting based on luminance of each pixel

		//Vec3 wi1 = SamplingDistributions::uniformSampleSphere(sampler->next(), sampler->next());
		//float pdf1 = SamplingDistributions::uniformSpherePDF(wi1);
		//Colour reflectedColour1 = evaluate(shadingData, wi1);

		Colour reflectedColour1;
		float pdf1;
		Vec3 wi1 = shadingData.bsdf->sample(shadingData,sampler, reflectedColour1,pdf1);



		float u, v;
		float pdf2 = 0;
		tabuDist->sample(sampler->next(), sampler->next(), u, v, pdf2);
		float theta = v * M_PI;
		float phi = u * 2.0f * M_PI;
		float sinTheta = sinf(theta);
		Vec3 wi2(sinTheta * cosf(phi), cosf(theta), sinTheta * sinf(phi));
		Colour reflectedColour2 = evaluate(wi2);

		float pTotal = pdf1 + pdf2;

		pdf = pTotal;

		if (pTotal > 0.0f) {
			reflectedColour = (reflectedColour1 * pdf1 / pTotal) + (reflectedColour2 * pdf2 / pTotal);
		}
		return wi1;
	

	}
	Vec3 sampleMISWrong1(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf)
	{
		// Assignment: Update this code to importance sampling lighting based on luminance of each pixel

		float rand = sampler->next();

		if (rand < 0.5f) {
			Vec3 wi1 = SamplingDistributions::uniformSampleSphere(sampler->next(), sampler->next());
			float pdf11 = SamplingDistributions::uniformSpherePDF(wi1);
			Colour reflectedColour1 = evaluate(wi1);

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

			pdf = pTotal1;

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
			Colour reflectedColour2 = evaluate(wi2);

			float pdf22 = SamplingDistributions::uniformSpherePDF(wi2);

			float pTotal2 = pdf21 + pdf22;

			pdf = pTotal2;


			if (pTotal2 > 0.0f) {
				reflectedColour = reflectedColour2 * pdf21 / pTotal2;
			}
			return wi2;
		}

	}

	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf)
	{
		// Assignment: Update this code to importance sampling lighting based on luminance of each pixel

		float u, v;
		tabuDist->sample(sampler->next(), sampler->next(), u, v, pdf);
		float theta = v * M_PI;
		float phi = u * 2.0f * M_PI;

		float sinTheta = sinf(theta);
		Vec3 wi(sinTheta * cosf(phi), cosf(theta), sinTheta * sinf(phi));
		reflectedColour = evaluate(wi);

		return wi;
	}
	Colour evaluate(const Vec3& wi)
	{
		float u = atan2f(wi.z, wi.x);
		u = (u < 0.0f) ? u + (2.0f * M_PI) : u;
		u = u / (2.0f * M_PI);
		float v = acosf(wi.y) / M_PI;
		return env->sample(u, v);
	}
	float PDFOld(const ShadingData& shadingData, const Vec3& wi)
	{
		return SamplingDistributions::uniformSpherePDF(wi);
	}
	float PDF(const ShadingData& shadingData, const Vec3& wi)
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

		return tabuDist->getPDF(u, v);
	}

	bool isArea()
	{
		return false;
	}
	Vec3 normal(const Vec3& wi)
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
	Vec3 samplePositionFromLight(Sampler* sampler, float& pdf)
	{
		// Samples a point on the bounding sphere of the scene. Feel free to improve this.
		Vec3 p = SamplingDistributions::uniformSampleSphere(sampler->next(), sampler->next());
		p = p * use<SceneBounds>().sceneRadius;
		p = p + use<SceneBounds>().sceneCentre;
		pdf = 1.0f / (4 * M_PI * SQ(use<SceneBounds>().sceneRadius));
		return p;
	}
	Vec3 sampleDirectionFromLight(Sampler* sampler, float& pdf)
	{
		// Replace this tabulated sampling of environment maps
		float u, v;
		tabuDist->sample(sampler->next(), sampler->next(), u, v, pdf);
		float theta = v * M_PI;
		float phi = u * 2.0f * M_PI;

		float sinTheta = sinf(theta);
		Vec3 wi(sinTheta * cosf(phi), cosf(theta), sinTheta * sinf(phi));
		return wi;

		//Vec3 wi = SamplingDistributions::uniformSampleSphere(sampler->next(), sampler->next());
		//pdf = SamplingDistributions::uniformSpherePDF(wi);
		//return wi;
	}
};

