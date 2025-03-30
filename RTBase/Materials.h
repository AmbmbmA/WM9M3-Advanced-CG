#pragma once

#include "Core.h"
#include "Imaging.h"
#include "Sampling.h"

#pragma warning( disable : 4244)

class BSDF;

class ShadingData
{
public:
	Vec3 x;
	Vec3 wo;
	Vec3 sNormal;
	Vec3 gNormal;
	float tu;
	float tv;
	Frame frame;
	BSDF* bsdf;
	float t;
	ShadingData() {}
	ShadingData(Vec3 _x, Vec3 n)
	{
		x = _x;
		gNormal = n;
		sNormal = n;
		bsdf = NULL;
	}
};

class ShadingHelper
{
public:
	static float fresnelDielectric(float cosTheta, float iorInt, float iorExt)
	{
		// Add code here

		// if exit
		if (cosTheta < 0.0f) {
			float temp = iorInt;
			iorInt = iorExt;
			iorExt = temp;
			cosTheta = -cosTheta;
		}

		float ior = iorExt / iorInt;
		float sinTheta2 = std::max(0.0f, (1.0f - cosTheta * cosTheta));

		float tempTerm = ior * ior * sinTheta2;
		// Total Internal Reflection
		if (tempTerm >= 1) return 1.0f;

		float cosThetaT = std::sqrtf(1 - tempTerm);

		float Fparallel = (cosTheta - ior * cosThetaT) / (cosTheta + ior * cosThetaT);
		float Fperpen = (ior * cosTheta - cosThetaT) / (ior * cosTheta + cosThetaT);

		return (Fparallel * Fparallel + Fperpen * Fperpen) * 0.5f;
	}

	static Colour fresnelConductor(float cosTheta, Colour ior, Colour k)
	{
		// Add code here

		Colour iorKsum2 = ior * ior + k * k;
		float cos2 = cosTheta * cosTheta;
		float sin2 = 1.0f - cos2;

		Colour ior2cos = ior * cosTheta * 2.0f;
		Colour sin2C(sin2, sin2, sin2);
		Colour cos2C(cos2, cos2, cos2);

		Colour Fparallel2 = (iorKsum2 * cos2 - ior2cos + sin2C) / (iorKsum2 * cos2 + ior2cos + sin2C);
		Colour Fperpen2 = (iorKsum2 - ior2cos + cos2C) / (iorKsum2 + ior2cos + cos2C);

		return (Fparallel2 + Fperpen2) * 0.5f;

	}
	static float lambdaGGX(Vec3 wi, float alpha)
	{
		// Add code here
		float cosTheta = wi.z;
		if (cosTheta <= 0.0f) {
			return 0.0f;
		}

		float cos2 = cosTheta * cosTheta;
		float sin2 = 1.0f - cos2;

		if (sin2 < 0.0f) {
			sin2 = 0.0f;
		}

		float tan2 = sin2 / cos2;

		float tempTerm = sqrtf(1 + alpha * alpha * tan2);

		return (tempTerm - 1.0f) * 0.5f;
	}

	static float Gggx(Vec3 wi, Vec3 wo, float alpha)
	{
		// Add code here
		return 1.0f / ((1.0f + lambdaGGX(wi, alpha)) * (1.0f + lambdaGGX(wo, alpha)));
	}
	static float Dggx(Vec3 h, float alpha)
	{
		// Add code here
		float a2 = alpha * alpha;
		float tempTerm = (h.z * h.z) * (a2 - 1.0f) + 1.0f;
		return a2 / (M_PI * tempTerm * tempTerm);
	}
};

class BSDF
{
public:
	Colour emission;
	virtual Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf) = 0;
	virtual Colour evaluate(const ShadingData& shadingData, const Vec3& wi) = 0;
	virtual float PDF(const ShadingData& shadingData, const Vec3& wi) = 0;
	virtual bool isPureSpecular() = 0;
	virtual bool isTwoSided() = 0;
	bool isLight()
	{
		return emission.Lum() > 0 ? true : false;
	}
	void addLight(Colour _emission)
	{
		emission = _emission;
	}
	Colour emit(const ShadingData& shadingData, const Vec3& wi)
	{
		return emission;
	}
	virtual float mask(const ShadingData& shadingData) = 0;
};


class DiffuseBSDF : public BSDF
{
public:
	Texture* albedo;
	DiffuseBSDF() = default;
	DiffuseBSDF(Texture* _albedo)
	{
		albedo = _albedo;
	}
	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf)
	{
		// Add correct sampling code here
		Vec3 wi = SamplingDistributions::cosineSampleHemisphere(sampler->next(), sampler->next());
		pdf = SamplingDistributions::cosineHemispherePDF(wi);
		reflectedColour = albedo->sample(shadingData.tu, shadingData.tv) / M_PI;
		wi = shadingData.frame.toWorld(wi);
		return wi;

	}
	Colour evaluate(const ShadingData& shadingData, const Vec3& wi)
	{

		return albedo->sample(shadingData.tu, shadingData.tv) / M_PI;
	}
	float PDF(const ShadingData& shadingData, const Vec3& wi)
	{
		// Add correct PDF code here
		Vec3 wilocal = shadingData.frame.toLocal(wi);
		return SamplingDistributions::cosineHemispherePDF(wilocal);
	}
	bool isPureSpecular()
	{
		return false;
	}
	bool isTwoSided()
	{
		return true;
	}
	float mask(const ShadingData& shadingData)
	{
		return albedo->sampleAlpha(shadingData.tu, shadingData.tv);
	}
};

class MirrorBSDF : public BSDF
{
public:
	Texture* albedo;
	MirrorBSDF() = default;
	MirrorBSDF(Texture* _albedo)
	{
		albedo = _albedo;
	}

	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf)
	{
		// Replace this with Mirror sampling code
		Vec3 wo = shadingData.frame.toLocal(shadingData.wo);
		Vec3 wr = Vec3(-wo.x, -wo.y, wo.z);
		Vec3 wi = shadingData.frame.toWorld(wr);
		pdf = 1;
		reflectedColour = albedo->sample(shadingData.tu, shadingData.tv) / wr.z;

		return wi;
	}
	Colour evaluate(const ShadingData& shadingData, const Vec3& wi)
	{
		// Replace this with Mirror evaluation code
		return Colour(0, 0, 0);
		//return albedo->sample(shadingData.tu, shadingData.tv) / wi.dot(shadingData.sNormal);
	}
	float PDF(const ShadingData& shadingData, const Vec3& wi)
	{
		// Replace this with Mirror PDF
		return 0;
	}
	bool isPureSpecular()
	{
		return true;
	}
	bool isTwoSided()
	{
		return true;
	}
	float mask(const ShadingData& shadingData)
	{
		return albedo->sampleAlpha(shadingData.tu, shadingData.tv);
	}
};

#define mirrorEdge 0.05f
class ConductorBSDF : public BSDF
{
public:
	Texture* albedo;
	Colour eta;
	Colour k;
	float alpha;
	ConductorBSDF() = default;
	ConductorBSDF(Texture* _albedo, Colour _eta, Colour _k, float roughness)
	{
		albedo = _albedo;
		eta = _eta;
		k = _k;
		alpha = 1.62142f * sqrtf(roughness);
	}
	Vec3 reflect(const Vec3& v, const Vec3& n) {
		return v - n * 2.0f * Dot(v, n);
	}
	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf)
	{
		// Replace this with Conductor sampling code

		//if too smooth, mirror
		if (alpha < mirrorEdge) {
			Vec3 wo = shadingData.frame.toLocal(shadingData.wo);
			Vec3 wr = Vec3(-wo.x, -wo.y, wo.z);
			pdf = 1;

			Vec3 wi = shadingData.frame.toWorld(wr);
			reflectedColour = ShadingHelper::fresnelConductor(wo.z, eta, k) * albedo->sample(shadingData.tu, shadingData.tv) / wo.z;
			return wi;

		}

		Vec3 wo = shadingData.frame.toLocal(shadingData.wo);
		float cosTheta = wo.z;

		// if back, return dark
		if (cosTheta <= 0.0f) {
			pdf = 0;
			reflectedColour = Colour(0, 0, 0);
			return Vec3(0, 0, 0);
		}

		// get wm
		float thetaM = acosf(sqrtf(1.0f / (1 - alpha * alpha * logf(1.0f - sampler->next()))));
		float phiM = 2.0f * M_PI * sampler->next();

		float sinThetaM = sinf(thetaM);
		float cosThetaM = cosf(thetaM);
		Vec3 wm = Vec3(sinThetaM * cosf(phiM), sinThetaM * sinf(phiM), cosThetaM);
		wm = wm.normalize();

		// change wo into wm local
		Frame wmFrame;
		wmFrame.fromVector(wm);
		Vec3 temp = wmFrame.toLocal(wo);
		// reflect 
		temp = Vec3(-temp.x, -temp.y, temp.z);

		// back to shading local
		Vec3 wiLocal = wmFrame.toWorld(temp);

		// if back, dark
		if (wiLocal.z <= 0.0f) {
			pdf = 0;
			reflectedColour = Colour(0, 0, 0);
			return Vec3(0, 0, 0);
		}

		float D = ShadingHelper::Dggx(wm, alpha);
		float G = ShadingHelper::Gggx(wo, wiLocal, alpha);

		pdf = (D * wm.z) / (4.0f * wiLocal.dot(wm));

		// Fresnel
		Colour F = ShadingHelper::fresnelConductor(fabs(wo.dot(wm)), eta, k);

		reflectedColour = albedo->sample(shadingData.tu, shadingData.tv) * (F * D * G) / (4.0f * wo.z * wiLocal.z);

		Vec3 wi = shadingData.frame.toWorld(wiLocal);

		return wi;
	}

	Colour evaluate(const ShadingData& shadingData, const Vec3& wi)
	{
		// Replace this with Conductor evaluation code

		if (alpha < mirrorEdge) {
			Vec3 wo = shadingData.frame.toLocal(shadingData.wo);
			Vec3 wr = Vec3(-wo.x, -wo.y, wo.z);
			Vec3 wi = shadingData.frame.toWorld(wr);

			return ShadingHelper::fresnelConductor(wo.z, eta, k) * albedo->sample(shadingData.tu, shadingData.tv) / wo.z;

		}

		Vec3 wo = shadingData.frame.toLocal(shadingData.wo);
		Vec3 wiLocal = shadingData.frame.toLocal(wi);
		float cosTheta = wo.z;

		// if back, return dark
		if (cosTheta <= 0.0f) {
			return Colour(0, 0, 0);
		}
		
		// get wm as middle 
		Vec3 wm = wiLocal - wo;
		wm = wm.normalize();

		float D = ShadingHelper::Dggx(wm, alpha);
		float G = ShadingHelper::Gggx(wo, wiLocal, alpha);

		// Fresnel
		Colour F = ShadingHelper::fresnelConductor(fabs(wo.dot(wm)), eta, k);

		return albedo->sample(shadingData.tu, shadingData.tv) * (F * D * G) / (4.0f * wo.z * wiLocal.z);
	}

	float PDF(const ShadingData& shadingData, const Vec3& wi)
	{
		// Replace this with Conductor PDF

		if (alpha < mirrorEdge) {
			return 1;
		}
		Vec3 wo = shadingData.frame.toLocal(shadingData.wo);
		Vec3 wiLocal = shadingData.frame.toLocal(wi);
		float cosTheta = wo.z;

		if (cosTheta <= 0.0f || wiLocal.z < 0.0f) {
			return 0;
		}

		// get wm as middle 
		Vec3 wm = wiLocal - wo;
		wm = wm.normalize();
		float D = ShadingHelper::Dggx(wm, alpha);

		return (D * wm.z) / (4.0f * wiLocal.dot(wm));

	}

	bool isPureSpecular()
	{
		return false;
	}
	bool isTwoSided()
	{
		return true;
	}
	float mask(const ShadingData& shadingData)
	{
		return albedo->sampleAlpha(shadingData.tu, shadingData.tv);
	}
};

class GlassBSDF : public BSDF
{
public:
	Texture* albedo;
	float intIOR;
	float extIOR;
	GlassBSDF() = default;
	GlassBSDF(Texture* _albedo, float _intIOR, float _extIOR)
	{
		albedo = _albedo;
		intIOR = _intIOR;
		extIOR = _extIOR;
	}


	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf)
	{
		// Replace this with Glass sampling code
		Vec3 wo = shadingData.frame.toLocal(shadingData.wo);
		Vec3 wr = Vec3(-wo.x, -wo.y, wo.z);
		Vec3 N = shadingData.sNormal;

		float cosTheta = wo.z;

		// exit
		float iorInt = intIOR;
		float iorExt = extIOR;

		float Fw = ShadingHelper::fresnelDielectric(cosTheta, iorInt, iorExt);

		Vec3 wt;
		Vec3 w1;
		if (sampler->next() < Fw) {
			// reflect
			pdf = Fw;
			reflectedColour = albedo->sample(shadingData.tu, shadingData.tv) * Fw / std::fabs(wr.z);
			Vec3 wi = shadingData.frame.toWorld(wr);

			return wi;
		}
		else {
			// transmission
			pdf = 1 - Fw;
			if (cosTheta < 0.0f) {
				iorInt = extIOR;
				iorExt = intIOR;
				cosTheta = -cosTheta;
				N = -N;
			}

			float ior = iorExt / iorInt;
			float sinTheta2 = std::max(0.0f, (1.0f - cosTheta * cosTheta));
			float tempTerm = ior * ior * sinTheta2;
			float cosThetaT = std::sqrtf(1.0f - tempTerm);


			//wt = Vec3((-wo.x) * ior, (-wo.y) * ior, -cosThetaT);

			wt = (-shadingData.wo) * ior + N * (ior * cosTheta - cosThetaT);
			wt = shadingData.frame.toLocal(wt);

			//if (wt.x != w1.x) {
			//	std::cout << " wt: " << wt.x << "  ,  " << wt.y << "  ,  " << wt.z << std::endl;
			//	std::cout << " w1: " << w1.x << "  ,  " << w1.y << "  ,  " << w1.z << std::endl;
			//	std::cout << " diff: " << wt.x - w1.x << std::endl;
			//}


			reflectedColour = albedo->sample(shadingData.tu, shadingData.tv) * ior * ior * (1 - Fw) / std::fabs(wt.z);
			wt = shadingData.frame.toWorld(wt);

			//if (wt.x != w1.x) {
			//	std::cout << " wt: " << wt.x << "  ,  " << wt.y << "  ,  " << wt.z << std::endl;
			//	std::cout << " w1: " << w1.x << "  ,  " << w1.y << "  ,  " << w1.z << std::endl;
			//	std::cout << " diff: " << wt.x - w1.x << std::endl;
			//}

			return wt;

		}

	}
	Colour evaluate(const ShadingData& shadingData, const Vec3& wi)
	{
		// Replace this with Glass evaluation code
		return Colour(0, 0, 0);

	}
	float PDF(const ShadingData& shadingData, const Vec3& wi)
	{
		// Replace this with GlassPDF
		return 0;
	}
	bool isPureSpecular()
	{
		return true;
	}
	bool isTwoSided()
	{
		return false;
	}
	float mask(const ShadingData& shadingData)
	{
		return albedo->sampleAlpha(shadingData.tu, shadingData.tv);
	}
};


class DielectricBSDF : public BSDF
{
public:
	Texture* albedo;
	float intIOR;
	float extIOR;
	float alpha;
	DielectricBSDF() = default;
	DielectricBSDF(Texture* _albedo, float _intIOR, float _extIOR, float roughness)
	{
		albedo = _albedo;
		intIOR = _intIOR;
		extIOR = _extIOR;
		alpha = 1.62142f * sqrtf(roughness);
	}
	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf)
	{
		// Replace this with Dielectric sampling code
		Vec3 wi = SamplingDistributions::cosineSampleHemisphere(sampler->next(), sampler->next());
		pdf = wi.z / M_PI;
		reflectedColour = albedo->sample(shadingData.tu, shadingData.tv) / M_PI;
		wi = shadingData.frame.toWorld(wi);
		return wi;
	}
	Colour evaluate(const ShadingData& shadingData, const Vec3& wi)
	{
		// Replace this with Dielectric evaluation code
		return albedo->sample(shadingData.tu, shadingData.tv) / M_PI;
	}
	float PDF(const ShadingData& shadingData, const Vec3& wi)
	{
		// Replace this with Dielectric PDF
		Vec3 wiLocal = shadingData.frame.toLocal(wi);
		return SamplingDistributions::cosineHemispherePDF(wiLocal);
	}
	bool isPureSpecular()
	{
		return false;
	}
	bool isTwoSided()
	{
		return false;
	}
	float mask(const ShadingData& shadingData)
	{
		return albedo->sampleAlpha(shadingData.tu, shadingData.tv);
	}
};

class OrenNayarBSDF : public BSDF
{
public:
	Texture* albedo;
	float sigma;
	OrenNayarBSDF() = default;
	OrenNayarBSDF(Texture* _albedo, float _sigma)
	{
		albedo = _albedo;
		sigma = _sigma;
	}
	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf)
	{
		// Replace this with OrenNayar sampling code
		Vec3 wi = SamplingDistributions::cosineSampleHemisphere(sampler->next(), sampler->next());
		pdf = wi.z / M_PI;
		reflectedColour = albedo->sample(shadingData.tu, shadingData.tv) / M_PI;
		wi = shadingData.frame.toWorld(wi);
		return wi;
	}
	Colour evaluate(const ShadingData& shadingData, const Vec3& wi)
	{
		// Replace this with OrenNayar evaluation code

		return albedo->sample(shadingData.tu, shadingData.tv) / M_PI;
	}
	float PDF(const ShadingData& shadingData, const Vec3& wi)
	{
		// Replace this with OrenNayar PDF
		Vec3 wiLocal = shadingData.frame.toLocal(wi);
		return SamplingDistributions::cosineHemispherePDF(wiLocal);
	}
	bool isPureSpecular()
	{
		return false;
	}
	bool isTwoSided()
	{
		return true;
	}
	float mask(const ShadingData& shadingData)
	{
		return albedo->sampleAlpha(shadingData.tu, shadingData.tv);
	}
};

#define KS 0.3f
class PlasticBSDF : public BSDF
{
public:
	Texture* albedo;
	float intIOR;
	float extIOR;
	float alpha;
	PlasticBSDF() = default;
	PlasticBSDF(Texture* _albedo, float _intIOR, float _extIOR, float roughness)
	{
		albedo = _albedo;
		intIOR = _intIOR;
		extIOR = _extIOR;
		alpha = 1.62142f * sqrtf(roughness);
	}
	float alphaToPhongExponent()
	{
		return (2.0f / SQ(std::max(alpha, 0.001f))) - 2.0f;
	}
	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf)
	{
		// Replace this with Plastic sampling code

		float ex = alphaToPhongExponent();
		Vec3 wo = shadingData.frame.toLocal(shadingData.wo);
		Vec3 wr = Vec3(-wo.x, -wo.y, wo.z);
		wr = shadingData.frame.toWorld(wr);

		//float ks = ShadingHelper::fresnelDielectric(wo.z, extIOR, intIOR);
		float F0 = powf((intIOR - extIOR) / (intIOR + extIOR), 2);
		float F = F0 + (1.0f - F0) * powf(1.0f - wo.z, 5);
		float ks = F;
		//std::cout << ks << std::endl;
		//float ks = KS;

		if (sampler->next() < ks) {
			float theataLobe = acosf(powf(sampler->next(), (1.0f / (ex + 1))));
			float phiLobe = 2 * M_PI * sampler->next();

			Vec3 wLobe = Vec3(sinf(theataLobe) * cosf(phiLobe), sinf(theataLobe) * sinf(phiLobe), cosf(theataLobe));
			Frame wrFrame;
			wrFrame.fromVector(wr);

			Vec3 wi = wrFrame.toWorld(wLobe);

			pdf = ((ex + 1.0f) / (2.0f * M_PI)) * powf(std::max(0.0f, wr.dot(wi)), ex) * ks;

			reflectedColour = albedo->sample(shadingData.tu, shadingData.tv) * ((ex + 2.0f) / (2.0f * M_PI)) * powf(std::max(0.0f, wr.dot(wi)), ex) * ks;
			//reflectedColour = Colour(1, 1, 1) * ((ex + 2.0f) / (2.0f * M_PI)) * powf(std::max(0.0f, wr.dot(wi)), ex) * ks;

			return wi;
		}
		else {

			Vec3 wi = SamplingDistributions::cosineSampleHemisphere(sampler->next(), sampler->next());
			pdf = SamplingDistributions::cosineHemispherePDF(wi) * (1 - ks);
			reflectedColour = albedo->sample(shadingData.tu, shadingData.tv) / M_PI * (1 - ks);
			wi = shadingData.frame.toWorld(wi);
			return wi;
		}
	}

	Colour evaluate(const ShadingData& shadingData, const Vec3& wi)
	{
		// Replace this with Plastic evaluation code

		float ex = alphaToPhongExponent();
		Vec3 wo = shadingData.frame.toLocal(shadingData.wo);
		Vec3 wr = Vec3(-wo.x, -wo.y, wo.z);
		wr = shadingData.frame.toWorld(wr);

		//float ks = ShadingHelper::fresnelDielectric(wo.z, extIOR, intIOR);
		float F0 = powf((intIOR - extIOR) / (intIOR + extIOR), 2);
		float F = F0 + (1.0f - F0) * powf(1.0f - wo.z, 5);
		float ks = F;
		//float ks = KS;

		Colour Colour1 = albedo->sample(shadingData.tu, shadingData.tv) * ((ex + 2.0f) / (2.0f * M_PI)) * powf(std::max(0.0f, wr.dot(wi)), ex) * ks;
		Colour Colour2 = albedo->sample(shadingData.tu, shadingData.tv) / M_PI * (1.0f - ks);

		return Colour1 + Colour2;
	}

	float PDF(const ShadingData& shadingData, const Vec3& wi)
	{
		// Replace this with Plastic PDF

		float ex = alphaToPhongExponent();
		Vec3 wo = shadingData.frame.toLocal(shadingData.wo);
		Vec3 wr = Vec3(-wo.x, -wo.y, wo.z);
		wr = shadingData.frame.toWorld(wr);

		//float ks = ShadingHelper::fresnelDielectric(wo.z, extIOR, intIOR);
		float F0 = powf((intIOR - extIOR) / (intIOR + extIOR), 2);
		float F = F0 + (1.0f - F0) * powf(1.0f - wo.z, 5);
		float ks = F;
		//float ks = KS;

		Vec3 wiLocal = shadingData.frame.toLocal(wi);

		float pdf1 = ((ex + 1.0f) / (2.0f * M_PI)) * powf(std::max(0.0f, wr.dot(wi)), ex) * ks;
		float pdf2 = SamplingDistributions::cosineHemispherePDF(wiLocal) * (1.0f - ks);
		return pdf1 + pdf2;
	}

	bool isPureSpecular()
	{
		return false;
	}
	bool isTwoSided()
	{
		return true;
	}
	float mask(const ShadingData& shadingData)
	{
		return albedo->sampleAlpha(shadingData.tu, shadingData.tv);
	}
};

class LayeredBSDF : public BSDF
{
public:
	BSDF* base;
	Colour sigmaa;
	float thickness;
	float intIOR;
	float extIOR;
	LayeredBSDF() = default;
	LayeredBSDF(BSDF* _base, Colour _sigmaa, float _thickness, float _intIOR, float _extIOR)
	{
		base = _base;
		sigmaa = _sigmaa;
		thickness = _thickness;
		intIOR = _intIOR;
		extIOR = _extIOR;
	}
	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf)
	{
		// Add code to include layered sampling
		return base->sample(shadingData, sampler, reflectedColour, pdf);
	}
	Colour evaluate(const ShadingData& shadingData, const Vec3& wi)
	{
		// Add code for evaluation of layer

		return base->evaluate(shadingData, wi);
	}
	float PDF(const ShadingData& shadingData, const Vec3& wi)
	{
		// Add code to include PDF for sampling layered BSDF
		return base->PDF(shadingData, wi);
	}
	bool isPureSpecular()
	{
		return base->isPureSpecular();
	}
	bool isTwoSided()
	{
		return true;
	}
	float mask(const ShadingData& shadingData)
	{
		return base->mask(shadingData);
	}
};