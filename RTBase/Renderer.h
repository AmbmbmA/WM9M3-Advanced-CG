#pragma once

#include "Core.h"
#include "Sampling.h"
#include "Geometry.h"
#include "Imaging.h"
#include "Materials.h"
#include "Lights.h"
#include "Scene.h"
#include "GamesEngineeringBase.h"
#include <thread>
#include <functional>
#include <mutex>

#include <OpenImageDenoise/oidn.hpp>

#define MAX_DEPTH_PathT 16


class VPL {
public:
	ShadingData shadingData;
	Colour Le;
	bool isLight;


	VPL(ShadingData _shadingData, Colour _Le, bool _islight) {
		shadingData = _shadingData;
		Le = _Le;
		isLight = _islight;

	}

};
class RayTracer
{
public:
	Scene* scene;
	GamesEngineeringBase::Window* canvas;
	Film* film;
	Film* filmAlbedo;
	Film* filmNormal;
	MTRandom* samplers;
	std::thread** threads;
	int numProcs;
	std::vector<int> tileSPPGlobal;

	std::vector<VPL> VPLs;

	void init(Scene* _scene, GamesEngineeringBase::Window* _canvas)
	{
		scene = _scene;
		canvas = _canvas;
		film = new Film();
		filmAlbedo = new Film();
		filmNormal = new Film();
		film->init((unsigned int)scene->camera.width, (unsigned int)scene->camera.height, new BoxFilter());
		//filmAlbedo->init((unsigned int)scene->camera.width, (unsigned int)scene->camera.height, new BoxFilter());
		//filmNormal->init((unsigned int)scene->camera.width, (unsigned int)scene->camera.height, new BoxFilter());
		SYSTEM_INFO sysInfo;
		GetSystemInfo(&sysInfo);
		numProcs = sysInfo.dwNumberOfProcessors;
		threads = new std::thread * [numProcs];
		samplers = new MTRandom[numProcs];
		const int tileSize = 16;

		int numTilesX = (film->width + tileSize - 1) / tileSize;
		int numTilesY = (film->height + tileSize - 1) / tileSize;
		int totalTiles = numTilesX * numTilesY;
		tileSPPGlobal.resize(totalTiles, 0);

		clear();
	}
	void clear()
	{
		film->clear();
		tileSPPGlobal.clear();
		const int tileSize = 16;
		int numTilesX = (film->width + tileSize - 1) / tileSize;
		int numTilesY = (film->height + tileSize - 1) / tileSize;
		int totalTiles = numTilesX * numTilesY;
		tileSPPGlobal.resize(totalTiles, 0);

		//filmAlbedo->clear();
		//filmNormal->clear();
	}


	Colour computeDirect(ShadingData shadingData, Sampler* sampler)
	{
		// Is surface is specular we cannot computing direct lighting
		if (shadingData.bsdf->isPureSpecular() == true)
		{
			return Colour(0.0f, 0.0f, 0.0f);
		}
		// Compute direct lighting here
		float pmf;
		Light* light = scene->sampleLight(sampler, pmf);
		if (!light) return Colour(0.0f, 0.0f, 0.0f);

		float pdfLight;
		Colour emit(0, 0, 0);
		Vec3 lightPos = light->sample(shadingData, sampler, emit, pdfLight);

		float pdf = pmf * pdfLight;
		if (pdf < 0) return Colour(0.0f, 0.0f, 0.0f);

		if (light->isArea()) {
			Vec3 wi = lightPos - shadingData.x;
			float r2Inv = 1.0f / wi.lengthSq();
			wi = wi.normalize();
			float costheta = max(0, wi.dot(shadingData.sNormal));
			float costhetaL = max(0, -wi.dot(light->normal(wi)));
			float GeomtryTermHalfArea = costheta * costhetaL * r2Inv;
			if (GeomtryTermHalfArea > 0) {
				if (!scene->visible(shadingData.x, lightPos)) {
					return Colour(0.0f, 0.0f, 0.0f);

				}
				else {
					Colour f = shadingData.bsdf->evaluate(shadingData, wi);
					return  f * emit * GeomtryTermHalfArea / pdf;
				}
			}
			else {
				return Colour(0.0f, 0.0f, 0.0f);
			}

		}
		else {
			Vec3 wi = lightPos;
			wi = wi.normalize();
			float GeomtryTermHalfArea = max(0, wi.dot(shadingData.sNormal));
			if (GeomtryTermHalfArea > 0) {
				if (!scene->visible(shadingData.x, shadingData.x + (lightPos * 10000.0f))) {
					return Colour(0.0f, 0.0f, 0.0f);
				}
				else {
					Colour f = shadingData.bsdf->evaluate(shadingData, wi);
					return  f * emit * GeomtryTermHalfArea / pdf;
				}
			}
			else {
				return Colour(0.0f, 0.0f, 0.0f);

			}

		}

	}

	Colour pathTrace(Ray& r, Colour& pathThroughput, int depth, Sampler* sampler, bool canHitLight = true)
	{
		// Add pathtracer code here

		IntersectionData intersection = scene->traverse(r);
		ShadingData shadingData = scene->calculateShadingData(intersection, r);
		if (shadingData.t < FLT_MAX)
		{
			if (shadingData.bsdf->isLight())
			{
				if (canHitLight == true)
				{
					return pathThroughput * shadingData.bsdf->emit(shadingData, shadingData.wo);
				}
				else
				{
					return Colour(0.0f, 0.0f, 0.0f);
				}
			}
			Colour direct = pathThroughput * computeDirect(shadingData, sampler);
			if (depth > MAX_DEPTH_PathT)
			{
				return direct;
			}
			float russianRouletteProbability = min(pathThroughput.Lum(), 0.9f);
			if (sampler->next() < russianRouletteProbability)
			{
				pathThroughput = pathThroughput / russianRouletteProbability;
			}
			else
			{
				return direct;
			}
			Colour bsdf;
			float pdf;

			//Vec3 wi = SamplingDistributions::cosineSampleHemisphere(sampler->next(), sampler->next());
			//pdf = SamplingDistributions::cosineHemispherePDF(wi);
			//wi = shadingData.frame.toWorld(wi);
			//bsdf = shadingData.bsdf->evaluate(shadingData, wi);

			Vec3 wi = shadingData.bsdf->sample(shadingData, sampler, bsdf, pdf);
			pathThroughput = pathThroughput * bsdf * fabsf(Dot(wi, shadingData.sNormal)) / pdf;
			r.init(shadingData.x + (wi * 0.001f), wi);
			return (direct + pathTrace(r, pathThroughput, depth + 1, sampler, shadingData.bsdf->isPureSpecular()));
		}
		return scene->background->evaluate(r.dir);
		//return Colour(0.0f, 0.0f, 0.0f);

	}

	Colour computeDirectMIS(ShadingData shadingData, Sampler* sampler)
	{
		// Is surface is specular we cannot computing direct lighting
		if (shadingData.bsdf->isPureSpecular() == true)
		{
			return Colour(0.0f, 0.0f, 0.0f);
		}
		// Compute direct lighting here
		float pmf;
		Light* light = scene->sampleLight(sampler, pmf);
		if (!light) return Colour(0.0f, 0.0f, 0.0f);

		float pdfLight;
		Colour emit(0, 0, 0);
		Vec3 lightPos = light->sample(shadingData, sampler, emit, pdfLight);

		float pdf = pmf * pdfLight;
		if (pdf < 0) return Colour(0.0f, 0.0f, 0.0f);

		Vec3 wi(0, 0, 0);
		float GeomtryTermHalfArea = 0.0f;
		float pdfWeight = 1.0f;
		if (light->isArea()) {
			wi = lightPos - shadingData.x;
			float r2Inv = 1.0f / wi.lengthSq();
			wi = wi.normalize();
			float costheta = max(0, wi.dot(shadingData.sNormal));
			float costhetaL = max(0, -wi.dot(light->normal(wi)));
			GeomtryTermHalfArea = costheta * costhetaL * r2Inv;
			if (GeomtryTermHalfArea > 0) {
				if (!scene->visible(shadingData.x, lightPos)) {
					return Colour(0.0f, 0.0f, 0.0f);
				}
			}
			else {
				return Colour(0.0f, 0.0f, 0.0f);
			}



		}
		else {

			wi = lightPos;
			wi = wi.normalize();
			GeomtryTermHalfArea = max(0, wi.dot(shadingData.sNormal));
			if (GeomtryTermHalfArea > 0) {
				if (!scene->visible(shadingData.x, shadingData.x + (lightPos * 10000.0f))) {
					return Colour(0.0f, 0.0f, 0.0f);
				}
			}
			else {
				return Colour(0.0f, 0.0f, 0.0f);
			}
			float pdfBSDF = shadingData.bsdf->PDF(shadingData, wi);

			pdfWeight = pdf / (pdf + pdfBSDF);
		}
		Colour f = shadingData.bsdf->evaluate(shadingData, wi);


		return  (f * emit * GeomtryTermHalfArea / pdf) * pdfWeight;

	}
	Colour pathTraceMIS(Ray& r, Colour& pathThroughput, int depth, Sampler* sampler, ShadingData prevSD, bool canHitLight = true)
	{
		// Add pathtracer code here

		IntersectionData intersection = scene->traverse(r);
		ShadingData shadingData = scene->calculateShadingData(intersection, r);
		if (shadingData.t < FLT_MAX)
		{
			if (shadingData.bsdf->isLight())
			{
				if (canHitLight == true)
				{
					return pathThroughput * shadingData.bsdf->emit(shadingData, shadingData.wo);

				}
				else
				{
					return Colour(0.0f, 0.0f, 0.0f);
				}
			}
			Colour direct = pathThroughput * computeDirectMIS(shadingData, sampler);
			if (depth > MAX_DEPTH_PathT)
			{
				return direct;
			}
			float russianRouletteProbability = min(pathThroughput.Lum(), 0.9f);
			if (sampler->next() < russianRouletteProbability)
			{
				pathThroughput = pathThroughput / russianRouletteProbability;
			}
			else
			{
				return direct;
			}


			Colour bsdf;
			float pdfBSDF;
			Vec3 wi = shadingData.bsdf->sample(shadingData, sampler, bsdf, pdfBSDF);

			float pmf = 1.0f / scene->lights.size();
			float pdfLight = scene->background->PDF(shadingData, wi);
			float pdfenv = pmf * pdfLight;

			float pdfWeight = pdfBSDF / (pdfenv + pdfBSDF);

			pathThroughput = (pathThroughput * bsdf * fabsf(Dot(wi, shadingData.sNormal)) / pdfBSDF) * pdfWeight;

			r.init(shadingData.x + (wi * 0.001f), wi);
			return (direct + pathTraceMIS(r, pathThroughput, depth + 1, sampler, shadingData, shadingData.bsdf->isPureSpecular()));
		}

		Colour envColor = scene->background->evaluate(r.dir);
		if (!prevSD.bsdf || prevSD.bsdf->isPureSpecular()) {
			return envColor;
		}
		float pmf = 1.0f / scene->lights.size();
		float pdfLight = scene->background->PDF(shadingData, r.dir);
		float pdfenv = pmf * pdfLight;
		float pdfbsdf = prevSD.bsdf->PDF(prevSD, r.dir);

		float weight = pdfenv / (pdfenv + pdfbsdf);

		return envColor * weight;

	}

	Colour direct(Ray& r, Sampler* sampler)
	{
		// Compute direct lighting for an image sampler here
		IntersectionData intersection = scene->traverse(r);
		ShadingData shadingData = scene->calculateShadingData(intersection, r);
		if (shadingData.t < FLT_MAX)
		{
			if (shadingData.bsdf->isLight())
			{
				return shadingData.bsdf->emit(shadingData, shadingData.wo);
			}
			return computeDirect(shadingData, sampler);
		}
		return scene->background->evaluate(r.dir);
		//return Colour(0.0f, 0.0f, 0.0f);

	}
	void connectToCamera(Vec3 p, Vec3 n, Colour col) {
		float x, y;
		if (!scene->camera.projectOntoCamera(p, x, y)) return;

		Vec3 pToCam = scene->camera.origin - p;
		if (pToCam.length() <= 0.0f) return;
		float r2Inv = 1.0f / pToCam.lengthSq();
		pToCam = pToCam.normalize();

		float costheta = max(0, -pToCam.dot(scene->camera.viewDirection));
		float costhetaL = max(0, pToCam.dot(n));
		float GeomtryTermHalfArea = costheta * costhetaL * r2Inv;

		if (GeomtryTermHalfArea > 0) {
			if (!scene->visible(p, scene->camera.origin)) {
				return;
			}
			else {
				float cos2 = costheta * costheta;
				float cos4 = cos2 * cos2;
				float we = 1.0f / (scene->camera.Afilm * cos4);
				film->splat(x, y, col * GeomtryTermHalfArea * we);
			}
		}
		else {
			return;
		}

	}

	void lightTrace(Sampler* sampler) {
		float pmf;
		Light* light = scene->sampleLight(sampler, pmf);
		if (!light) return;

		float pdfPosition = 0.0f;
		Vec3 lightPos = light->samplePositionFromLight(sampler, pdfPosition);
		//std::cout << " pdf1 : " << pdfPosition << std::endl;

		float pdfDirection = 0.0f;
		Vec3 wi = light->sampleDirectionFromLight(sampler, pdfDirection);
		wi = wi.normalize();
		float pdfTotal = pmf * pdfPosition;

		if (pdfTotal <= 0.0f) return;

		Colour Le = light->evaluate(-wi);
		Colour col = Le / pdfTotal;

		connectToCamera(lightPos, light->normal(-wi), col);

		Ray r(lightPos + (wi * 0.001f), wi);

		Le = Le * wi.dot(light->normal(-wi));
		pdfTotal *= pdfDirection;

		lightTracePath(r, Colour(1, 1, 1), Le / pdfTotal, sampler, 0);

	}

	void lightTracePath(Ray& r, Colour pathThroughput, Colour Le, Sampler* sampler, int depth) {
		IntersectionData intersection = scene->traverse(r);
		ShadingData shadingData = scene->calculateShadingData(intersection, r);

		if (shadingData.t < FLT_MAX)
		{
			Vec3 wi1 = scene->camera.origin - shadingData.x;
			wi1 = wi1.normalize();

			connectToCamera(shadingData.x, shadingData.sNormal, (pathThroughput * shadingData.bsdf->evaluate(shadingData, -wi1) * Le));

			if (depth > MAX_DEPTH_PathT)
			{
				return;
			}
			float russianRouletteProbability = min(pathThroughput.Lum(), 0.9f);
			if (sampler->next() < russianRouletteProbability)
			{
				pathThroughput = pathThroughput / russianRouletteProbability;
			}
			else
			{
				return;
			}

			float pdf;
			Colour bsdf;
			Vec3 wi = shadingData.bsdf->sample(shadingData, sampler, bsdf, pdf);
			wi = wi.normalize();
			pathThroughput = pathThroughput * bsdf * fabsf(Dot(wi, shadingData.sNormal)) / pdf;

			r.init(shadingData.x + (wi * 0.001f), wi);
			lightTracePath(r, pathThroughput, Le, sampler, depth + 1);
		}
	}

	void traceVPLs(Sampler* sampler, int N_VPLs) {
		VPLs.clear();
		VPLs.reserve(N_VPLs);
		for (int i = 0; i < N_VPLs; ++i) {
			float pmf;
			Light* light = scene->sampleLight(sampler, pmf);
			if (!light) continue;

			float pdfPosition = 0.0f;
			Vec3 lightPos = light->samplePositionFromLight(sampler, pdfPosition);
			float pdfDirection = 0.0f;
			Vec3 wi = light->sampleDirectionFromLight(sampler, pdfDirection);
			wi = wi.normalize();
			float pdfTotal = pmf * pdfPosition * (float)N_VPLs;

			if (pdfTotal <= 0.0f) continue;
			Colour Le = light->evaluate(-wi);

			Colour col = Le / pdfTotal;

			VPLs.emplace_back(VPL(ShadingData(lightPos, light->normal(-wi)), col, true));

			Ray r(lightPos + (wi * 0.001f), wi);

			Le = Le * wi.dot(light->normal(-wi));
			pdfTotal *= pdfDirection;

			VPLTracePath(r, Colour(1, 1, 1), Le / pdfTotal, sampler, 0);
		}

	}

	void VPLTracePath(Ray& r, Colour pathThroughput, Colour Le, Sampler* sampler, int depth) {
		IntersectionData intersection = scene->traverse(r);
		ShadingData shadingData = scene->calculateShadingData(intersection, r);

		if (shadingData.t < FLT_MAX)
		{
			if (shadingData.bsdf->isLight()) {
				return;
			}
			else {
				VPLs.emplace_back(VPL(shadingData, pathThroughput * Le, shadingData.bsdf->isLight()));
			}

			if (depth > MAX_DEPTH_PathT)
			{
				return;
			}
			float russianRouletteProbability = min(pathThroughput.Lum(), 0.9f);
			if (sampler->next() < russianRouletteProbability)
			{
				pathThroughput = pathThroughput / russianRouletteProbability;
			}
			else
			{
				return;
			}

			float pdf;
			Colour bsdf;
			Vec3 wi = shadingData.bsdf->sample(shadingData, sampler, bsdf, pdf);
			wi = wi.normalize();
			pathThroughput = pathThroughput * bsdf * fabsf(Dot(wi, shadingData.sNormal)) / pdf;

			r.init(shadingData.x + (wi * 0.001f), wi);
			VPLTracePath(r, pathThroughput, Le, sampler, depth + 1);
		}

	}

	Colour ComputeDirectInstantRadiosity(ShadingData shadingData)
	{
		if (shadingData.bsdf->isPureSpecular() == true)
		{
			return Colour(0.0f, 0.0f, 0.0f);
		}

		Colour result(0, 0, 0);
		for (auto vpl : VPLs) {
			Colour temp(0, 0, 0);
			Vec3 wi(0, 0, 0);
			float GeomtryTermHalfArea = 0;

			wi = vpl.shadingData.x - shadingData.x;
			float r2Inv = 1.0f / wi.lengthSq();
			wi = wi.normalize();
			float costheta = max(0, wi.dot(shadingData.sNormal));
			float costhetaL = max(0, -wi.dot(vpl.shadingData.sNormal));
			GeomtryTermHalfArea = costheta * costhetaL * r2Inv;
			if (GeomtryTermHalfArea > 0) {
				if (!scene->visible(shadingData.x, vpl.shadingData.x)) {
					GeomtryTermHalfArea = 0;
				}
			}
			else {
				GeomtryTermHalfArea = 0;
			}

			if (vpl.isLight) {
				temp = vpl.Le * shadingData.bsdf->evaluate(shadingData, wi) * GeomtryTermHalfArea;
			}
			else {
				temp = vpl.shadingData.bsdf->evaluate(vpl.shadingData, -wi) * shadingData.bsdf->evaluate(shadingData, wi) * GeomtryTermHalfArea;
			}

			result = result + temp;
		}
		return result;
	}
	Colour directInstantRadiosity(Ray& r)
	{
		// Compute direct lighting for an image sampler here
		IntersectionData intersection = scene->traverse(r);
		ShadingData shadingData = scene->calculateShadingData(intersection, r);
		if (shadingData.t < FLT_MAX)
		{
			if (shadingData.bsdf->isLight())
			{
				return shadingData.bsdf->emit(shadingData, shadingData.wo);
			}
			return ComputeDirectInstantRadiosity(shadingData);
		}
		return scene->background->evaluate(r.dir);
		//return Colour(0.0f, 0.0f, 0.0f);

	}
	Colour albedo(Ray& r)
	{
		IntersectionData intersection = scene->traverse(r);
		ShadingData shadingData = scene->calculateShadingData(intersection, r);
		if (shadingData.t < FLT_MAX)
		{
			if (shadingData.bsdf->isLight())
			{
				return shadingData.bsdf->emit(shadingData, shadingData.wo);
			}
			return shadingData.bsdf->evaluate(shadingData, Vec3(0, 1, 0));
		}
		return scene->background->evaluate(r.dir);
	}


	Colour viewNormals(Ray& r)
	{
		IntersectionData intersection = scene->traverse(r);
		if (intersection.t < FLT_MAX)
		{
			ShadingData shadingData = scene->calculateShadingData(intersection, r);
			return Colour(fabsf(shadingData.sNormal.x), fabsf(shadingData.sNormal.y), fabsf(shadingData.sNormal.z));
		}
		return Colour(0.0f, 0.0f, 0.0f);
	}

	void renderTileBased(int tileX, int tileY, int tileSize, int filmWidth, int filmHeight) {
		int startX = tileX * tileSize;
		int startY = tileY * tileSize;
		int endX = min(startX + tileSize, filmWidth);
		int endY = min(startY + tileSize, filmHeight);


		for (int y = startY; y < endY; y++) {
			for (int x = startX; x < endX; x++) {

				unsigned char r, g, b;
				film->tonemap(x, y, r, g, b);
				canvas->draw(x, y, r, g, b);
			}
		}
	}

	void splatTileBased(int tileX, int tileY, int tileSize, int filmWidth, int filmHeight) {
		int startX = tileX * tileSize;
		int startY = tileY * tileSize;
		int endX = min(startX + tileSize, filmWidth);
		int endY = min(startY + tileSize, filmHeight);


		for (int y = startY; y < endY; y++) {
			for (int x = startX; x < endX; x++) {
				//float px = x + 0.5f;
				//float py = y + 0.5f;				
				float px = x + samplers->next();
				float py = y + samplers->next();
				Ray ray = scene->camera.generateRay(px, py);


				//Colour col = direct(ray, samplers);

				Colour pathThroughput(1.0f, 1.0f, 1.0f);
				Colour col = pathTrace(ray, pathThroughput, 0, samplers);

				ShadingData temp;
				//Colour col = pathTraceMIS(ray, pathThroughput, 0, samplers, temp);

				//Colour col = directInstantRadiosity(ray);


				film->splat(px, py, col);

				//col = albedo(ray);
				//filmAlbedo->splat(px, py, col);

				//col = viewNormals(ray);
				//filmNormal->splat(px, py, col);

			}
		}
	}

	void splatTileBasedLightTracing(int num) {

		for (int i = 0; i < num; ++i) {
			lightTrace(samplers);
		}

	}


	void render1() {
		film->incrementSPP();
		filmAlbedo->incrementSPP();
		filmNormal->incrementSPP();

		const int filmWidth = film->width;
		const int filmHeight = film->height;

		int numCore = std::thread::hardware_concurrency();
		//numCore = 1;

		const int tileSize = 32;

		int numTilesX = (filmWidth + tileSize - 1) / tileSize;
		int numTilesY = (filmHeight + tileSize - 1) / tileSize;
		int totalTiles = numTilesX * numTilesY;


		// atomic counter
		std::atomic<int> nextTile(0);

		int paths = (filmWidth * filmHeight) / totalTiles;

		std::vector<std::thread> threads;
		threads.reserve(numCore);

		//traceVPLs(samplers, 10);

		// multi-thread for splatting
		auto Splat = [&]() {
			while (true) {
				int tileIndex = nextTile.fetch_add(1);
				if (tileIndex >= totalTiles)
					break;
				int tileX = tileIndex % numTilesX;
				int tileY = tileIndex / numTilesX;
				splatTileBased(tileX, tileY, tileSize, filmWidth, filmHeight);
				//splatTileBasedLightTracing(paths);
			}
			};
		for (int i = 0; i < numCore; i++) {
			threads.emplace_back(Splat);
		}
		for (auto& t : threads) {
			if (t.joinable())
				t.join();
		}
		threads.clear();

		// multi thread for render

		////oidn::DeviceRef device = oidn::newDevice(); // CPU or GPU if available
		//oidn::DeviceRef device = oidn::newDevice(oidn::DeviceType::CPU);
		//device.commit();

		//// Create buffers for input/output images accessible by both host (CPU) and device (CPU/GPU)
		//oidn::BufferRef colorBuf = device.newBuffer(filmWidth * filmHeight * 3 * sizeof(float));
		//oidn::BufferRef albedoBuf = device.newBuffer(filmWidth * filmHeight * 3 * sizeof(float));
		//oidn::BufferRef normalBuf = device.newBuffer(filmWidth * filmHeight * 3 * sizeof(float));
		//oidn::BufferRef outputBuf = device.newBuffer(filmWidth * filmHeight * 3 * sizeof(float));

		//// Create a filter for denoising a beauty (color) image using prefiltered auxiliary images too
		//oidn::FilterRef filter = device.newFilter("RT"); // generic ray tracing filter
		//filter.setImage("color", colorBuf, oidn::Format::Float3, filmWidth, filmHeight); // beauty
		//filter.setImage("albedo", albedoBuf, oidn::Format::Float3, filmWidth, filmHeight); // auxiliary
		//filter.setImage("normal", normalBuf, oidn::Format::Float3, filmWidth, filmHeight); // auxiliary
		//filter.setImage("output", outputBuf, oidn::Format::Float3, filmWidth, filmHeight); // denoised beauty
		//filter.set("hdr", true); // beauty image is HDR
		//filter.set("cleanAux", true); // auxiliary images will be prefiltered
		//filter.commit();


		//float* colorPtr = (float*)colorBuf.getData();
		//float* albedoPtr = (float*)albedoBuf.getData();
		//float* normalPtr = (float*)normalBuf.getData();
		//size_t dataSize = film->width * film->height * 3 * sizeof(float);
		//memcpy(colorPtr, film->film, dataSize);
		//memcpy(albedoPtr, filmAlbedo->film, dataSize);
		//memcpy(normalPtr, filmNormal->film, dataSize);

		//// Create a separate filter for denoising an auxiliary albedo image (in-place)
		//oidn::FilterRef albedoFilter = device.newFilter("RT"); // same filter type as for beauty
		//albedoFilter.setImage("albedo", albedoBuf, oidn::Format::Float3, filmWidth, filmHeight);
		//albedoFilter.setImage("output", albedoBuf, oidn::Format::Float3, filmWidth, filmHeight);
		//albedoFilter.commit();

		////// Create a separate filter for denoising an auxiliary normal image (in-place)
		//oidn::FilterRef normalFilter = device.newFilter("RT"); // same filter type as for beauty
		//normalFilter.setImage("normal", normalBuf, oidn::Format::Float3, filmWidth, filmHeight);
		//normalFilter.setImage("output", normalBuf, oidn::Format::Float3, filmWidth, filmHeight);
		//normalFilter.commit();

		////// Prefilter the auxiliary images
		//albedoFilter.execute();
		//normalFilter.execute();

		//// Filter the beauty image
		//filter.execute();

		//float* outputPtr = (float*)outputBuf.getData();

		//memcpy(film->film, outputPtr, dataSize);


		nextTile = 0; //reset atomic
		auto Render = [&]() {
			while (true) {
				int tileIndex = nextTile.fetch_add(1);
				if (tileIndex >= totalTiles)
					break;
				int tileX = tileIndex % numTilesX;
				int tileY = tileIndex / numTilesX;
				renderTileBased(tileX, tileY, tileSize, filmWidth, filmHeight);
			}
			};

		for (int i = 0; i < numCore; i++) {
			threads.emplace_back(Render);
		}
		for (auto& t : threads) {
			if (t.joinable())
				t.join();
		}
	}

	void renderTileBasedAdaptive(int tileX, int tileY, int tileSize, int filmWidth, int filmHeight) {
		int startX = tileX * tileSize;
		int startY = tileY * tileSize;
		int endX = min(startX + tileSize, filmWidth);
		int endY = min(startY + tileSize, filmHeight);
		int numTilesX = (filmWidth + tileSize - 1) / tileSize;


		for (int y = startY; y < endY; y++) {
			for (int x = startX; x < endX; x++) {

				unsigned char r, g, b;
				film->tonemapSPP(x, y, r, g, b, tileSPPGlobal[tileY * numTilesX + tileX]);

				canvas->draw(x, y, r, g, b);
			}
		}
	}
	void splatTileBasedAdaptive(int tileX, int tileY, int tileSize, int filmWidth, int filmHeight, int SPP) {
		int startX = tileX * tileSize;
		int startY = tileY * tileSize;
		int endX = min(startX + tileSize, filmWidth);
		int endY = min(startY + tileSize, filmHeight);

		for (int i = 0; i < SPP; ++i) {
			for (int y = startY; y < endY; y++) {
				for (int x = startX; x < endX; x++) {
					float px = x + samplers->next();
					float py = y + samplers->next();
					Ray ray = scene->camera.generateRay(px, py);

					//Colour col = direct(ray, samplers);

					Colour pathThroughput(1.0f, 1.0f, 1.0f);
					Colour col = pathTrace(ray, pathThroughput, 0, samplers);

					ShadingData temp;

					//Colour col = directInstantRadiosity(ray);

					film->splat(px, py, col);

				}
			}
		}
	}
	float getBlockVariance(int tileX, int tileY, int tileSize) {
		unsigned int startX = tileX * tileSize;
		unsigned int startY = tileY * tileSize;
		unsigned int width = film->width;
		unsigned int height = film->height;
		int endX = min(startX + tileSize, width);
		int endY = min(startY + tileSize, height);
		float sum = 0.0f;
		int count = 0;
		for (int y = startY; y < endY; y++) {
			for (int x = startX; x < endX; x++) {
				float lum = film->film[y * width + x].Lum();
				sum += lum;
				count++;
			}
		}
		if (count <= 1) return 0.0f;
		float mean = sum / count;

		float sumTerm = 0;
		for (int y = startY; y < endY; y++) {
			for (int x = startX; x < endX; x++) {
				float temp = mean - film->film[y * width + x].Lum();
				sumTerm += temp * temp;
			}
		}
		return sumTerm / (count - 1);
	}

	void render() {

		int initialSPP = 4;

		const int filmWidth = film->width;
		const int filmHeight = film->height;

		int numCore = std::thread::hardware_concurrency();

		const int tileSize = 16;

		int numTilesX = (filmWidth + tileSize - 1) / tileSize;
		int numTilesY = (filmHeight + tileSize - 1) / tileSize;
		int totalTiles = numTilesX * numTilesY;

		// atomic counter
		std::atomic<int> nextTile(0);

		std::vector<std::thread> threads;
		threads.reserve(numCore);

		// multi-thread for splatting initial
		auto Splat = [&]() {
			while (true) {
				int tileIndex = nextTile.fetch_add(1);
				if (tileIndex >= totalTiles)
					break;
				int tileX = tileIndex % numTilesX;
				int tileY = tileIndex / numTilesX;
				splatTileBasedAdaptive(tileX, tileY, tileSize, filmWidth, filmHeight, initialSPP);
			}
			};
		for (int i = 0; i < numCore; i++) {
			threads.emplace_back(Splat);
		}
		for (auto& t : threads) {
			if (t.joinable())
				t.join();
		}
		threads.clear();

		nextTile = 0;

		// Get variance for block
		std::vector<float> variances(totalTiles, 0.0f);
		float totalVariance = 0.0f;
		for (int tileY = 0; tileY < numTilesY; tileY++) {
			for (int tileX = 0; tileX < numTilesX; tileX++) {
				float temp = getBlockVariance(tileX, tileY, tileSize);
				variances[tileY * numTilesX + tileX] = temp;
				totalVariance += temp;
			}
		}

		if (totalVariance <= 0.0f) {
			return;
		}

		// allocate the SPP
		const int totalSPP = totalTiles * initialSPP;
		std::vector<int>tileSPP(totalTiles, 0);
		for (int tileY = 0; tileY < numTilesY; tileY++) {
			for (int tileX = 0; tileX < numTilesX; tileX++) {
				float wi = variances[tileY * numTilesX + tileX] / totalVariance;
				int temp = static_cast<int>(std::ceil(wi * totalSPP)) + initialSPP;
				tileSPP[tileY * numTilesX + tileX] = temp;
				tileSPPGlobal[tileY * numTilesX + tileX] += temp;
			}
		}

		// atomic counter
		nextTile = 0;

		// multi-thread for splatting initial
		auto Splat2 = [&]() {
			while (true) {
				int tileIndex = nextTile.fetch_add(1);
				if (tileIndex >= totalTiles)
					break;
				int tileX = tileIndex % numTilesX;
				int tileY = tileIndex / numTilesX;
				splatTileBasedAdaptive(tileX, tileY, tileSize, filmWidth, filmHeight, tileSPP[tileY * numTilesX + tileX]);
			}
			};
		for (int i = 0; i < numCore; i++) {
			threads.emplace_back(Splat2);
		}
		for (auto& t : threads) {
			if (t.joinable())
				t.join();
		}
		threads.clear();

		nextTile = 0; //reset atomic
		auto Render = [&]() {
			while (true) {
				int tileIndex = nextTile.fetch_add(1);
				if (tileIndex >= totalTiles)
					break;
				int tileX = tileIndex % numTilesX;
				int tileY = tileIndex / numTilesX;
				renderTileBasedAdaptive(tileX, tileY, tileSize, filmWidth, filmHeight);
			}
			};

		for (int i = 0; i < numCore; i++) {
			threads.emplace_back(Render);
		}
		for (auto& t : threads) {
			if (t.joinable())
				t.join();
		}
	}



	void renderOld()
	{
		film->incrementSPP();
		for (unsigned int y = 0; y < film->height; y++)
		{
			for (unsigned int x = 0; x < film->width; x++)
			{
				float px = x + 0.5f;
				float py = y + 0.5f;
				Ray ray = scene->camera.generateRay(px, py);
				Colour col = direct(ray, samplers);
				//Colour col = viewNormals(ray);
				//Colour col = albedo(ray);

				film->splat(px, py, col);
				unsigned char r = (unsigned char)(col.r * 255);
				unsigned char g = (unsigned char)(col.g * 255);
				unsigned char b = (unsigned char)(col.b * 255);
				film->tonemap(x, y, r, g, b);

				canvas->draw(x, y, r, g, b);
			}
		}
	}

	int getSPP()
	{
		return film->SPP;
	}
	void saveHDR(std::string filename)
	{
		film->save(filename);
	}
	void savePNG(std::string filename)
	{
		stbi_write_png(filename.c_str(), canvas->getWidth(), canvas->getHeight(), 3, canvas->getBackBuffer(), canvas->getWidth() * 3);
	}
};