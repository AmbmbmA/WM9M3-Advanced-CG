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

class RayTracer
{
public:
	Scene* scene;
	GamesEngineeringBase::Window* canvas;
	Film* film;
	MTRandom* samplers;
	std::thread** threads;
	int numProcs;
	Colour* TileBasedBuffer;

	void init(Scene* _scene, GamesEngineeringBase::Window* _canvas)
	{
		scene = _scene;
		canvas = _canvas;
		film = new Film();
		film->init((unsigned int)scene->camera.width, (unsigned int)scene->camera.height, new MitchellNetravaliFilter());
		SYSTEM_INFO sysInfo;
		GetSystemInfo(&sysInfo);
		numProcs = sysInfo.dwNumberOfProcessors;
		threads = new std::thread * [numProcs];
		samplers = new MTRandom[numProcs];
		TileBasedBuffer = new Colour[(unsigned int)scene->camera.width * (unsigned int)scene->camera.height];
		clear();
	}
	void clear()
	{
		film->clear();
	}
	Colour computeDirect(ShadingData shadingData, Sampler* sampler)
	{
		// Is surface is specular we cannot computing direct lighting
		if (shadingData.bsdf->isPureSpecular() == true)
		{
			return Colour(0.0f, 0.0f, 0.0f);
		}
		// Compute direct lighting here
		return Colour(0.0f, 0.0f, 0.0f);
	}
	Colour pathTrace(Ray& r, Colour& pathThroughput, int depth, Sampler* sampler)
	{
		// Add pathtracer code here
		return Colour(0.0f, 0.0f, 0.0f);
	}
	Colour direct(Ray& r, Sampler* sampler)
	{
		// Compute direct lighting for an image sampler here
		return Colour(0.0f, 0.0f, 0.0f);
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
		return scene->background->evaluate(shadingData, r.dir);
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
				Colour col = TileBasedBuffer[y * filmWidth + x];
				unsigned char r = static_cast<unsigned char>(col.r * 255);
				unsigned char g = static_cast<unsigned char>(col.g * 255);
				unsigned char b = static_cast<unsigned char>(col.b * 255);
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
				float px = x + 0.5f;
				float py = y + 0.5f;
				Ray ray = scene->camera.generateRay(px, py);
				Colour col = viewNormals(ray);
				//Colour col = albedo(ray);

				film->splat(px, py, col);
				TileBasedBuffer[y * filmWidth + x] = col;
			}
		}
	}

	void render()
	{
		film->incrementSPP();

		const int filmWidth = film->width;
		const int filmHeight = film->height;

		int numCore = std::thread::hardware_concurrency();
		const int tileSize = sqrtf(filmWidth* filmHeight / numCore);
		//const int tileSize = 256;

		int numTilesX = (filmWidth + tileSize - 1) / tileSize;
		int numTilesY = (filmHeight + tileSize - 1) / tileSize;

		std::vector<std::thread> threads;
		threads.reserve(numTilesX* numTilesY);

		// multi-thread for splatting
		for (int tileY = 0; tileY < numTilesY; tileY++)
		{
			for (int tileX = 0; tileX < numTilesX; tileX++)
			{
				threads.emplace_back(&RayTracer::splatTileBased, this, tileX, tileY, tileSize, filmWidth, filmHeight);
			}
		}

		for (auto& t : threads)
		{
			if (t.joinable())
				t.join();
		}

		threads.clear();

		// multi thread for render
		for (int tileY = 0; tileY < numTilesY; tileY++)
		{
			for (int tileX = 0; tileX < numTilesX; tileX++)
			{
				threads.emplace_back(&RayTracer::renderTileBased, this, tileX, tileY, tileSize, filmWidth, filmHeight);
			}
		}
		for (auto& t : threads)
		{
			if (t.joinable())
				t.join();
		}

	}

	void renderold()
	{
		film->incrementSPP();
		for (unsigned int y = 0; y < film->height; y++)
		{
			for (unsigned int x = 0; x < film->width; x++)
			{
				float px = x + 0.5f;
				float py = y + 0.5f;
				Ray ray = scene->camera.generateRay(px, py);
				Colour col = viewNormals(ray);
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