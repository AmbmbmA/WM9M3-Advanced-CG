#pragma once

#include "Core.h"
#include "Sampling.h"

class Ray
{
public:
	Vec3 o;
	Vec3 dir;
	Vec3 invDir;
	Ray()
	{
	}
	Ray(Vec3 _o, Vec3 _d)
	{
		init(_o, _d);
	}
	void init(Vec3 _o, Vec3 _d)
	{
		o = _o;
		dir = _d;
		invDir = Vec3(1.0f / dir.x, 1.0f / dir.y, 1.0f / dir.z);
	}
	Vec3 at(const float t) const
	{
		return (o + (dir * t));
	}
};

class Plane
{
public:
	Vec3 n;
	float d;
	void init(Vec3& _n, float _d)
	{
		n = _n;
		d = _d;
	}
	// Add code here
	bool rayIntersect(Ray& r, float& t)
	{

		// Plane normal vertical to ray direction hence ray parallel to plane
		if (n.dot(r.dir) == 0) return false;

		// calculate the distance
		t = (d - n.dot(r.o)) / n.dot(r.dir);


		// if in the front
		if (t >= 0) {
			return true;
		}

		return false;

	}


};


void testRayPlaneIntersect() {
	Vec3 raydir = Vec3(0, -1, 0);
	Ray testTay(Vec3(0, 10, 0), raydir);
	Plane testPlane;
	Vec3 testNormal = Vec3(0, 1, 0);
	testPlane.init(testNormal, 0);
	float t = 0.0f;
	if (testPlane.rayIntersect(testTay, t)) {
		std::cout << "Ray Plane Intersect correct!" << std::endl;
		std::cout << "Distance = " << t << std::endl;
		// t = 10;

	}
	else {
		std::cout << "Ray Plane Intersect Wrong!" << std::endl;
	}
}


//#define EPSILON 0.001f
#define EPSILON 1e-7f

class Triangle
{
public:
	Vertex vertices[3];
	Vec3 e1; // Edge 1
	Vec3 e2; // Edge 2
	Vec3 n; // Geometric Normal
	float area; // Triangle area
	float d; // For ray triangle if needed
	unsigned int materialIndex;
	void init(Vertex v0, Vertex v1, Vertex v2, unsigned int _materialIndex)
	{
		materialIndex = _materialIndex;
		vertices[0] = v0;
		vertices[1] = v1;
		vertices[2] = v2;
		e1 = vertices[2].p - vertices[1].p;
		e2 = vertices[0].p - vertices[2].p;
		n = e1.cross(e2).normalize();
		area = e1.cross(e2).length() * 0.5f;
		d = Dot(n, vertices[0].p);
	}
	Vec3 centre() const
	{
		return (vertices[0].p + vertices[1].p + vertices[2].p) / 3.0f;
	}
	// Add code here
	bool rayIntersectBasic(const Ray& r, float& t, float& u, float& v) const
	{
		// normal dot ray dir
		float dotnodir = n.dot(r.dir);

		// parallel
		if (dotnodir == 0) return false;

		t = (d - n.dot(r.o)) / dotnodir;

		// if behind
		if (t < 0) return false;

		float invA = 1.0f / e1.cross(e2).dot(n);

		Vec3 q1 = (r.o + r.dir * t) - vertices[1].p;
		u = e1.cross(q1).dot(n) * invA;
		if (u < 0 || u > 1.0f) return false;

		Vec3 q2 = (r.o + r.dir * t) - vertices[2].p;
		v = e2.cross(q2).dot(n) * invA;
		if (v < 0 || (u + v) > 1.0f) return false;

		return true;

	}

	bool rayIntersectMollerTrumbore(const Ray& r, float& t, float& u, float& v) const
	{
		Vec3 e1m = vertices[0].p - vertices[2].p;
		Vec3 e2m = vertices[1].p - vertices[2].p;
		Vec3 v0 = vertices[2].p;

		Vec3 T = r.o - v0;
		Vec3 d = r.dir;

		Vec3 p = d.cross(e2m);
		float det = e1m.dot(p);
		// parallel
		if (fabs(det) < 1e-7f) return false;

		float invdet = 1.0f / det;

		u = T.dot(p) * invdet;
		if (u < 0 || u > 1.0f) return false;

		Vec3 q = T.cross(e1m);
		v = d.dot(q) * invdet;
		if (v < 0 || (u + v) > 1.0f) return false;

		t = e2m.dot(q) * invdet;

		// if behind
		if (t < 0) return false;

		return true;

	}

	void interpolateAttributes(const float alpha, const float beta, const float gamma, Vec3& interpolatedNormal, float& interpolatedU, float& interpolatedV) const
	{
		interpolatedNormal = vertices[0].normal * alpha + vertices[1].normal * beta + vertices[2].normal * gamma;
		interpolatedNormal = interpolatedNormal.normalize();
		interpolatedU = vertices[0].u * alpha + vertices[1].u * beta + vertices[2].u * gamma;
		interpolatedV = vertices[0].v * alpha + vertices[1].v * beta + vertices[2].v * gamma;
	}

	// Add code here
	Vec3 sample(Sampler* sampler, float& pdf)
	{
		float r1 = sampler->next();
		float r2 = sampler->next();

		pdf = 1.0f / area;

		float alpha = 1 - sqrtf(r1);
		float beta = r2 * sqrtf(r1);
		float gamma = 1 - alpha - beta;

		Vec3 result = vertices[0].p * alpha + vertices[1].p * beta + vertices[2].p * gamma;

		return result;

	}
	Vec3 gNormal()
	{
		return (n * (Dot(vertices[0].normal, n) > 0 ? 1.0f : -1.0f));
	}
};

void testRayTriBasicIntersect() {
	Vec3 raydir = Vec3(0, -1, 0);
	Ray testTay(Vec3(0, 10, 0), raydir);
	Triangle testTri;
	Vertex v0, v1, v2;
	v0.p = Vec3(-1, 0, 0);
	v1.p = Vec3(1, 1, 0);
	v2.p = Vec3(0, 0, 1);

	testTri.init(v0, v1, v2, 0);
	float t = 0.0f;
	float u = 0.0f;
	float v = 0.0f;

	if (testTri.rayIntersectBasic(testTay, t, u, v)) {
		std::cout << "Ray Triangle Basic Intersect correct!" << std::endl;
		std::cout << "Distance = " << t << std::endl;
		std::cout << "u = " << u << std::endl;
		std::cout << "v = " << v << std::endl;
		// t = 9.5
	}
	else {
		std::cout << "Ray Triangle Basic Intersect Wrong!" << std::endl;
	}
}

void testRayTriMollerTrumboreIntersect() {
	Vec3 raydir = Vec3(0, -1, 0);
	Ray testTay(Vec3(0, 10, 0), raydir);
	Triangle testTri;
	Vertex v0, v1, v2;
	v0.p = Vec3(-1, 0, 0);
	v1.p = Vec3(1, 1, 0);
	v2.p = Vec3(0, 0, 1);

	testTri.init(v0, v1, v2, 0);
	float t = 0.0f;
	float u = 0.0f;
	float v = 0.0f;

	if (testTri.rayIntersectMollerTrumbore(testTay, t, u, v)) {
		std::cout << "Ray Triangle Moller-Trumbore Intersect correct!" << std::endl;
		std::cout << "Distance = " << t << std::endl;
		std::cout << "u = " << u << std::endl;
		std::cout << "v = " << v << std::endl;
		// t = 9.5
	}
	else {
		std::cout << "Ray Triangle Moller-Trumbore Intersect Wrong!" << std::endl;
	}
}

class AABB
{
public:
	Vec3 max;
	Vec3 min;
	AABB()
	{
		reset();
	}
	AABB(const Vec3& maxPoint, const Vec3& minPoint)
	{
		max = maxPoint;
		min = minPoint;
	}
	void reset()
	{
		max = Vec3(-FLT_MAX, -FLT_MAX, -FLT_MAX);
		min = Vec3(FLT_MAX, FLT_MAX, FLT_MAX);
	}
	void extend(const Vec3 p)
	{
		max = Max(max, p);
		min = Min(min, p);
	}
	// Add code here
	bool rayAABB(const Ray& r, float& t)
	{
		float x = INFINITY;
		float y = INFINITY;
		float z = INFINITY;
		// prevent parallel division
		if (fabs(r.dir.x) > EPSILON) {
			x = 1.0f / r.dir.x;
		}
		else {
			if (r.o.x < min.x || r.o.x > max.x) {
				return false;
			}
		}
		if (fabs(r.dir.y) > EPSILON) {
			y = 1.0f / r.dir.y;
		}
		else {
			if (r.o.y < min.y || r.o.y > max.y) {
				return false;
			}
		}
		if (fabs(r.dir.z) > EPSILON) {
			z = 1.0f / r.dir.z;
		}
		else {
			if (r.o.z < min.z || r.o.z > max.z) {
				return false;
			}
		}

		Vec3 dinv = Vec3(x, y, z);
		//Vec3 dinv = Vec3(1.0f / r.dir.x, 1.0f / r.dir.y, 1.0f / r.dir.z);
		Vec3 tmin = (min - r.o) * dinv;
		Vec3 tmax = (max - r.o) * dinv;

		Vec3 tentry = Min(tmin, tmax);
		Vec3 texit = Max(tmin, tmax);

		float tentryf = std::max(tentry.x, std::max(tentry.y, tentry.z));
		float texitf = std::min(texit.x, std::min(texit.y, texit.z));

		if (texitf >= 0 && tentryf <= texitf) {
			if (tentryf < 0) {
				t = 0;
			}
			else {
				t = tentryf;
			}

			return true;
		}

		return false;
	}

	// Add code here
	bool rayAABB(const Ray& r)
	{
		float x = INFINITY;
		float y = INFINITY;
		float z = INFINITY;
		// prevent parallel division
		if (fabs(r.dir.x) > EPSILON) {
			x = 1.0f / r.dir.x;
		}
		else {
			if (r.o.x < min.x || r.o.x > max.x) {
				return false;
			}
		}
		if (fabs(r.dir.y) > EPSILON) {
			y = 1.0f / r.dir.y;
		}
		else {
			if (r.o.y < min.y || r.o.y > max.y) {
				return false;
			}
		}
		if (fabs(r.dir.z) > EPSILON) {
			z = 1.0f / r.dir.z;
		}
		else {
			if (r.o.z < min.z || r.o.z > max.z) {
				return false;
			}
		}

		Vec3 dinv = Vec3(x, y, z);
		//Vec3 dinv = Vec3(1.0f / r.dir.x, 1.0f / r.dir.y, 1.0f / r.dir.z);
		Vec3 tmin = (min - r.o) * dinv;
		Vec3 tmax = (max - r.o) * dinv;

		Vec3 tentry = Min(tmin, tmax);
		Vec3 texit = Max(tmin, tmax);

		float tentryf = std::max(tentry.x, std::max(tentry.y, tentry.z));
		float texitf = std::min(texit.x, std::min(texit.y, texit.z));

		if (texitf >= 0 && tentryf <= texitf) {
			//if (tentryf < 0) { }
			return true;
		}

		return false;
	}

	// Add code here
	float area()
	{
		Vec3 size = max - min;
		return ((size.x * size.y) + (size.y * size.z) + (size.x * size.z)) * 2.0f;
	}

};

void testRayAABB() {
	Vec3 raydir = Vec3(0, -1, 0);
	Ray testTay(Vec3(0, 10, 0), raydir);
	AABB testBox(Vec3(1, 1, 1), Vec3(-1, -1, -1));
	float t = 0.0f;

	if (testBox.rayAABB(testTay, t) && testBox.rayAABB(testTay)) {
		std::cout << "Ray AABB Intersect correct!" << std::endl;
		std::cout << "Distance = " << t << std::endl;
		// t = 9
	}
	else {
		std::cout << "Ray AABB Intersect Wrong!" << std::endl;
	}

	if (testBox.area() == 24) {
		std::cout << "AABB area correct!" << std::endl;
	}
	else {
		std::cout << "AABB area wrong!" << std::endl;
	}
}

class Sphere
{
public:
	Vec3 centre;
	float radius;
	void init(Vec3& _centre, float _radius)
	{
		centre = _centre;
		radius = _radius;
	}
	// Add code here
	bool rayIntersect(Ray& r, float& t)
	{

		Vec3 l = r.o - centre;
		float b = l.dot(r.dir);
		float c = l.dot(l) - radius * radius;
		float dis = b * b - c;
		if (dis < 0) {
			return false;
		}
		else if (dis == 0) {
			t = -b;
			return true;
		}
		else {
			float t1 = -b - sqrtf(dis);
			float t2 = -b + sqrtf(dis);

			if (t2 < 0) {
				return false;
			}
			else {
				if (t1 > 0) {
					t = t1;
				}
				else {
					t = 0;
				}
				return true;
			}

		}

	}
};

void testRaySphere() {
	Vec3 raydir = Vec3(0, -1, 0);
	Ray testTay(Vec3(0, 10, 0), raydir.normalize());
	Sphere testSphere;
	Vec3 center = Vec3(0, 0, 0);
	testSphere.init(center, 2);

	float t = 0.0f;

	if (testSphere.rayIntersect(testTay, t)) {
		std::cout << "Ray Sphere Intersect correct!" << std::endl;
		std::cout << "Distance = " << t << std::endl;
		// t = 8
	}
	else {
		std::cout << "Ray Sphere Intersect Wrong!" << std::endl;
	}

}


struct IntersectionData
{
	unsigned int ID;
	float t;
	float alpha;
	float beta;
	float gamma;
};

#define MAXNODE_TRIANGLES 8
#define TRAVERSE_COST 1.0f
#define TRIANGLE_COST 2.0f
#define BUILD_BINS 32

#define BUILD_DEPTH 32

class BVHNode
{
public:
	AABB bounds;
	BVHNode* r; // left sub node
	BVHNode* l; // rigth sub node
	// This can store an offset and number of triangles in a global triangle list for example
	// But you can store this however you want!
	unsigned int startIndex;
	unsigned int triNum;

	BVHNode()
	{
		r = NULL;
		l = NULL;
		startIndex = 0;
		triNum = 0;
	}
	// Note there are several options for how to implement the build method. Update this as required
	void build(std::vector<Triangle>& inputTriangles, std::vector<unsigned int>& triIndex, unsigned int start, unsigned int end, unsigned int depth)
	{
		// Add BVH building code here
		startIndex = start;
		triNum = end - start;


		// get the box for this node
		bounds.reset();
		for (unsigned int i = start; i < end; i++) {
			unsigned int triId = triIndex[i];
			bounds.extend(inputTriangles[triId].vertices[0].p);
			bounds.extend(inputTriangles[triId].vertices[1].p);
			bounds.extend(inputTriangles[triId].vertices[2].p);
		}

		// check if this is a leaf by triNum or depth
		if (triNum <= MAXNODE_TRIANGLES || depth > BUILD_DEPTH) {
			l = nullptr;
			r = nullptr;
			return;
		}


		float minCost = FLT_MAX; // record the minimum cost
		int optAxis = -1; // the optimised axis to cut
		// {0,1,2} -> {x,y,z}
		float optPos = 0.0f; // the optimised position to cut

		Vec3 boxSize = bounds.max - bounds.min;

		// go through x,y,z axises to update cut with lower cost
		for (unsigned int axis = 0; axis < 3; axis++) {
			int step = BUILD_BINS;
			float minPos = bounds.min.coords[axis];
			float maxPos = bounds.max.coords[axis];
			float range = maxPos - minPos;

			// if the axis size is too small ignore
			if (fabs(range) < EPSILON) continue;

			// go through all stide ( 1 to step-1)
			for (int s = 1; s < step; s++) {
				float frac = float(s) / float(step);
				float cutPos = minPos + range * frac;

				// cut left and rigth box, count
				AABB leftBounds, rightBounds;
				leftBounds.reset();
				rightBounds.reset();
				int leftTriNum = 0;
				int rightTriNum = 0;
				for (unsigned int i = start; i < end; i++) {
					unsigned int triId = triIndex[i];
					Vec3 center = inputTriangles[triId].centre();
					if (center.coords[axis] < cutPos) {
						leftTriNum++;
						leftBounds.extend(inputTriangles[triId].vertices[0].p);
						leftBounds.extend(inputTriangles[triId].vertices[1].p);
						leftBounds.extend(inputTriangles[triId].vertices[2].p);
					}
					else {
						rightTriNum++;
						rightBounds.extend(inputTriangles[triId].vertices[0].p);
						rightBounds.extend(inputTriangles[triId].vertices[1].p);
						rightBounds.extend(inputTriangles[triId].vertices[2].p);
					}
				}

				// if either box is empty, skip this stride
				if (rightTriNum == 0 || leftTriNum == 0) continue;

				// calculate the cost
				float parentAreaInv = 1.0f / bounds.area();
				float leftArea = leftBounds.area();
				float rightArea = rightBounds.area();

				float cost = TRAVERSE_COST + parentAreaInv * TRIANGLE_COST * (leftArea * leftTriNum + rightArea * rightTriNum);

				// update
				if (cost < minCost) {
					minCost = cost;
					optAxis = axis;
					optPos = cutPos;
				}
			}

		}

		// if not found a cut with lower cost, stop
		if (optAxis < 0) {
			l = nullptr;
			r = nullptr;
			return;
		}

		// seperate the triangles

		unsigned int middle = start;
		for (unsigned int i = start; i < end; i++) {
			unsigned int triId = triIndex[i];
			Vec3 center = inputTriangles[triId].centre();
			// if center at the cut axis is small than the cut position -> left
			if (center.coords[optAxis] < optPos) {
				// swap it to the left
				std::swap(triIndex[middle], triIndex[i]);
				middle++;
			}
		}

		// empty cut
		if (middle == start || middle == end) {
			l = nullptr;
			r = nullptr;
			return;
		}

		//std::cout << "BVH depth = " << depth << std::endl;


		// Recursion build left and right

		l = new BVHNode();
		l->build(inputTriangles, triIndex, start, middle, depth + 1);

		r = new BVHNode();
		r->build(inputTriangles, triIndex, middle, end, depth + 1);
	}

	void traverse(const Ray& ray, const std::vector<Triangle>& triangles, std::vector<unsigned int>& triIndex, IntersectionData& intersection)
	{
		// Add BVH Traversal code here

		float tbox;

		// if intersect the node
		if (!bounds.rayAABB(ray, tbox)) return;

		// if intersect close than t, then no updation
		if (tbox >= intersection.t) return;

		// if leaf
		if (!l && !r) {

			// go through all triangles
			for (unsigned int i = 0; i < triNum; i++) {
				unsigned int triId = triIndex[startIndex + i];
				float t;
				float u;
				float v;
				if (triangles[triId].rayIntersectMollerTrumbore(ray, t, u, v))
				{
					if (t < intersection.t)
					{
						intersection.t = t;
						intersection.ID = triId;
						intersection.alpha = u;
						intersection.beta = v;
						intersection.gamma = 1.0f - (u + v);
					}
				}
			
			}
			return;
		}

		if (l) l->traverse(ray, triangles, triIndex,intersection);
		if (r) r->traverse(ray, triangles, triIndex,intersection);

	}

	IntersectionData traverse(const Ray& ray, const std::vector<Triangle>& triangles, std::vector<unsigned int>& triIndex)
	{
		IntersectionData intersection;
		intersection.t = FLT_MAX;
		traverse(ray, triangles, triIndex, intersection);
		return intersection;
	}


	bool traverseVisible(const Ray& ray, const std::vector<Triangle>& triangles, const std::vector<unsigned int>& triIndex,const float maxT)
	{
		//return true;
		// Add visibility code here
		float tbox;

		// if intersect the node or too far
		if (!bounds.rayAABB(ray, tbox) || tbox > maxT) return true;

		// if leaf
		if (!l && !r) {

			// go through all triangles
			for (unsigned int i = 0; i < triNum; i++) {
				unsigned int triId = triIndex[startIndex + i];
				float t;
				float u;
				float v;
				if (triangles[triId].rayIntersectMollerTrumbore(ray, t, u, v))
				{
					if (t < maxT && t >= 0.0f)
					{
						return false;
					}
				}

			}
			return true;
		}

		if (l && !l->traverseVisible(ray, triangles, triIndex, maxT)) return false;
		if (r && !r->traverseVisible(ray, triangles, triIndex, maxT)) return false;

		return true;
	}

};
