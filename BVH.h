#ifndef MIRO_ACCEL_H_INCLUDED
#define MIRO_ACCEL_H_INCLUDED

#include "Miro.h"
#include "Object.h"



class BVH
{
public:
	static long long num_nodes;
	static long long num_leaves;
	static long long num_rays;
	static long long num_ray_box;
	static long long num_ray_tri;

	~BVH();

	void build(Objects* objs);
	bool intersect(HitInfo& result, const Ray& ray,
		float tMin = 0.0f, float tMax = MIRO_TMAX) const;
	bool intersect_bvh(HitInfo& result, const Ray& ray,
		float tMin = 0.0f, float tMax = MIRO_TMAX) const;

protected:

	static const int NUM_SPLITS = 5;
	static const int NUM_TRI_IN_LEAF = 8;
	static const int X_AXIS = 0;
	static const int Y_AXIS = 1;
	static const int Z_AXIS = 2;

	void build(std::vector<Bbox*>& bboxes);
	int getSplitPlane(std::vector<Bbox*>& bboxes, float& minVal, float& maxVal);
	void getMinMax(std::vector<Bbox*>& bboxes, int axis, Vector3& minVal, Vector3& maxVal);
	int getSplitIndex(float val, float minVal, float splitLength, int axis);

	Bbox* m_bbox;
	std::vector<BVH*> m_children;
};


struct Bbox
{
	Vector3 min = Vector3(MIRO_TMAX, MIRO_TMAX, MIRO_TMAX);
	Vector3 max = Vector3(-MIRO_TMAX, -MIRO_TMAX, -MIRO_TMAX);
	std::vector<Object*> object;

	void addObject(Object* obj);
	bool hit(const Ray& ray);
	bool hit(const Ray& ray, float& tMin, float& tMax);
	Vector3 hitHelper(const Ray& ray, const Vector3& v);
};

class BVHHelper
{
public:
	int num_objs = 0;
	float sa = 0.0f;
	Vector3 min = Vector3(MIRO_TMAX, MIRO_TMAX, MIRO_TMAX);
	Vector3 max = Vector3(-MIRO_TMAX, -MIRO_TMAX, -MIRO_TMAX);

	void update(Bbox* box);
};
#endif // MIRO_ACCEL_H_INCLUDED
