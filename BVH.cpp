#include "BVH.h"
#include "Ray.h"
#include "Console.h"
#include <math.h>
#include <algorithm>

long long BVH::num_nodes = 0;
long long BVH::num_leaves = 0;
long long BVH::num_rays = 0;
long long BVH::num_ray_box = 0;
long long BVH::num_ray_tri = 0;



void
BVH::build(Objects* objs)//For *objs[i], bboxes[i]->addObject((*objs)[i]); m_bbox->addObject((*objs)[i]);build(bboxes);

{
	if (objs == NULL) {
		std::cerr << "(BVH::build) objs is nullpointer" << std::endl;
		return;
	}

	int size = objs->size();
	std::vector<BVHBox*> bboxes;
	m_bbox = new BVHBox();			// Root

	for (int i = 0; i < size; i++) {
		bboxes.push_back(new BVHBox());

		bboxes[i]->addObject((*objs)[i]);
		m_bbox->addObject((*objs)[i]);
	}

	build(bboxes);

}

//overload
void
BVH::build(std::vector<BVHBox*>& bboxes)
{
	num_nodes++;
	int num = bboxes.size();

	if (num == 0) {
		std::cerr << "(BVH::build) bboxes has size 0" << std::endl;
		return;
	}

	// Make leaf node if num less than Num_TRI_IN_LEAF=8
	if (num <= NUM_TRI_IN_LEAF) {
		num_leaves++;
	
		m_bbox = new BVHBox();



		for (int i = 0; i < num; i++) {
			m_bbox->addObject(bboxes[i]->object[0]);
		}



		for (int i = 0; i < num; i++) {
			delete bboxes[i];
		}
		return;
	}

	// Get split axis based on objects' center NUM-SPLITS=5
	Spliter splits[NUM_SPLITS];



	Vector3 best_minVal, best_maxVal;
	int best_axis = -1;
	float best_min_cost = MIRO_TMAX;
	int best_plane = -1;
	float best_splitLength;

#pragma omp parallel for simd
	for (int axis = 0; axis < 3; axis++) {
		Vector3 minVal, maxVal;
		getMinMax(bboxes, axis, minVal, maxVal);//updating minVal and maxVal
		float splitLength = (maxVal[axis] - minVal[axis]) / ((float)NUM_SPLITS);
		int tmp_best_plane = -1;

		// initialize
		for (int i = 0; i < NUM_SPLITS; i++) {
			splits[i] = Spliter();
		}

		// Put bbox (objects) in the right box split
		for (int i = 0; i < num; i++) {
			if (bboxes[i]->object.size() > 1) {
				std::cerr << "(BVH::build) bboxes[i]->object has size > 1" << std::endl;
				return;
			}
			float boxSplitVal = bboxes[i]->object[0]->center()[axis];
			int index = getSplitIndex(boxSplitVal, minVal[axis], splitLength, axis);

			if (index >= NUM_SPLITS) {

				std::cerr << "(BVH::build) index > NUM_SPLITS (1)" << std::endl;

			}
			splits[index].update(bboxes[i]);
		}


		float l = (maxVal[axis] - minVal[axis]) / ((float)NUM_SPLITS);
		float w = maxVal[(axis + 1) % 3] - minVal[(axis + 1) % 3];
		float h = maxVal[(axis + 2) % 3] - minVal[(axis + 2) % 3];

		// Get the plane with minimum cost
		float min_cost = MIRO_TMAX;

		for (int i = 1; i < NUM_SPLITS; i++) {
			float l_sa = 0.0f, r_sa = 0.0f;
			int l_num = 0, r_num = 0;
			float left_side = 0.0f, right_side = 0.0f;

			// left side
			for (int j = 0; j < i; j++) {

				l_num += splits[j].num_objs;
			}

			// right side
			for (int j = i; j < NUM_SPLITS; j++) {

				r_num += splits[j].num_objs;
			}



			l_sa = l * i;
			r_sa = l * (NUM_SPLITS - i);
			left_side = l_sa/l;
			right_side = r_sa/l;
			//traversal cost ignored
			float cost = left_side * l_num + right_side * r_num;

			if (cost < min_cost) {
				min_cost = cost;
				tmp_best_plane = i;
			}
		}

		if (min_cost < best_min_cost) {
			best_min_cost = min_cost;
			best_plane = tmp_best_plane;
			best_axis = axis;
			best_minVal = minVal;
			best_maxVal = maxVal;
			best_splitLength = splitLength;
		}
	}

	// Put bbox (objects) back to the left or right split
	std::vector<BVHBox*> leftBoxes; 
	std::vector<BVHBox*> rightBoxes;
	BVHBox* lBox = new BVHBox();
	BVHBox* rBox = new BVHBox();

	int l_index = 0, r_index = 0;

#pragma omp parallel for simd
	for (int i = 0; i < num; i++) {
		float boxSplitVal = bboxes[i]->object[0]->center()[best_axis];
		int index = getSplitIndex(boxSplitVal, best_minVal[best_axis], best_splitLength, best_axis);
		float boundary = m_bbox->min[best_axis] + (best_plane * best_splitLength);

		if (index < best_plane) {
			//leftBoxes[l_index++] = *bboxes[i];
			leftBoxes.push_back(bboxes[i]);
			lBox->addObject(bboxes[i]->object[0]);
		}
		else {
			//rightBoxes[r_index++] = *bboxes[i];
			rightBoxes.push_back(bboxes[i]);
			rBox->addObject(bboxes[i]->object[0]);
		}
	}



	BVH* leftChild = new BVH();
	BVH* rightChild = new BVH();

	leftChild->m_bbox = lBox;
	rightChild->m_bbox = rBox;

	leftChild->build(leftBoxes);
	rightChild->build(rightBoxes);

	m_children.push_back(leftChild);
	m_children.push_back(rightChild);
}



int BVH::getSplitPlane(std::vector<BVHBox*>& bboxes, float& minVal, float& maxVal)
{
	Vector3 min = Vector3(MIRO_TMAX, MIRO_TMAX, MIRO_TMAX);
	Vector3 max = Vector3(-MIRO_TMAX, -MIRO_TMAX, -MIRO_TMAX);

#pragma omp parallel for simd
	for (int i = 0; i < bboxes.size(); i++) {
		Object* obj = bboxes[i]->object[0];
		Vector3 pos = obj->center();

		min.x = std::min(min.x, pos.x);
		min.y = std::min(min.y, pos.y);
		min.z = std::min(min.z, pos.z);

		max.x = std::max(max.x, pos.x);
		max.y = std::max(max.y, pos.y);
		max.z = std::max(max.z, pos.z);
	}

	Vector3 diff = max - min;

	if ((diff.x > diff.y) && (diff.x > diff.z)) {
		minVal = min.x;
		maxVal = max.x;
		return X_AXIS;
	}
	else if (diff.y > diff.z) {
		minVal = min.y;
		maxVal = max.y;
		return Y_AXIS;
	}
	else {
		minVal = min.z;
		maxVal = max.z;
		return Z_AXIS;
	}
}



void BVH::getMinMax(std::vector<BVHBox*>& bboxes, int axis, Vector3& minVal, Vector3& maxVal)
{
	minVal = Vector3(MIRO_TMAX, MIRO_TMAX, MIRO_TMAX);
	maxVal = Vector3(-MIRO_TMAX, -MIRO_TMAX, -MIRO_TMAX);
#pragma omp parallel for simd
	for (int i = 0; i < bboxes.size(); i++) {
		Object* obj = bboxes[i]->object[0];
		Vector3 pos = obj->center();

		minVal.x = std::min(minVal.x, pos.x);
		minVal.y = std::min(minVal.y, pos.y);
		minVal.z = std::min(minVal.z, pos.z);

		maxVal.x = std::max(maxVal.x, pos.x);
		maxVal.y = std::max(maxVal.y, pos.y);
		maxVal.z = std::max(maxVal.z, pos.z);
	}
}


int BVH::getSplitIndex(float val, float minVal, float splitLength, int axis)
{

	int index = -1;
	float boundary = minVal;


	for (int i = 0; i < NUM_SPLITS; i++) {
		if (val < boundary) {
			break;
		}
		index = i;
		boundary += splitLength;
	}

	if (index == -1) {
		std::cerr << "(BVH::getSplitIndex) index is -1" << std::endl;
	}

	return index;
}


bool
BVH::intersect(HitInfo& minHit, const Ray& ray, float tMin, float tMax) const
{
	// intersect is ximply traverse all the obj in the box when the box is leaf
	//While intersect_bvh is doing the true traversal

	bool hit = false;
	HitInfo tempMinHit;

#pragma omp parallel for simd
	for (size_t i = 0; i < m_bbox->object.size(); ++i)
	{

		if (m_bbox->object[i]->intersect(tempMinHit, ray, tMin, tMax))
		{
			num_ray_tri++;
			hit = true;
			if (tempMinHit.t < minHit.t)
				minHit = tempMinHit;
		}
		num_rays++;
	}

	return hit;
}

bool BVH::intersect_bvh(HitInfo& minHit, const Ray& ray, float tMin, float tMax) const
{
	bool is_leaf = (m_children.size() == 0);

	num_rays++;
	if (m_bbox->hit(ray)) {
		BVH::num_ray_box++;

		if (is_leaf) {
			return intersect(minHit, ray, tMin, tMax);
		}
		// for each child box
		else {
			// Error check
			if (m_children.size() == 0) {
				std::cerr << "(BVH::intersect_bvh) m_children is empty" << std::endl;
				return false;
			}

			bool didHit = false;

			// Check left
			if (m_children[0]->intersect_bvh(minHit, ray, tMin, tMax))
				didHit = true;
			// Check right
			if (m_children[1]->intersect_bvh(minHit, ray, tMin, tMax))
				didHit = true;

			return didHit;
		}
	}
	return false;
}

void BVHBox::addObject(Object* obj)
{
	if (obj == NULL) {
		std::cerr << "(BVHBox::addObject) obj is nullptr" << std::endl;
		return;
	}

	object.push_back(obj);

	min.x = std::min(min.x, obj->min().x);
	min.y = std::min(min.y, obj->min().y);
	min.z = std::min(min.z, obj->min().z);

	max.x = std::max(max.x, obj->max().x);
	max.y = std::max(max.y, obj->max().y);
	max.z = std::max(max.z, obj->max().z);
}

bool BVHBox::hit(const Ray& ray)
{
	float tMin, tMax;
	return hit(ray, tMin, tMax);
}

//Overload
bool BVHBox::hit(const Ray& ray, float& tMin, float& tMax)// With Hithelper to perform Slab Test
{
	Vector3 t1, t2;


	t1 = hitHelper(ray, min);
	t2 = hitHelper(ray, max);

	tMin = std::max(std::max(std::min(t1.x, t2.x), std::min(t1.y, t2.y)), std::min(t1.z, t2.z));
	tMax = std::min(std::min(std::max(t1.x, t2.x), std::max(t1.y, t2.y)), std::max(t1.z, t2.z));


	return (tMin <= tMax);
}

Vector3 BVHBox::hitHelper(const Ray& ray, const Vector3& v)
{
	Vector3 t;

	t.x = (v.x - ray.o.x) / ray.d.x;
	t.y = (v.y - ray.o.y) / ray.d.y;
	t.z = (v.z - ray.o.z) / ray.d.z;

	return t;
}

void Spliter::update(BVHBox* box)
{
	if (box == NULL) {
		std::cerr << "(Spliter::update) box is nullptr" << std::endl;
		return;
	}

	num_objs++;

	min.x = std::min(min.x, box->min.x);
	min.y = std::min(min.y, box->min.y);
	min.z = std::min(min.z, box->min.z);

	max.x = std::max(max.x, box->max.x);
	max.y = std::max(max.y, box->max.y);
	max.z = std::max(max.z, box->max.z);
}
BVH::~BVH()
{
	delete m_bbox;

	for (int i = 0; i < m_children.size(); i++) {
		delete m_children[i];
	}
}