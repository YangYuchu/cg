#include "Triangle.h"
#include "TriangleMesh.h"
#include "Ray.h"
#include <iomanip>
#include <algorithm> 

//enable barycentric interpolation
const bool barycentric_on = true;

Triangle::Triangle(TriangleMesh * m, unsigned int i) :
	m_mesh(m), m_index(i)
{
}


Triangle::~Triangle()
{
}

void
Triangle::renderGL()
{
	TriangleMesh::TupleI3 ti3 = m_mesh->vIndices()[m_index];
	const Vector3 & v0 = m_mesh->vertices()[ti3.x]; //vertex a of triangle
	const Vector3 & v1 = m_mesh->vertices()[ti3.y]; //vertex b of triangle
	const Vector3 & v2 = m_mesh->vertices()[ti3.z]; //vertex c of triangle

	glBegin(GL_TRIANGLES);
		glVertex3f(v0.x, v0.y, v0.z);
		glVertex3f(v1.x, v1.y, v1.z);
		glVertex3f(v2.x, v2.y, v2.z);
	glEnd();
}




bool
Triangle::intersect(HitInfo& result, const Ray& r,float tMin, float tMax)
{
	TriangleMesh::TupleI3 ti3 = m_mesh->vIndices()[m_index];
	const Vector3 & v_a = m_mesh->vertices()[ti3.x]; //vertex a of triangle
	const Vector3 & v_b = m_mesh->vertices()[ti3.y]; //vertex b of triangle
	const Vector3 & v_c = m_mesh->vertices()[ti3.z]; //vertex c of triangle

	const Vector3 v_ao = r.o - v_a;			// AO
	const Vector3 v_ab = v_b - v_a;			// AB
	const Vector3 v_ac = v_c - v_a;			// AC
	const Vector3 v_ob = r.o - v_b;			// OB
    
	float det_A = determinant(-r.d, v_ab, v_ac); // signed volume of O'-ABC, where O'Q is unit vector, Q is the intersect point

	if (det_A < 0.00000001f && det_A > -0.00001f) return false;//triangle area is 0
  
	float t = determinant(v_ao, v_ab, v_ac) / det_A; //calculate O-ABC/O'-ABC = t

    if (t< tMin || t > tMax) return false;
    
	float beta = determinant(-r.d, v_ao, v_ac) / det_A;
	float gamma = determinant(v_ao, -r.d, v_ab) / det_A;
	float alpha = 1.0f - beta - gamma;

	// Check if ray intersects
	if ((beta >= 0.000000001f) && (gamma >= 0.000000001f) && (beta + gamma <= 1.0f)) {

		result.t = t;
		result.P = r.o + result.t*r.d;

		//Get the normals of A,B,C
		TriangleMesh::TupleI3 ni3 = m_mesh->nIndices()[m_index];
		const Vector3 & n_a = m_mesh->normals()[ni3.x]; //normal a of triangle
		const Vector3 & n_b = m_mesh->normals()[ni3.y]; //normal b of triangle
		const Vector3 & n_c = m_mesh->normals()[ni3.z]; //normal c of triangle
		if (barycentric_on)
		{
			//barycentric interpolation - ON
			result.N = n_a * alpha + n_b * beta + n_c * gamma;
			result.N.normalize();
		}
		else
		{
			// barycentric interpolation - OFF
			result.N = n_a * 1 / 3 + n_b * 1 / 3 + n_c * 1 / 3;
		}

		result.material = this->m_material;
		return true;
	}

	// The ray does not intersect
    return false;
}

Vector3 Triangle::min()
{
	TriangleMesh::TupleI3 ti3 = m_mesh->vIndices()[m_index];
	const Vector3& v0 = m_mesh->vertices()[ti3.x]; //vertex a of triangle
	const Vector3& v1 = m_mesh->vertices()[ti3.y]; //vertex b of triangle
	const Vector3& v2 = m_mesh->vertices()[ti3.z]; //vertex c of triangle

	float x = std::min(std::min(v0.x, v1.x), v2.x);
	float y = std::min(std::min(v0.y, v1.y), v2.y);
	float z = std::min(std::min(v0.z, v1.z), v2.z);

	return Vector3(x, y, z);
}

Vector3 Triangle::max()
{
	TriangleMesh::TupleI3 ti3 = m_mesh->vIndices()[m_index];
	const Vector3& v0 = m_mesh->vertices()[ti3.x]; //vertex a of triangle
	const Vector3& v1 = m_mesh->vertices()[ti3.y]; //vertex b of triangle
	const Vector3& v2 = m_mesh->vertices()[ti3.z]; //vertex c of triangle

	float x = std::max(std::max(v0.x, v1.x), v2.x);
	float y = std::max(std::max(v0.y, v1.y), v2.y);
	float z = std::max(std::max(v0.z, v1.z), v2.z);

	return Vector3(x, y, z);

}

Vector3 Triangle::center()
{
	TriangleMesh::TupleI3 ti3 = m_mesh->vIndices()[m_index];
	const Vector3& v0 = m_mesh->vertices()[ti3.x]; //vertex a of triangle
	const Vector3& v1 = m_mesh->vertices()[ti3.y]; //vertex b of triangle
	const Vector3& v2 = m_mesh->vertices()[ti3.z]; //vertex c of triangle

	return (v0 + v1 + v2) / 3.0f;
}
