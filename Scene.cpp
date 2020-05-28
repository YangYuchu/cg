#include "Miro.h"
#include "Scene.h"
#include "Camera.h"
#include "Image.h"
#include "Console.h"
#include <omp.h>
#include<time.h>

Scene * g_scene = 0;
clock_t startTime, endTime;

void
Scene::openGL(Camera *cam)
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	cam->drawGL();

	// draw objects
	for (size_t i = 0; i < m_objects.size(); ++i) 
	{
		m_objects[i]->renderGL();
	}

	glutSwapBuffers();
}

void
Scene::preCalc()
{
	Objects::iterator it;
	for (it = m_objects.begin(); it != m_objects.end(); it++)
	{
		Object* pObject = *it;
		pObject->preCalc();//No definition of Object.perCalc() now. Definie it if needed.
	}
	Lights::iterator lit;
	for (lit = m_lights.begin(); lit != m_lights.end(); lit++)
	{
		PointLight* pLight = *lit;
		pLight->preCalc();
	}
	// Count Construction TIme
	startTime = clock();

	m_accel.build(&m_objects);

	endTime = clock();
	std::cout << "Construction time : " << (double)(endTime - startTime) / CLOCKS_PER_SEC << "s" << std::endl;
	printf("Number of BVH nodes : %d\n", BVH::num_nodes);
	printf("Number of BVH leaves: %d\n", BVH::num_leaves);
}

void
Scene::raytraceImage(Camera *cam, Image *img)
{
	startTime = clock();	
	
	// loop over all pixels in the image
//#pragma omp parallel for simd
//#pragma omp simd

	for (int j = 0; j < img->height(); ++j)
	{
		#pragma omp parallel for  
		for (int i = 0; i < img->width(); ++i)
		{
			Ray ray;
			HitInfo hitInfo;
			Vector3 shadeResult;
			ray = cam->eyeRay(i, j, img->width(), img->height());
			if (trace(hitInfo, ray))
			{
				shadeResult = hitInfo.material->shade(ray, hitInfo, *this);
				img->setPixel(i, j, shadeResult);
			}
		}
		img->drawScanline(j);
		glFinish();
		printf("Rendering Progress: %.3f%%\r", j/float(img->height())*100.0f);
		fflush(stdout);
	}



	endTime = clock();
	printf("Rendering Progress: 100.000%\n");
	debug("done Raytracing!\n");
	std::cout << "Ray tracing time : " << (double)(endTime - startTime) / CLOCKS_PER_SEC << "s" << std::endl;

	printf("Number of rays: %d\n", BVH::num_rays);
	printf("Number of ray box: %d\n", BVH::num_ray_box);
	printf("Number of ray triangle: %d\n", BVH::num_ray_tri);
}

bool
Scene::trace(HitInfo& minHit, const Ray& ray, float tMin, float tMax) const
{
	minHit.t = MIRO_TMAX;
	return m_accel.intersect_bvh(minHit, ray, tMin, tMax);
}
