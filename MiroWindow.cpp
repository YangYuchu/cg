#include "MiroWindow.h"
#include "OpenGL.h"
#include "Miro.h"
#include "Camera.h"
#include "Image.h"
#include <stdlib.h>
#include <time.h>

#define ANGFACT 1.0
#define LEFT    4
#define MIDDLE  2
#define RIGHT   1

#ifdef _WIN32
	// disable useless warnings
	#pragma warning(disable:4996)
#endif


namespace
{
	// Non-member functions used as proxy callbacks to our real C++ member functions
	MiroWindow *g_miroWindow;
	void display() {g_miroWindow->display();}
	void resize(int x,int y) {g_miroWindow->reshape(x,y);}
	void keyboard(unsigned char key, int x, int y) {g_miroWindow->keyboard(key,x,y);}
	void mouse(int btn,int state,int x,int y) {g_miroWindow->mouse(btn,state,x,y);}
	void motion(int x, int y) {g_miroWindow->motion(x,y);}
} // namespace


MiroWindow::MiroWindow(int * argc, char* argv[]) :
	m_scaleFact(0.1f),
	m_activeButton(0),
	m_mouseX(0),
	m_mouseY(0)
{
	// Initialize GLUT
	glutInit(argc, argv);

	// Create the window
	glutInitWindowSize(g_image->width(), g_image->height());
	glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE);
	glutInitWindowPosition(200, 200);
	glutCreateWindow("miro");

	// Initialize some OpenGL state
	glClearColor(0.25f, 0.25f, 0.25f, 1);
	glDisable(GL_LIGHTING);
	glDisable(GL_TEXTURE_2D);
	glShadeModel(GL_SMOOTH);
	glPolygonMode(GL_FRONT_AND_BACK, GL_LINE); // draw outlines only
}


void
MiroWindow::mainLoop()
{
	// Setup callback functions
	g_miroWindow = this;
	glutDisplayFunc(::display);
	glutKeyboardFunc(::keyboard);
	glutMouseFunc(::mouse);
	glutMotionFunc(::motion);
	glutReshapeFunc(::resize);

	// Start the glut main loop, never returns
	glutMainLoop();
}


void
MiroWindow::display()
{
	g_camera->click(g_scene, g_image); // take a snapshot of the scene

	glFinish(); // flush the openGL pipeline
}


void
MiroWindow::motion(int x, int y)
{
	int dx, dy; // change in mouse coordinates

	dx = x - m_mouseX; // change in mouse coords
	dy = y - m_mouseY;

	if (m_activeButton & LEFT)
	{
		float xfact = -ANGFACT*dy;
		float yfact = -ANGFACT*dx;
		// construct a coordinate system from up and viewdir
		Vector3 vRight = cross(g_camera->viewDir(), g_camera->up());
		// now rotate everything
		Vector3 v = g_camera->viewDir();
		v.rotate(xfact*PI/180., vRight);
		v.rotate(yfact*PI/180., g_camera->up());
		g_camera->setViewDir(v);
	}

	m_mouseX = x; // new current position
	m_mouseY = y;

	glutPostRedisplay();
}


void
MiroWindow::mouse(int button, int state, int x, int y)
{
	int b; // LEFT, MIDDLE, or RIGHT

	switch (button)//检测按得是哪个按键
	{
		case GLUT_LEFT_BUTTON:
			b = LEFT;//LEFT=4
		break;

		case GLUT_MIDDLE_BUTTON:
			b = MIDDLE;//MIDDLE=2
		break;

		case GLUT_RIGHT_BUTTON:
			b = RIGHT;//RIGHT=1
		break;

		default:
			b = 0;
	}

	if (state == GLUT_DOWN)//如果按下了鼠标
	{
		m_mouseX = x;
		m_mouseY = y;
		m_activeButton |= b; //|=是按位或，b根据按键不同，二进制取001,010,100，和m_activeButton做按位或，只有该位全为0结果才是零
							 /* set the proper bit   */
	}
	else
		m_activeButton &= ~b;  //按位与，同上
							   /* clear the proper bit */
}

void
MiroWindow::keyboard(unsigned char key, int x, int y)
{
	switch (key)
	{
		case 27:
			exit(0);
		break;

		case 'i':
		case 'I':
		{
			char str[1024];
			sprintf(str, "miro_%d.ppm", time(0));
			if (g_camera->isOpenGL())
			{
				unsigned char* buf = new unsigned char[g_image->width()*g_image->height()*3];
				glReadPixels(0, 0, g_image->width(), g_image->height(),
								GL_RGB, GL_UNSIGNED_BYTE, buf);
				g_image->writePPM(str, buf, g_image->width(), g_image->height());
			}
			else
			{
				g_image->writePPM(str);
			}
			break;
		}

		case 'r':
		case 'R':
			g_camera->setRenderer(Camera::RENDER_RAYTRACE);//RENDER_RAYTRACE = 1，通过r键，运行setRenderer(1)，即m_renderer = 1.这是执行ray tracing的关键
		break;

		case 'g':
		case 'G':
			g_camera->setRenderer(Camera::RENDER_OPENGL);//RENDER_OPENGL = 0
		break;

		case '=':
			m_scaleFact *= 1.5;//增大镜头移动的速度1.5倍
		break;

		case '-':
			m_scaleFact /= 1.5;//减小镜头移动的速度1.5倍
		break;

		case 'w':
		case 'W':
			g_camera->setEye(g_camera->eye() + m_scaleFact*g_camera->viewDir());
		break;

		case 's':
		case 'S':
			g_camera->setEye(g_camera->eye() - m_scaleFact*g_camera->viewDir());
		break;

		case 'q':
		case 'Q':
			g_camera->setEye(g_camera->eye() + m_scaleFact*g_camera->up());
		break;

		case 'z':
		case 'Z':
			g_camera->setEye(g_camera->eye() - m_scaleFact*g_camera->up());
		break;

		case 'a':
		case 'A':
		{
			Vector3 vRight = cross(g_camera->viewDir(), g_camera->up());
			g_camera->setEye(g_camera->eye() - m_scaleFact*vRight);
			break;
		}

		case 'd':
		case 'D':
		{
			Vector3 vRight = cross(g_camera->viewDir(), g_camera->up());
			g_camera->setEye(g_camera->eye() + m_scaleFact*vRight);
			break;
		}
		break;

		default:
		break;
	}
	glutPostRedisplay();
}


void
MiroWindow::reshape(int w, int h)
{
	g_image->resize(w, h);
	glViewport(0, 0, w, h);
	g_camera->setRenderer(Camera::RENDER_OPENGL);
	glutPostRedisplay();
}
