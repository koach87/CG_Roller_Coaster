/************************************************************************
     File:        TrainView.cpp

     Author:     
                  Michael Gleicher, gleicher@cs.wisc.edu

     Modifier
                  Yu-Chi Lai, yu-chi@cs.wisc.edu
     
     Comment:     
						The TrainView is the window that actually shows the 
						train. Its a
						GL display canvas (Fl_Gl_Window).  It is held within 
						a TrainWindow
						that is the outer window with all the widgets. 
						The TrainView needs 
						to be aware of the window - since it might need to 
						check the widgets to see how to draw

	  Note:        we need to have pointers to this, but maybe not know 
						about it (beware circular references)

     Platform:    Visio Studio.Net 2003/2005

*************************************************************************/
/*
todo：
 3. 地板
 (done)4. 打光
 (done)7. arclength 參數化
 8. 輪子
 (done)9. 重力
 10. 煙
 11. 非平坦地形
*/

// opencv
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>

#include "time.h"
#include <iostream>
#include <fstream>
#include <Fl/fl.h>

// we will need OpenGL, and OpenGL needs windows.h
#include <windows.h>
//#include "GL/gl.h"
#include <glad/glad.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include "GL/glu.h"

#include "TrainView.H"
#include "TrainWindow.H"
#include "Utilities/3DUtils.H"

using namespace cv;

// include assimp library
#include <assimp/scene.h>
#include<assimp/postprocess.h>
#include<assimp/postprocess.h>
// load model 
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include "model.h"
#include "shader.h"
#include"filesystem.h"

#define EXAMPLE_SOLUTION_TRACK

#ifdef EXAMPLE_SOLUTION
#	include "TrainExample/TrainExample.H"
#endif
// for read 3d model
float transAngle(float angle_) {
	if (sinf(angle_) * cosf(angle_) >= 0) {
		return angle_;
	}
	if(sinf(angle_) <= 0) {
		angle_ -= 180;
		return angle_;
	}
	angle_ += 180;
	return angle_;
}
class obj3dmodel
{
	struct vertex {
		double x;
		double y;
		double z;
	};
	struct face {
		unsigned int v1, v2, v3;
	};
	std::vector<vertex> vetexes;
	std::vector<face> faces;

public:
	void readfile(const char* filename);
	void draw();
};
void obj3dmodel::readfile(const char* filename)
{
	std::string s;
	std::ifstream fin(filename);
	if (!fin)
		return;
	while (fin >> s)
	{
		switch (*s.c_str())
		{
		case 'v':
		{
			vertex v;
			fin >> v.x >> v.y >> v.z;
			this->vetexes.push_back(v);
		}
		break;
		case 'f':
		{
			face f;
			fin >> f.v1 >> f.v2 >> f.v3;
			faces.push_back(f);
		}
		break;
		}
	}
}

// curve function
//state 1,2,3 = linear, cardinal, B-Spline 
//pos 1/0 = pos
Pnt3f  curve_function(Pnt3f G1, Pnt3f G2, Pnt3f G3, Pnt3f G4, float t, int state, bool pos) {
	//做GMT
	float G[3][4] = {
		{G1.x, G2.x, G3.x, G4.x},
		{G1.y, G2.y, G3.y, G4.y},
		{G1.z, G2.z, G3.z, G4.z},
	};
	float M_cardinal[4][4] = {
	{-0.5f, 1.0f, -0.5f, 0.0f},
	{1.5f, -2.5f, 0.0f, 1.0f},
	{-1.5f, 2.0f, 0.5f, 0.0f},
	{0.5f, -0.5f, 0.0f, 0.0f}
	};
	float M_bspline[4][4] = {
	{-0.167f ,	0.5f, -0.5f, 0.167f},
	{0.5f, -1.0f, 0.0f, 0.667f},
	{-0.5f, 0.5f, 0.5f, 0.167f},
	{0.167f, 0.0f, 0.0f, 0.0f}
	};
	float T[4][1] = {
	{t * t * t},
	{t * t},
	{t},
	{1}
	};
	/*else {
		float T[4][1] = {
		{t * t },
		{t },
		{1},
		{0}
		};
	}*/
	
	float G_M[4][4] = {
		{0,0,0,0},{0,0,0,0},{0,0,0,0},{0,0,0,0}
	};
	float G_M_T[3][1] = {
		{0},{0},{0},
	};
	//G*M
	if (state == 2) {
		for (int i = 0; i < 3; i++) {
			for (int j = 0; j < 4; j++) {
				for (int k = 0; k < 4; k++) {
					G_M[i][j] += G[i][k] * M_cardinal[k][j];
				}
			}
		}
	}
	else if (state == 3) {
		for (int i = 0; i < 3; i++) {
			for (int j = 0; j < 4; j++) {
				for (int k = 0; k < 4; k++) {
					G_M[i][j] += G[i][k] * M_bspline[k][j];
				}
			}
		}
	}

	//GM*T
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 1; j++) {
			for (int k = 0; k < 4; k++) {
				G_M_T[i][j] += G_M[i][k] *T[k][j];
			}
		}
	}
	return Pnt3f(G_M_T[0][0], G_M_T[1][0], G_M_T[2][0]);
}

//************************************************************************
//
// * Constructor to set up the GL window
//========================================================================
TrainView::
TrainView(int x, int y, int w, int h, const char* l) 
	: Fl_Gl_Window(x,y,w,h,l)
//========================================================================
{
	mode( FL_RGB|FL_ALPHA|FL_DOUBLE | FL_STENCIL );

	resetArcball();
}

//************************************************************************
//
// * Reset the camera to look at the world
//========================================================================
void TrainView::
resetArcball()
//========================================================================
{
	// Set up the camera to look at the world
	// these parameters might seem magical, and they kindof are
	// a little trial and error goes a long way
	arcball.setup(this, 40, 250, .2f, .4f, 0);
}

//************************************************************************
//
// * FlTk Event handler for the window
//########################################################################
// TODO: 
//       if you want to make the train respond to other events 
//       (like key presses), you might want to hack this.
//########################################################################
//========================================================================
int TrainView::handle(int event)
{
	// see if the ArcBall will handle the event - if it does, 
	// then we're done
	// note: the arcball only gets the event if we're in world view
	if (tw->worldCam->value())
		if (arcball.handle(event)) 
			return 1;

	// remember what button was used
	static int last_push;

	switch(event) {
		// Mouse button being pushed event
		case FL_PUSH:
			last_push = Fl::event_button();
			// if the left button be pushed is left mouse button
			if (last_push == FL_LEFT_MOUSE  ) {
				doPick();
				damage(1);
				return 1;
			};
			break;

	   // Mouse button release event
		case FL_RELEASE: // button release
			damage(1);
			last_push = 0;
			return 1;

		// Mouse button drag event
		case FL_DRAG:

			// Compute the new control point position
			if ((last_push == FL_LEFT_MOUSE) && (selectedCube >= 0)) {
				ControlPoint* cp = &m_pTrack->points[selectedCube];

				double r1x, r1y, r1z, r2x, r2y, r2z;
				getMouseLine(r1x, r1y, r1z, r2x, r2y, r2z);

				double rx, ry, rz;
				mousePoleGo(r1x, r1y, r1z, r2x, r2y, r2z, 
								static_cast<double>(cp->pos.x), 
								static_cast<double>(cp->pos.y),
								static_cast<double>(cp->pos.z),
								rx, ry, rz,
								(Fl::event_state() & FL_CTRL) != 0);

				cp->pos.x = (float) rx;
				cp->pos.y = (float) ry;
				cp->pos.z = (float) rz;
				damage(1);
			}
			break;

		// in order to get keyboard events, we need to accept focus
		case FL_FOCUS:
			return 1;

		// every time the mouse enters this window, aggressively take focus
		case FL_ENTER:	
			focus(this);
			break;

		case FL_KEYBOARD:
		 		int k = Fl::event_key();
				int ks = Fl::event_state();
				if (k == 'p') {
					// Print out the selected control point information
					if (selectedCube >= 0) 
						printf("Selected(%d) (%g %g %g) (%g %g %g)\n",
								 selectedCube,
								 m_pTrack->points[selectedCube].pos.x,
								 m_pTrack->points[selectedCube].pos.y,
								 m_pTrack->points[selectedCube].pos.z,
								 m_pTrack->points[selectedCube].orient.x,
								 m_pTrack->points[selectedCube].orient.y,
								 m_pTrack->points[selectedCube].orient.z);
					else
						printf("Nothing Selected\n");

					return 1;
				};
				break;
	}

	return Fl_Gl_Window::handle(event);
}

//************************************************************************
//
// * this is the code that actually draws the window
//   it puts a lot of the work into other routines to simplify things
//========================================================================
void TrainView::draw()
{

	//*********************************************************************
	//
	// * Set up basic opengl informaiton
	//
	//**********************************************************************
	//initialized glad
	if (gladLoadGL())
	{
		//initiailize VAO, VBO, Shader...
	}
	else
		throw std::runtime_error("Could not initialize GLAD!");

	// Set up the view port
	glViewport(0,0,w(),h());

	// clear the window, be sure to clear the Z-Buffer too
	glClearColor(0,0,.3f,0);		// background should be blue

	// we need to clear out the stencil buffer since we'll use
	// it for shadows
	glClearStencil(0);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);
	glEnable(GL_DEPTH);

	// Blayne prefers GL_DIFFUSE
    glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);

	// prepare for projection
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	setProjection();		// put the code to set up matrices here

	//######################################################################
	// TODO: 
	// you might want to set the lighting up differently. if you do, 
	// we need to set up the lights AFTER setting up the projection
	//######################################################################
	// enable the lighting
	glEnable(GL_COLOR_MATERIAL);
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);

	// top view only needs one light
	if (tw->topCam->value()) {
		glDisable(GL_LIGHT1);
		glDisable(GL_LIGHT2);
	} else {
		glEnable(GL_LIGHT1);
		glEnable(GL_LIGHT2);
	}

	//*********************************************************************
	//
	// * set the light parameters
	//
	//**********************************************************************
	GLfloat lightPosition1[]	= {0,1,1,0}; // {50, 200.0, 50, 1.0};
	GLfloat lightPosition2[]	= {1, 0, 0, 0};
	GLfloat lightPosition3[]	= {0, -1, 0, 0};
	GLfloat yellowLight[]		= {0.5f, 0.5f, .1f, 1.0};
	GLfloat whiteLight[]			= {1.0f, 1.0f, 1.0f, 1.0};
	GLfloat blueLight[]			= {.1f,.1f,.3f,1.0};
	GLfloat grayLight[]			= {.3f, .3f, .3f, 1.0};

	glLightfv(GL_LIGHT0, GL_POSITION, lightPosition1);
	glLightfv(GL_LIGHT0, GL_DIFFUSE, whiteLight);
	glLightfv(GL_LIGHT0, GL_AMBIENT, grayLight);

	glLightfv(GL_LIGHT1, GL_POSITION, lightPosition2);
	glLightfv(GL_LIGHT1, GL_DIFFUSE, yellowLight);

	glLightfv(GL_LIGHT2, GL_POSITION, lightPosition3);
	glLightfv(GL_LIGHT2, GL_DIFFUSE, blueLight);



	//*********************************************************************
	// now draw the ground plane
	//*********************************************************************
	// set to opengl fixed pipeline(use opengl 1.x draw function)
	glUseProgram(0);

	setupFloor();

	glDisable(GL_LIGHTING);
	glEnable(GL_LIGHTING);
	drawFloor(200,5);


	//*********************************************************************
	// now draw the object and we need to do it twice
	// once for real, and then once for shadows
	//*********************************************************************

	setupObjects();

	drawStuff();

	// this time drawing is for shadows (except for top view)
	if (!tw->topCam->value()) {
		setupShadows();
		drawStuff(true);
		unsetupShadows();
	}
	
	float yellowAmbientDiffuse[] = { 0.0f,0.0f, 1.0f, 0.3f };
	float position[] = { 0.0f, 100.0f, 0.0f, 1.0f };

	glEnable(GL_LIGHT1);
	glLightfv(GL_LIGHT1, GL_AMBIENT, yellowAmbientDiffuse);
	glLightfv(GL_LIGHT1, GL_POSITION, position);
	
	
}

//************************************************************************
//
// * This sets up both the Projection and the ModelView matrices
//   HOWEVER: it doesn't clear the projection first (the caller handles
//   that) - its important for picking
//========================================================================
void TrainView::
setProjection()
//========================================================================
{
	// Compute the aspect ratio (we'll need it)
	float aspect = static_cast<float>(w()) / static_cast<float>(h());

	// Check whether we use the world camp
	if (tw->worldCam->value())
		arcball.setProjection(false);
	// Or we use the top cam
	else if (tw->topCam->value()) {
		float wi, he;
		if (aspect >= 1) {
			wi = 110;
			he = wi / aspect;
		} 
		else {
			he = 110;
			wi = he * aspect;
		}

		// Set up the top camera drop mode to be orthogonal and set
		// up proper projection matrix
		glMatrixMode(GL_PROJECTION);
		glOrtho(-wi, wi, -he, he, 200, -200);
		glMatrixMode(GL_MODELVIEW);
		glLoadIdentity();
		glRotatef(-90,1,0,0);
	} 
	// Or do the train view or other view here
	//####################################################################
	// TODO: 
	// put code for train view projection here!	
	//####################################################################
	else {
#ifdef EXAMPLE_SOLUTION
		trainCamView(this,aspect);
#endif
		glMatrixMode(GL_MODELVIEW);
		glLoadIdentity();
		gluLookAt(
			point_list[point_index].x, point_list[point_index].y + 20.0f, point_list[point_index].z,
			point_list[(point_index + 1) % point_list.size()].x,
			point_list[(point_index + 1) % point_list.size()].y + 20.0f,
			point_list[(point_index + 1) % point_list.size()].z,
			//orient_list[point_index].x, orient_list[point_index].y, orient_list[point_index].z
			//0.0f, 1.0f, orient_list[point_index].z
			0.0f, 1.0f, 0.0f
			);
		glMatrixMode(GL_PROJECTION);
		glLoadIdentity();
		gluPerspective(60, aspect, 1.0, 200.0);
	}
}

//************************************************************************
//
// * this draws all of the stuff in the world
//
//	NOTE: if you're drawing shadows, DO NOT set colors (otherwise, you get 
//       colored shadows). this gets called twice per draw 
//       -- once for the objects, once for the shadows
//########################################################################
// TODO: 
// if you have other objects in the world, make sure to draw them
//########################################################################
//========================================================================
void TrainView::drawStuff(bool doingShadows)
{
	// Draw the control points
	// don't draw the control points if you're driving 
	// (otherwise you get sea-sick as you drive through them)
	if (!tw->trainCam->value()) {
		for(size_t i=0; i<m_pTrack->points.size(); ++i) {
			if (!doingShadows) {
				if ( ((int) i) != selectedCube)
					glColor3ub(240, 60, 60);
				else
					glColor3ub(240, 240, 30);
			}
			m_pTrack->points[i].draw();
		}
	}
	// draw the track
	//####################################################################
	// TODO: 
	// call your own track drawing code
	//####################################################################

	
	
#ifdef EXAMPLE_SOLUTION_TRACK
	drawTrack(doingShadows);
#endif

	// draw the train
	//####################################################################
	// TODO: 
	//	call your own train drawing code
	//####################################################################
	for (int i = 0; i < carAmount; i++) {
		if (i == carAmount-1) {
			drawTrain(
				doingShadows,
				point_list[((point_index + carInterval * i) % point_list.size())],
				point_list[((point_index + carInterval * i + 1) % point_list.size())],
				orient_list[((point_index + carInterval * i) % point_list.size())],
				true
			);
		}
		else {
			drawTrain(
				doingShadows,
				point_list[((point_index + carInterval * i) % point_list.size())],
				point_list[((point_index + carInterval * i + 1) % point_list.size())],
				orient_list[((point_index + carInterval * i) % point_list.size())],
				false
			);
		}
		
		//std::cout << "==========\n";
		//std::cout << (point_index + carInterval * i) % point_list.size() << std::endl;
		//std::cout << (point_index + carInterval * i + 1) % point_list.size() << std::endl;
		//std::cout << "==========\n";
		//glTranslatef((point_list[(point_index + carInterval * i) % point_list.size()].x+ point_list[(point_index + carInterval * i + 20) % point_list.size()].x)/2,
		//	(point_list[(point_index + carInterval * i) % point_list.size()].y+ point_list[(point_index + carInterval * i + 20) % point_list.size()].y)/2+14,
		//	(point_list[(point_index + carInterval * i) % point_list.size()].z+ point_list[(point_index + carInterval * i + 20) % point_list.size()].z)/2);
		//drawCube(Pnt3f(1, 2, 1), Pnt3f(1, -2, 1), Pnt3f(-1, -2, 1), Pnt3f(-1, 2, 1));
		//drawCube(Pnt3f(2, 2, 1), Pnt3f(2, 4, 1), Pnt3f(-2, 4, 1), Pnt3f(-2, 2, 1));
		//drawCube(Pnt3f(1, 0, 1), Pnt3f(1, 1, 1), Pnt3f(3, 2, 1), Pnt3f(3, 1, 1));
		//drawCube(Pnt3f(-1, 0, 1), Pnt3f(-1, 1, 1), Pnt3f(-3, 2, 1), Pnt3f(-3, 1, 1));
		//drawCube(Pnt3f(0, -2, 1), Pnt3f(-1, -2, 1), Pnt3f(-2, -4, 1), Pnt3f(-1, -4, 1));
		//drawCube(Pnt3f(0, -2, 1), Pnt3f(1, -2, 1), Pnt3f(2, -4, 1), Pnt3f(1, -4, 1));
		//glTranslatef(-(point_list[(point_index + carInterval * i) % point_list.size()].x + point_list[(point_index + carInterval * i + 20) % point_list.size()].x) / 2,
		//	-(point_list[(point_index + carInterval * i) % point_list.size()].y + point_list[(point_index + carInterval * i + 20) % point_list.size()].y) / 2 - 14,
		//	-(point_list[(point_index + carInterval * i) % point_list.size()].z + point_list[(point_index + carInterval * i + 20) % point_list.size()].z) / 2);
		//glColor3ub(10, 10, 10);
	}
	//drawWheel();
}

// 
//************************************************************************
//
// * this tries to see which control point is under the mouse
//	  (for when the mouse is clicked)
//		it uses OpenGL picking - which is always a trick
//########################################################################
// TODO: 
//		if you want to pick things other than control points, or you
//		changed how control points are drawn, you might need to change this
//########################################################################
//========================================================================
void TrainView::
doPick()
//========================================================================
{
	// since we'll need to do some GL stuff so we make this window as 
	// active window
	make_current();		

	// where is the mouse?
	int mx = Fl::event_x(); 
	int my = Fl::event_y();

	// get the viewport - most reliable way to turn mouse coords into GL coords
	int viewport[4];
	glGetIntegerv(GL_VIEWPORT, viewport);

	// Set up the pick matrix on the stack - remember, FlTk is
	// upside down!
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity ();
	gluPickMatrix((double)mx, (double)(viewport[3]-my), 
						5, 5, viewport);

	// now set up the projection
	setProjection();

	// now draw the objects - but really only see what we hit
	GLuint buf[100];
	glSelectBuffer(100,buf);
	glRenderMode(GL_SELECT);
	glInitNames();
	glPushName(0);

	// draw the cubes, loading the names as we go
	for(size_t i=0; i<m_pTrack->points.size(); ++i) {
		glLoadName((GLuint) (i+1));
		m_pTrack->points[i].draw();
	}

	// go back to drawing mode, and see how picking did
	int hits = glRenderMode(GL_RENDER);
	if (hits) {
		// warning; this just grabs the first object hit - if there
		// are multiple objects, you really want to pick the closest
		// one - see the OpenGL manual 
		// remember: we load names that are one more than the index
		selectedCube = buf[3]-1;
	} else // nothing hit, nothing selected
		selectedCube = -1;

	printf("Selected Cube %d\n",selectedCube);
}

void TrainView::
drawTrack(bool doingShadow) {
	nonArcCnt = 0;
	POINT_CNT_LEN = 0.0f;
	RAIL_CNT_LEN = 0.0f;
	DIS = 0.0f;
	point_list.clear();
	orient_list.clear();
	int state = tw->splineBrowser->value();
	int arcState = tw->arcLength->value();
	//auto state = tw->splineBrowser->value();
	// 目前選擇的線狀態 tw->splineBrowser->value(); 
	// state : Linear
	if (state == 1) {

		for (size_t i = 0; i < m_pTrack->points.size(); ++i) {
			// p1為當前點，p2為下個點。
			// pos 
			Pnt3f cp_pos_p1 = m_pTrack->points[i].pos;
			Pnt3f cp_pos_p2 = m_pTrack->points[(i + 1) % m_pTrack->points.size()].pos;
			// orient
			Pnt3f cp_orient_p1 = m_pTrack->points[i].orient;
			Pnt3f cp_orient_p2 = m_pTrack->points[(i + 1) % m_pTrack->points.size()].orient;

			// DIVIDE_LINE代表切成幾段
			float percent = 1.0f / DIVIDE_LINE;

			// t 是貝茲曲線的 t
			float t = 0;
			Pnt3f qt = (1 - t) * cp_pos_p1 + t * cp_pos_p2;

			for (size_t j = 0; j < DIVIDE_LINE; j++) {

				Pnt3f orient_t = (1 - t) * cp_orient_p1 + t * cp_orient_p2;
				Pnt3f qt0 = qt;
				t += percent;
				qt = (1 - t) * cp_pos_p1 + t * cp_pos_p2;
				Pnt3f qt1 = qt;
				orient_t.normalize();
				Pnt3f cross_t = (qt1 - qt0) * orient_t;
				cross_t.normalize();
				cross_t = cross_t * 2.5f;

				glLineWidth(3);
				glBegin(GL_LINES);
				glVertex3f(qt0.x + cross_t.x, qt0.y + cross_t.y, qt0.z + cross_t.z);
				glVertex3f(qt1.x + cross_t.x, qt1.y + cross_t.y, qt1.z + cross_t.z);
				glVertex3f(qt0.x - cross_t.x, qt0.y - cross_t.y, qt0.z - cross_t.z);
				glVertex3f(qt1.x - cross_t.x, qt1.y - cross_t.y, qt1.z - cross_t.z);
				glEnd();
				glLineWidth(1);

				DIS = (qt1 - qt0).distance();
				RAIL_CNT_LEN += DIS;
				POINT_CNT_LEN += DIS;
				if (RAIL_CNT_LEN > RAIL_INTERVAL_LEN) {
					Pnt3f cross_tt = cross_t * 2;
					qt1 = qt1 - qt0;
					qt1.x = qt0.x + qt1.x / DIS * RAIL_WIDTH;
					qt1.y = qt0.y + qt1.y / DIS * RAIL_WIDTH;
					qt1.z = qt0.z + qt1.z / DIS * RAIL_WIDTH;
					RAIL_CNT_LEN = 0;

					glBegin(GL_POLYGON);
					glVertex3f(qt0.x + cross_tt.x, qt0.y + cross_tt.y, qt0.z + cross_tt.z);
					glVertex3f(qt0.x - cross_tt.x, qt0.y - cross_tt.y, qt0.z - cross_tt.z);
					glVertex3f(qt1.x - cross_tt.x, qt1.y - cross_tt.y, qt1.z - cross_tt.z);
					glVertex3f(qt1.x + cross_tt.x, qt1.y + cross_tt.y, qt1.z + cross_tt.z);
					glEnd();
				}
				//std::cout << arcState << "\n";
				if (POINT_CNT_LEN > POINT_INTERVAL_LEN) {
					point_list.push_back(qt0);
					orient_list.push_back(cross_t);
					POINT_CNT_LEN = 0;
				}
			}
		}
	}
	// state : Cardinal Cubic
	else {
		for (size_t i = 0; i < m_pTrack->points.size(); ++i) {
			// p1為當前點，p2為下個點。
			// pos 位置
			Pnt3f cp_pos_p1 = m_pTrack->points[i].pos;
			Pnt3f cp_pos_p2 = m_pTrack->points[(i + 1) % m_pTrack->points.size()].pos;
			Pnt3f cp_pos_p3 = m_pTrack->points[(i + 2) % m_pTrack->points.size()].pos;
			Pnt3f cp_pos_p4 = m_pTrack->points[(i + 3) % m_pTrack->points.size()].pos;
			// orient 方向
			Pnt3f cp_orient_p1 = m_pTrack->points[i].orient;
			Pnt3f cp_orient_p2 = m_pTrack->points[(i + 1) % m_pTrack->points.size()].orient;
			Pnt3f cp_orient_p3 = m_pTrack->points[(i + 2) % m_pTrack->points.size()].orient;
			Pnt3f cp_orient_p4 = m_pTrack->points[(i + 3) % m_pTrack->points.size()].orient;

			// DIVIDE_LINE代表切成幾段
			float percent = 1.0f / DIVIDE_LINE;

			// t 是貝茲曲線的 t
			float t = 0;
			Pnt3f qt = curve_function(cp_pos_p1, cp_pos_p2, cp_pos_p3, cp_pos_p4, t, state, true);

			for (size_t j = 0; j < DIVIDE_LINE; j++) {

				Pnt3f orient_t = curve_function(cp_orient_p1, cp_orient_p2, cp_orient_p3, cp_orient_p4, t, state, true); ;
				Pnt3f qt0 = qt;
				t += percent;
				qt = curve_function(cp_pos_p1, cp_pos_p2, cp_pos_p3, cp_pos_p4, t, state, true);
				Pnt3f qt1 = qt;
				orient_t.normalize();
				Pnt3f cross_t = (qt1 - qt0) * orient_t;
				cross_t.normalize();
				cross_t = cross_t * 2.5f;

				glLineWidth(3);
				glBegin(GL_LINES);
				glVertex3f(qt0.x + cross_t.x, qt0.y + cross_t.y, qt0.z + cross_t.z);
				glVertex3f(qt1.x + cross_t.x, qt1.y + cross_t.y, qt1.z + cross_t.z);
				glVertex3f(qt0.x - cross_t.x, qt0.y - cross_t.y, qt0.z - cross_t.z);
				glVertex3f(qt1.x - cross_t.x, qt1.y - cross_t.y, qt1.z - cross_t.z);
				glEnd();
				glLineWidth(1);

				DIS = (qt1 - qt0).distance();
				RAIL_CNT_LEN += DIS;
				POINT_CNT_LEN += DIS;
				
				//draw rail
				if (RAIL_CNT_LEN > RAIL_INTERVAL_LEN ) {
					// record last 2 point 
					if (firstLock)
					{
						firstLock = false;
						lastTwoPoint = qt0;
					}
					else if (secondLock)
					{
						secondLock = false;
						lastPoint = qt0;
					}
					else
					{
						lastVec = (lastTwoPoint - lastPoint);
						nowVec = (lastPoint - qt0);
						//std::cout << lastVec.x <<"\t" << lastVec.y<<"\t" << lastVec.z << "\n";
						//std::cout << nowVec.x << "\t" << nowVec.y << "\t" << nowVec.z << "\n";
						////內積/距離相乘=cos theta
						//std::cout << (lastVec % nowVec) / (lastTwoPoint - lastPoint).distance() / (lastPoint - qt0).distance() << "\n";
						//std::cout << "===================\n";
						//RAIL_CNT_LEN = 0;
						//std::cout<< (lastVec % nowVec) << std::endl;

						if ((lastVec % nowVec) / (lastTwoPoint - lastPoint).distance() / (lastPoint - qt0).distance() < 0.9f|| adaptiveLock)
						{
							// set point of rail
							Pnt3f cross_tt = cross_t * 2;
							qt1 = qt1 - qt0;
							qt1.x = qt0.x + qt1.x / DIS * RAIL_WIDTH;
							qt1.y = qt0.y + qt1.y / DIS * RAIL_WIDTH;
							qt1.z = qt0.z + qt1.z / DIS * RAIL_WIDTH;

							glBegin(GL_POLYGON);
							glVertex3f(qt0.x + cross_tt.x, qt0.y + cross_tt.y, qt0.z + cross_tt.z);
							glVertex3f(qt0.x - cross_tt.x, qt0.y - cross_tt.y, qt0.z - cross_tt.z);
							glVertex3f(qt1.x - cross_tt.x, qt1.y - cross_tt.y, qt1.z - cross_tt.z);
							glVertex3f(qt1.x + cross_tt.x, qt1.y + cross_tt.y, qt1.z + cross_tt.z);
							glEnd();
							adaptiveLock = false;
							//std::cout << "Draw"<<cc << std::endl;
							cc++;
						}
						else
						{
							//std::cout << "noDraw" << cc << std::endl;
							adaptiveLock = true;
							cc++;
						}
						RAIL_CNT_LEN = 0;

						lastTwoPoint = lastPoint;
						lastPoint = qt0;
					}
					
					

					
				}
				//set point
				if (POINT_CNT_LEN > POINT_INTERVAL_LEN) {
					point_list.push_back(qt0);
					orient_list.push_back(cross_t);
					POINT_CNT_LEN = 0;
				}
			}
		}
	}
	// draw obj
	// drawModel();
	// draw Tunnel，分三個cube。
	

	glTranslatef(50.0f, 0.0f, 0.0f);
	drawCube(Pnt3f(-20, 0, 15), Pnt3f(-20, 10, 1), Pnt3f(-10, 40, 1), Pnt3f(-10, 0, 1));
	drawCube(Pnt3f(-10, 40, 15), Pnt3f(-10, 20, 1), Pnt3f(10, 20, 1), Pnt3f(10, 40, 1));
	drawCube(Pnt3f(10, 40, 15), Pnt3f(10, 0, 1), Pnt3f(20, 0, 1), Pnt3f(20, 10, 1));
	glTranslatef(-50.0f, 0.0f, 0.0f);
	glColor3ub(10, 10, 10);
}
void TrainView::
drawTrain(bool doingShadow, Pnt3f pos0, Pnt3f pos1, Pnt3f ori, bool head) {
	auto TRAIN_HEIGHT = 10.0f;
	auto TRAIN_WIDTH = 5.0f;
	auto TRAIN_DEPTH = 20.0f;

	auto WIDTH = ori.distance();
	ori.x = ori.x / WIDTH * TRAIN_WIDTH;
	ori.y = ori.y / WIDTH * TRAIN_WIDTH;
	ori.z = ori.z / WIDTH * TRAIN_WIDTH;


	pos1 = pos1 - pos0;
	auto DEPTH = pos1.distance();
	pos1.x = pos0.x + pos1.x / DEPTH * TRAIN_DEPTH;
	pos1.y = pos0.y + pos1.y / DEPTH * TRAIN_DEPTH;
	pos1.z = pos0.z + pos1.z / DEPTH * TRAIN_DEPTH;
	
	Pnt3f vectorA = (pos1 - pos0);
	Pnt3f vectorB = (ori);
	Pnt3f normal;
	normal.x = vectorA.y * vectorB.z - vectorA.z * vectorB.y;
	normal.y = vectorA.x * vectorB.z - vectorA.z * vectorB.x;
	normal.z = vectorA.x * vectorB.y - vectorA.y * vectorB.x;
	auto normal_len = normal.distance();
	normal.x = normal.x / normal_len;
	normal.y = normal.y / normal_len;
	normal.z = normal.z / normal_len;

	Pnt3f pos0_u;
	pos0_u.x = pos0.x - normal.x * TRAIN_HEIGHT;
	pos0_u.y = pos0.y + normal.y * TRAIN_HEIGHT;
	pos0_u.z = pos0.z - normal.z * TRAIN_HEIGHT;

	vectorA = (ori);
	vectorB = (pos0 - pos1);
	normal.x = vectorA.y * vectorB.z - vectorA.z * vectorB.y;
	normal.y = vectorA.x * vectorB.z - vectorA.z * vectorB.x;
	normal.z = vectorA.x * vectorB.y - vectorA.y * vectorB.x;
	normal_len = normal.distance();
	normal.x = normal.x / normal_len;
	normal.y = normal.y / normal_len;
	normal.z = normal.z / normal_len;

	Pnt3f pos1_u;
	pos1_u.x = pos1.x - normal.x * TRAIN_HEIGHT;
	pos1_u.y = pos1.y + normal.y * TRAIN_HEIGHT;
	pos1_u.z = pos1.z - normal.z * TRAIN_HEIGHT;

	//Up
	glBegin(GL_QUADS);
	glNormal3f(0,1,0);
	if (!doingShadow)
		glColor3ub(255,255,255);
	glTranslatef(0.0f, 20.0f, 0.0f);
	glVertex3f(pos0_u.x + ori.x, pos0_u.y + ori.y, pos0_u.z + ori.z);
	glVertex3f(pos0_u.x - ori.x, pos0_u.y - ori.y, pos0_u.z - ori.z);
	glVertex3f(pos1_u.x - ori.x, pos1_u.y - ori.y, pos1_u.z - ori.z);
	glVertex3f(pos1_u.x + ori.x, pos1_u.y + ori.y, pos1_u.z + ori.z);
	glEnd();

	//Down
	glBegin(GL_QUADS);
	glNormal3f(0, -1, 0);
	if (!doingShadow)
		glColor3ub(100, 0, 0);
	glVertex3f(pos0.x + ori.x, pos0.y + ori.y, pos0.z + ori.z);
	glVertex3f(pos0.x - ori.x, pos0.y - ori.y, pos0.z - ori.z);
	glVertex3f(pos1.x - ori.x, pos1.y - ori.y, pos1.z - ori.z);
	glVertex3f(pos1.x + ori.x, pos1.y + ori.y , pos1.z + ori.z);
	glEnd();

	//Left
	glBegin(GL_QUADS);
	glNormal3f(-1, 0, 0);
	if (!doingShadow)
		glColor3ub(100, 0, 0);
	glVertex3f(pos0_u.x - ori.x, pos0_u.y - ori.y, pos0_u.z - ori.z);
	glVertex3f(pos1_u.x - ori.x, pos1_u.y - ori.y, pos1_u.z - ori.z);
	glVertex3f(pos1.x - ori.x, pos1.y - ori.y , pos1.z - ori.z);
	glVertex3f(pos0.x - ori.x, pos0.y - ori.y , pos0.z - ori.z);
	glEnd();

	//Right
	glBegin(GL_QUADS);
	glNormal3f(1, 0, 0);
	if (!doingShadow)
		glColor3ub(100, 0, 0);
	glVertex3f(pos0_u.x + ori.x, pos0_u.y + ori.y, pos0_u.z + ori.z);
	glVertex3f(pos1_u.x + ori.x, pos1_u.y + ori.y, pos1_u.z + ori.z);
	glVertex3f(pos1.x + ori.x, pos1.y + ori.y , pos1.z + ori.z);
	glVertex3f(pos0.x + ori.x, pos0.y + ori.y , pos0.z + ori.z);
	glEnd();

	//Front
	glBegin(GL_QUADS);
	glNormal3f(0, 0, -1);
	if (!doingShadow)
		glColor3ub(100, 0, 0);
	glVertex3f(pos1_u.x - ori.x, pos1_u.y - ori.y, pos1_u.z - ori.z);
	glVertex3f(pos1.x - ori.x, pos1.y - ori.y , pos1.z - ori.z);
	glVertex3f(pos1.x + ori.x, pos1.y + ori.y , pos1.z + ori.z);
	glVertex3f(pos1_u.x + ori.x, pos1_u.y + ori.y, pos1_u.z + ori.z);
	glEnd();

	//Back
	glBegin(GL_QUADS);
	glNormal3f(0, 0, 1);
	if (!doingShadow)
		glColor3ub(100, 0, 0);
	glVertex3f(pos0_u.x + ori.x, pos0_u.y + ori.y, pos0_u.z + ori.z);
	glVertex3f(pos0.x - ori.x, pos0.y - ori.y , pos0.z - ori.z);
	glVertex3f(pos0.x + ori.x, pos0.y + ori.y , pos0.z + ori.z);
	glVertex3f(pos0_u.x - ori.x, pos0_u.y - ori.y, pos0_u.z - ori.z);
	glEnd();


	if (head) {
		// another cube
	//Up
		glBegin(GL_QUADS);
		glNormal3f(0, 1, 0);
		if (!doingShadow)
			glColor3ub(0, 0, 255);
		glTranslatef(0.0f, 20.0f, 0.0f);

		glVertex3f(pos0_u.x +0.5*ori.x, pos0_u.y +  ori.y+10, pos0_u.z +  0.5*ori.z);
		glVertex3f(pos0_u.x - 0.5*ori.x, pos0_u.y -  ori.y+10, pos0_u.z -  0.5*ori.z);
		glVertex3f(pos1_u.x - 0.5*ori.x, pos1_u.y -  ori.y+10, pos1_u.z -  0.5*ori.z);
		glVertex3f(pos1_u.x + 0.5*ori.x, pos1_u.y +  ori.y+10, pos1_u.z +  0.5*ori.z);

		glEnd();

		//Left
		glBegin(GL_QUADS);
		glNormal3f(-1, 0, 0);
		if (!doingShadow)
			glColor3ub(100, 0, 0);
		glVertex3f(pos0_u.x - 0.5 * ori.x , pos0_u.y - ori.y + 10, pos0_u.z - 0.5 * ori.z);
		glVertex3f(pos1_u.x - 0.5 * ori.x, pos1_u.y - ori.y + 10, pos1_u.z - 0.5 * ori.z);
		glVertex3f(pos1.x - 0.5 * ori.x, pos1.y - ori.y + 10, pos1.z - 0.5 * ori.z);
		glVertex3f(pos0.x - 0.5 * ori.x , pos0.y - ori.y + 10, pos0.z - 0.5 * ori.z);
		glEnd();

		//Right
		glBegin(GL_QUADS);
		glNormal3f(1, 0, 0);
		if (!doingShadow)
			glColor3ub(100, 0, 0);

		glVertex3f(pos1_u.x + 0.5 * ori.x, pos1_u.y + ori.y + 10, pos1_u.z + 0.5 * ori.z);
		glVertex3f(pos0_u.x + 0.5 * ori.x , pos0_u.y + ori.y + 10, pos0_u.z + 0.5 * ori.z);
		glVertex3f(pos0.x + 0.5 * ori.x , pos0.y + ori.y + 10, pos0.z + 0.5 * ori.z);
		glVertex3f(pos1.x + 0.5 * ori.x, pos1.y + ori.y + 10, pos1.z + 0.5 * ori.z);

		glEnd();

		//Front
		glBegin(GL_QUADS);
		glNormal3f(0, 0, -1);
		if (!doingShadow)
			glColor3ub(100, 0, 0);
		glVertex3f(pos1_u.x - 0.5 * ori.x, pos1_u.y - ori.y + 10, pos1_u.z - 0.5 * ori.z);
		glVertex3f(pos1_u.x + 0.5 * ori.x, pos1_u.y + ori.y + 10, pos1_u.z + 0.5 * ori.z);
		glVertex3f(pos1.x + 0.5 * ori.x, pos1.y + ori.y + 10, pos1.z + 0.5 * ori.z);
		glVertex3f(pos1.x - 0.5 * ori.x, pos1.y - ori.y + 10, pos1.z - 0.5 * ori.z);

		glEnd();

		//Back
		glBegin(GL_QUADS);
		glNormal3f(0, 0, 1);
		if (!doingShadow)
			glColor3ub(100, 0, 0);
		glVertex3f(pos0_u.x + 0.5 * ori.x , pos0_u.y + ori.y + 10, pos0_u.z + 0.5 * ori.z);
		glVertex3f(pos0_u.x - 0.5 * ori.x , pos0_u.y - ori.y + 10, pos0_u.z - 0.5 * ori.z);
		glVertex3f(pos0.x - 0.5 * ori.x , pos0.y - ori.y + 10, pos0.z - 0.5 * ori.z);
		glVertex3f(pos0.x + 0.5 * ori.x , pos0.y + ori.y + 10, pos0.z + 0.5 * ori.z);
		glEnd();
	}
	

	//glTranslatef((point_list[(point_index + carInterval * i) % point_list.size()].x+ point_list[(point_index + carInterval * i + 20) % point_list.size()].x)/2,
	//	(point_list[(point_index + carInterval * i) % point_list.size()].y+ point_list[(point_index + carInterval * i + 20) % point_list.size()].y)/2+14,
	//	(point_list[(point_index + carInterval * i) % point_list.size()].z+ point_list[(point_index + carInterval * i + 20) % point_list.size()].z)/2);
	//glRotatef()
	glTranslatef(pos0.x , pos0.y +15, pos0.z );
	drawCube(Pnt3f(1, 2, 1), Pnt3f(1, -2, 1), Pnt3f(-1, -2, 1), Pnt3f(-1, 2, 1));
	drawCube(Pnt3f(2, 2, 1), Pnt3f(2, 4, 1), Pnt3f(-2, 4, 1), Pnt3f(-2, 2, 1));
	drawCube(Pnt3f(1, 0, 1), Pnt3f(1, 1, 1), Pnt3f(3, 2, 1), Pnt3f(3, 1, 1));
	drawCube(Pnt3f(-1, 0, 1), Pnt3f(-1, 1, 1), Pnt3f(-3, 2, 1), Pnt3f(-3, 1, 1));
	drawCube(Pnt3f(0, -2, 1), Pnt3f(-1, -2, 1), Pnt3f(-2, -4, 1), Pnt3f(-1, -4, 1));
	drawCube(Pnt3f(0, -2, 1), Pnt3f(1, -2, 1), Pnt3f(2, -4, 1), Pnt3f(1, -4, 1));
	glTranslatef(-pos0.x , -pos0.y -15, -pos0.z );
	//glTranslatef(-(point_list[(point_index + carInterval * i) % point_list.size()].x + point_list[(point_index + carInterval * i + 20) % point_list.size()].x) / 2,
	//	-(point_list[(point_index + carInterval * i) % point_list.size()].y + point_list[(point_index + carInterval * i + 20) % point_list.size()].y) / 2 - 14,
	//	-(point_list[(point_index + carInterval * i) % point_list.size()].z + point_list[(point_index + carInterval * i + 20) % point_list.size()].z) / 2);
	//glColor3ub(10, 10, 10);
}


void TrainView::
drawWheel() {
}

void TrainView::
drawModel() {
	Shader ourShader("model_loading.vs", "model_loading.fs");
	Model ourModel(FileSystem::getPath("backpack/backpack.obj"));
	// don't forget to enable shader before setting uniforms
	ourShader.use();

	// view/projection transformations
	glm::mat4 projection = glm::perspective(glm::radians(45.0f), (float)50 / (float)50, 0.1f, 100.0f);
	glm::mat4 view = glm::lookAt(glm::vec3(0.0f,0.0f,0.0f), glm::vec3(0.0f, 0.0f, 0.0f) + glm::vec3(0.0f, 1.0f, 0.0f), glm::vec3(0.0f, 0.0f, 0.0f));
	ourShader.setMat4("projection", projection);
	ourShader.setMat4("view", view);

	glm::mat4 model = glm::mat4(1.0f);
	model = glm::translate(model, glm::vec3(0.0f, 0.0f, 0.0f)); // translate it down so it's at the center of the scene
	model = glm::scale(model, glm::vec3(100.0f, 100.0f, 100.0f));	// it's a bit too big for our scene, so scale it down
	ourShader.setMat4("model", model);
	ourModel.Draw(ourShader);
	std::cout << "loaded\n";
}


void TrainView::drawTunnel() {
	
}
void TrainView::drawCube(Pnt3f p0, Pnt3f p1, Pnt3f p2, Pnt3f p3) {
	float x[4] = { p0.x, p1.x, p2.x, p3.x };
	float y[4] = { p0.y, p1.y, p2.y, p3.y };
	float depth = p0.z;

	glColor3ub(100, 100, 100);
	// front 
	glBegin(GL_QUADS);
	glNormal3f(0, 1, 0);
	for (int i = 0; i < 4; i++) 
		glVertex3f(x[i], y[i], depth);
	glEnd();
	// back
	glBegin(GL_QUADS);
	glNormal3f(0, -1, 0);
	for (int i = 0; i < 4; i++)
		glVertex3f(x[i], y[i], -depth);
	glEnd();
	//other
	glBegin(GL_QUADS);
	for (int i = 0; i < 4; i++) {
		glVertex3f(x[i], y[i], depth);
		glVertex3f(x[(i + 1) % 4], y[(i + 1) % 4], depth);
		glVertex3f(x[(i + 1) % 4], y[(i + 1) % 4], -depth);
		glVertex3f(x[i], y[i], -depth);
	}
	glEnd();
}
