/************************************************************************
     File:        TrainView.H

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

#pragma once

// Preclarify for preventing the compiler error
class TrainWindow;
class CTrack;


//#######################################################################
// TODO
// You might change the TrainView in order to add different objects to
// be drawn, or adjust the lighting, or ...
//#######################################################################


// The TrainView also needs to handle its user interface events.
// since this is an FLTk Window, we need to include FlTk Headers
// the pragmas turn off the warnings from FlTk - our code should have no warnings,
// but their code doesn't meet that standard
#pragma warning(push)
#pragma warning(disable:4312)
#pragma warning(disable:4311)
#include <Fl/Fl_Gl_Window.h>
#pragma warning(pop)

// this uses the old ArcBall Code
#include "Utilities/ArcBallCam.H"
#include "TrainWindow.H"
#include<iostream>
class TrainView : public Fl_Gl_Window
{
	public:
		// note that we keep the "standard widget" constructor arguments
		TrainView(int x, int y, int w, int h, const char* l = 0);

		// overrides of important window things
		virtual int handle(int);
		virtual void draw();

		// all of the actual drawing happens in this routine
		// it has to be encapsulated, since we draw differently if
		// we're drawing shadows (no colors, for example)
		void drawStuff(bool doingShadows=false);

		// setup the projection - assuming that the projection stack has been
		// cleared for you
		void setProjection();

		// Reset the Arc ball control
		void resetArcball();

		// pick a point (for when the mouse goes down)
		void doPick();

		//drawTrack
		void drawTrack(bool doingShadow);

		//drawTrain
		void drawTrain(bool doingShadow, Pnt3f pos0, Pnt3f pos1, Pnt3f ori, bool head = false);

		//drawWheel
		void drawWheel();
		//void drawWheel(bool doingShadow, Pnt3f pos0, Pnt3f pos1, Pnt3f ori);

		void drawModel();
		void drawTunnel();

		void drawCube(Pnt3f p0, Pnt3f p1, Pnt3f p2, Pnt3f p3);
	public:
		ArcBallCam		arcball;			// keep an ArcBall for the UI
		int				selectedCube;  // simple - just remember which cube is selected
		
		TrainWindow*	tw;				// The parent of this display window
		CTrack*			m_pTrack;		// The track of the entire scene


		//add by koach
		vector<Pnt3f> point_list;
		vector<Pnt3f> orient_list;
		int nonArcCnt = 0;
		int point_index = 0;
		int carAmount = 3;
		int carInterval = 30;
		float POINT_INTERVAL_LEN = 1.0f;
		float POINT_CNT_LEN = 0.0f;
		float RAIL_INTERVAL_LEN = 10.0f;
		float RAIL_WIDTH = 3.0f;
		float RAIL_CNT_LEN = 0.0f;
		float DIS = 0.0f;
		float DIVIDE_LINE = 1000.0f;

		// adaptive 
		bool adaptiveLock = false;
		bool firstLock = true;
		bool secondLock = true;
		Pnt3f lastTwoPoint;
		Pnt3f lastPoint;
		Pnt3f lastVec;
		Pnt3f nowVec;
		int cc = 0;
};