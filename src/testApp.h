#pragma once

#include "ofMain.h"
#include "ofxOpenCv.h"
#include "ofxCv.h"
#include "ofxKinect.h"
#include "ofxGui.h"
#include "ObjectTracker.h"

// uncomment this to read from two kinects simultaneously
//#define USE_TWO_KINECTS

class testApp : public ofBaseApp {
public:
	
	void setup();
	void update();
	void draw();
	void exit();
	
	void drawPointCloud();
	
	void keyPressed(int key);
	void mouseDragged(int x, int y, int button);
	void mousePressed(int x, int y, int button);
	void mouseReleased(int x, int y, int button);
	void windowResized(int w, int h);
    float distanceToBackground (int kinectX, int kinectY);
    	
	ofxKinect kinect;
	
#ifdef USE_TWO_KINECTS
	ofxKinect kinect2;
#endif
	
    ofxCvGrayscaleImage objectsImage;
	
    ofxCv::ContourFinder objectsFinder;
    
    ObjectTracker objectTracker;
    
    bool bDefiningPoints;
    
    bool bCalibratingBackground;
    vector<ofVec3f> backgroundPoints;
    ofVec3f background_n;
    ofVec3f background_v0;
	
	bool bThreshWithOpenCV;
	//bool bDrawDetectors;
    
    //GUI
    ofxPanel gui;
    ofxFloatSlider virtualObjectsRadius;
    ofxVec2Slider floorThresholdSlider;
    ofxVec2Slider objectsBlobSize;
    ofxToggle drawDetectors;
	
	int angle;
	
	// used for viewing the point cloud
	ofEasyCam easyCam;
};
