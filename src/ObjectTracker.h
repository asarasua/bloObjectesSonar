//
//  ObjectTracker.h
//  bloObjectesKinect
//
//  Created by Álvaro Sarasúa Berodia on 11/12/13.
//
//

#ifndef bloObjectesKinect_ObjectTracker_h
#define bloObjectesKinect_ObjectTracker_h

#include "ofxCv.h"
#include "ofMain.h"
#include "ofxKinect.h"
#include "ofxOsc.h"

/*class Object{
protected:
    //cv::Point2f center;
    float area;
    unsigned int category;
    bool touched;
    
public:
    Object(){};
    Object(float Area);
    virtual ~Object(){};
    
    //cv::Point2f getCenter();
    
    void setArea (float _area);
    float getArea();
    
    void setCategory (unsigned int _category);
    unsigned int getCategory();
    
    void setTouched (bool _touched);
    bool isTouched ();
};*/

class VirtualObject{
protected:
    ofVec3f worlCenter;
    ofVec2f imgCenter;
    bool touchedCurr;
    bool touchedPrev;
    
public:
    VirtualObject(){};
    VirtualObject(ofVec3f _worldCenter, ofVec2f _imgCenter);
    virtual ~VirtualObject(){};
    
    ofVec3f getWorldCenter();
    ofVec2f getImgCenter();
    
    void setTouchedCurr(bool _touched);
    bool isTouchedCurr();
    void setTouchedPrev(bool _touched);
    bool isTouchedPrev();
};

class ObjectTracker{
public:
    ObjectTracker();
    virtual ~ObjectTracker(){};
    void update(ofxCv::ContourFinder& objectFinder, ofxKinect& kinect, ofVec3f background_n, ofVec3f background_v0);
    vector <VirtualObject> getVirtualObjects();
    void newVirtualObject(ofVec3f _worldCenter, ofVec2f _imgCenter);
    void clearVirtualObjects();
    void setVirtualObjectsRadius (float _radius);
 
private:
    void clearBundle();
	template <class T>
	void addMessage(string address, T data);
	void sendBundle();
    
    //map<unsigned int, Object> objects;
    vector<VirtualObject> virtualObjects;
    float virtualObjectsRadius;
    
    ofVec3f pointInBackground(ofVec3f realPoint, ofVec3f background_n, ofVec3f background_v0);
    float distanceToPlane(ofVec3f realPoint, ofVec3f background_n, ofVec3f background_v0);
    
    ofxOscSender oscSender;
    ofxOscBundle bundle;
    
};

#endif
