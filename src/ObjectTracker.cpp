//
//  ObjectTracker.cpp
//  bloObjectesKinect
//
//  Created by Álvaro Sarasúa Berodia on 11/12/13.
//
//

#include "ObjectTracker.h"

//class:OBJECT
/*Object::Object(float _area){
    area = _area;
    touched = false;
}

float Object::getArea(){
    return area;
}

void Object::setArea(float _area){
    area = _area;
}

unsigned int Object::getCategory(){
    return category;
}

void Object::setCategory(unsigned int _category){
    category = _category;
}

void Object::setTouched (bool _touched){
    touched = _touched;
}

bool Object::isTouched (){
    return touched;
}*/

//class:VIRTUALOBJECT
VirtualObject::VirtualObject(ofVec3f _worldCenter, ofVec2f _imgCenter){
    worlCenter = _worldCenter;
    imgCenter = _imgCenter;
    touchedCurr = false;
    touchedPrev = false;
}

void VirtualObject::setTouchedCurr(bool _touched){
    touchedCurr = _touched;
}

bool VirtualObject::isTouchedCurr(){
    return touchedCurr;
}

void VirtualObject::setTouchedPrev(bool _touched){
    touchedPrev = _touched;
}

bool VirtualObject::isTouchedPrev(){
    return touchedPrev;
}

ofVec3f VirtualObject::getWorldCenter(){
    return worlCenter;
}

ofVec2f VirtualObject::getImgCenter(){
    return imgCenter;
}

//class:OBJECTTRACKER

ObjectTracker::ObjectTracker(){
    oscSender.setup("localhost", 9000);
    virtualObjectsRadius = 100;
}

//_________________________________________________________________________OSC
void ObjectTracker::clearBundle() {
	bundle.clear();
}

template <>
void ObjectTracker::addMessage(string address, ofVec3f data) {
	ofxOscMessage msg;
	msg.setAddress(address);
	msg.addFloatArg(data.x);
	msg.addFloatArg(data.y);
	msg.addFloatArg(data.z);
	bundle.addMessage(msg);
}

template <>
void ObjectTracker::addMessage(string address, ofVec2f data) {
	ofxOscMessage msg;
	msg.setAddress(address);
	msg.addFloatArg(data.x);
	msg.addFloatArg(data.y);
	bundle.addMessage(msg);
}

template <>
void ObjectTracker::addMessage(string address, float data) {
	ofxOscMessage msg;
	msg.setAddress(address);
	msg.addFloatArg(data);
	bundle.addMessage(msg);
}

template <>
void ObjectTracker::addMessage(string address, int data) {
	ofxOscMessage msg;
	msg.setAddress(address);
	msg.addIntArg(data);
	bundle.addMessage(msg);
}

template <>
void ObjectTracker::addMessage(string address, unsigned int data) {
	ofxOscMessage msg;
	msg.setAddress(address);
	msg.addIntArg(data);
	bundle.addMessage(msg);
}

template <>
void ObjectTracker::addMessage(string address, string data) {
	ofxOscMessage msg;
	msg.setAddress(address);
	msg.addStringArg(data);
	bundle.addMessage(msg);
}

template <>
void ObjectTracker::addMessage(string address, bool data) {
	ofxOscMessage msg;
	msg.setAddress(address);
	if (data) {
        msg.addIntArg(1);
    } else {
        msg.addIntArg(0);
    }
	bundle.addMessage(msg);
}

void ObjectTracker::sendBundle() {
	oscSender.sendBundle(bundle);
}

//_________________________________________________________________________________

void ObjectTracker::update(ofxCv::ContourFinder& objectFinder, ofxKinect& kinect, ofVec3f background_n, ofVec3f background_v0){
    clearBundle();
    
    for (int i = 0; i < virtualObjects.size(); ++i) {
        //Set previous state to last current state and set current hypothesis to not touched
        virtualObjects[i].setTouchedPrev(virtualObjects[i].isTouchedCurr());
        virtualObjects[i].setTouchedCurr(false);
        
        //Check if it's touched now
        int j = 0;
        while (j < objectFinder.size() && !virtualObjects[i].isTouchedCurr()) {
            cv::Point2f center = objectFinder.getCenter(j);
            ofVec3f pointInBackbround = pointInBackground(kinect.getWorldCoordinateAt(center.x, center.y), background_n, background_v0);
            float distance = virtualObjects[i].getWorldCenter().distance(pointInBackbround);
            virtualObjects[i].setTouchedCurr(distance < virtualObjectsRadius);
            j++;
        }
        
        //Now we decide if we have to send a message
        //Wasn't touched, now is
        if (!virtualObjects[i].isTouchedPrev() && virtualObjects[i].isTouchedCurr()) {
            ofxOscMessage msg;
            msg.setAddress("/touchedObject");
            msg.addIntArg(i);
            bundle.addMessage(msg);
        }
        //Was touched, now isn't
        else if (virtualObjects[i].isTouchedPrev() && !virtualObjects[i].isTouchedCurr()) {
            ofxOscMessage msg;
            msg.setAddress("/untouchedObject");
            msg.addIntArg(i);
            bundle.addMessage(msg);
        }
    }
    
    //Send messages just if we have messages to send
    if (bundle.getMessageCount()) {
        sendBundle();
    }
}

vector<VirtualObject> ObjectTracker::getVirtualObjects(){
    return virtualObjects;
}

void ObjectTracker::clearVirtualObjects(){
    virtualObjects.clear();
}

void ObjectTracker::newVirtualObject(ofVec3f _worldCenter, ofVec2f _imgCenter){
    VirtualObject newVirtualObject = VirtualObject(_worldCenter, _imgCenter);
    virtualObjects.push_back(newVirtualObject);
}

void ObjectTracker::setVirtualObjectsRadius(float _radius){
    virtualObjectsRadius = _radius;
}

ofVec3f ObjectTracker::pointInBackground(ofVec3f realPoint, ofVec3f background_n, ofVec3f background_v0){
    return realPoint + distanceToPlane(realPoint, background_n, background_v0) * background_n;
}

float ObjectTracker::distanceToPlane(ofVec3f realPoint, ofVec3f background_n, ofVec3f background_v0){
    return abs(background_n.dot(realPoint-background_v0));
}
