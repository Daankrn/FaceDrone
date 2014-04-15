#pragma once

#include "ofMain.h"
#include "ofxStreamer.h"
#include "ofxARDrone.h"
#include "ofxOpenCv.h"
#include "ofxCv.h"


class ofApp : public ofBaseApp{

	public:
		void setup();
		void update();
		void draw();
        void exit();
    
		void keyPressed(int key);
		void keyReleased(int key);
		void mouseMoved(int x, int y );
		void mouseDragged(int x, int y, int button);
		void mousePressed(int x, int y, int button);
		void mouseReleased(int x, int y, int button);
		void windowResized(int w, int h);
		void dragEvent(ofDragInfo dragInfo);
		void gotMessage(ofMessage msg);
        void startSequence();
    
        ofxStreamerReceiver dronecam;
        ofxCv::ObjectFinder finder;
    cv::Mat imgMat;
    cv::KalmanFilter KF;
	cv::Mat_<float> measurement;
	ofVec2f point;
    ofVec3f fac;

   
    ofxARDrone::Drone drone;                // the main big daddy class
    ofxARDrone::Simulator droneSimulator;   // for displaying on screen (OPTIONAL)
    ofEasyCam   easycam;
    bool doPause;
    
    
    bool video;
    bool keys[65535];
    bool autopilot;
    bool stopauto;
    bool setzero;
    bool searchMode;
    float searchSpeed;
    float starttime;
    float pixelsPerDegree;
    float cameraFovY;
    float horizon;
    
    string testHost = "udp://localhost";
    string droneHost = "tcp://192.168.1.1";


};
