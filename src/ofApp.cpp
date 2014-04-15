#include "ofApp.h"

using namespace cv;
using namespace ofxCv;

//--------------------------------------------------------------
void ofApp::setup(){
    ofSetVerticalSync(true);
    ofSetWindowTitle("Receiver");
    ofSetWindowPosition(0, 0);
    ofLogLevel(OF_LOG_WARNING);

    
    // set drone port
    dronecam.setup(5555, droneHost);
    
    memset(keys, 0, sizeof(*keys));
    
    // connect to drone
    drone.connect();
    
    cameraFovY = 35;
    
    // setup command history lengths for debugging and dumping onscreen (OPTIONAL)
    drone.controller.commandHistory.setMaxLength(30);
    drone.dataReceiver.commandHistory.setMaxLength(30);
    
    // setup facefinder
    finder.setup("haarcascade_frontalface_alt2.xml");
    finder.setPreset(ObjectFinder::Accurate);
    finder.getTracker().setSmoothingRate(.2);
    finder.setRescale(.25);
    
//    droneSimulator.setup(&drone);

    
    starttime = ofGetElapsedTimeMillis();
    
    
    searchSpeed = 0.3;
    
    /*kalman filter
    KF.init(4, 2, 0);
    
    KF.transitionMatrix = *(Mat_<float>(4, 4) <<
                            1,0,1,0,
                            0,1,0,1,
                            0,0,1,0,
                            0,0,0,1);

    measurement = Mat_<float>::zeros(2,1);
    
	KF.statePre.at<float>(0) = fac.x; //mousex
	KF.statePre.at<float>(1) = fac.y; // mousey
	KF.statePre.at<float>(2) = 0;
	KF.statePre.at<float>(3) = 0;
    
    setIdentity(KF.measurementMatrix);
	setIdentity(KF.processNoiseCov, Scalar::all(1e-4));
	setIdentity(KF.measurementNoiseCov, Scalar::all(1e-1));
	setIdentity(KF.errorCovPost, Scalar::all(.1));*/
    
}

//--------------------------------------------------------------
void ofApp::update(){

    dronecam.update();
    
    if(dronecam.isConnected()){
    // face detection timer for searching new face
    float timer = ofGetElapsedTimeMillis() - starttime;
    
    // if no face has been detected for 2 seconds, start searching
    if (timer > 2000 && !searchMode) {
        searchMode = true;
        searchMode *= -1;
    }
    
    if(dronecam.isFrameNew()){
        //ofImage imgTemp = *dronecam.lastFrame;
        
        // set so that resize is done in openCV objectfinder
        //imgMat = toCv(imgTemp);
        
        //cv::resize(imgMat, imgMat, cv::Size(round(0.5*imgMat.cols), round(0.5*imgMat.rows)));
        //finder.update(imgMat);
        finder.update(toCv(*dronecam.lastFrame));
        pixelsPerDegree = dronecam.getHeight() / cameraFovY;
        horizon = (drone.state.getPitch()/1000) * pixelsPerDegree;
        ofLog(OF_LOG_NOTICE, ofToString(pixelsPerDegree));
        ofLog(OF_LOG_NOTICE, ofToString(horizon));
    }
    

    // if face detected
    if(finder.size() >= 1){
        stopauto = false;                       // autopilot is allowed
        starttime = ofGetElapsedTimeMillis();   // startime is current time
        searchMode = false;                     // turn searchmode off
    }
    // if face is not found for more than 150ms, stop autopilot
    else if(finder.size() == 0 && (ofGetElapsedTimeMillis() - starttime) < 200) {
        stopauto = false;
    }
    else stopauto = true;
    if (stopauto != setzero){
        if(stopauto){
            drone.controller.spinSpeed = 0;
            drone.controller.pitchAmount = 0;
            drone.controller.liftSpeed = 0;
        }
        setzero = stopauto;
    }
    

    
    // autopilot mode for drone only activate on new frame
    if(!stopauto && dronecam.isFrameNew()){
        // so far only for one face (0)
        if(finder.size() > 0){
        for(int i = 0; i < finder.size(); i++) {
            ofRectangle face = finder.getObjectSmoothed(finder.size()-1);
            
            fac.x = face.x + face.width/2;
            fac.y = face.y + face.height/2;
            fac.z = face.width;
            
            
            /* kalman filter useless
            // First predict, to update the internal statePre variable
            Mat prediction = KF.predict();
            cv::Point predictPt(prediction.at<float>(0),prediction.at<float>(1));
            point = toOf(predictPt);
            
            // Get mouse point
            measurement(0) = fac.x;
            measurement(1) = fac.y;
            
            cv::Point measPt(measurement(0),measurement(1));
            
            
            // The "correct" phase that is going to use the predicted value and our measurement
            Mat estimated = KF.correct(measurement);
            cv::Point statePt(estimated.at<float>(0),estimated.at<float>(1));
            
            //// end of kalman filter*/
            
            if(autopilot && !searchMode){
                // define spinspeed on position of face one the x axis
                //float spinSpd = ofMap((face.x + face.width) /2, 0, dronecam.getWidth()/4, -.8, .8);
                float spinSpd = ofMap(fac.x, 0, dronecam.getHeight(), -.6, .6);
                
                if((spinSpd < 0.06 && spinSpd > 0) || (spinSpd > - 0.06 && spinSpd < 0)){
                    spinSpd = 0;
                }
                
                //if face is small (far away), come closer
                if(face.width < 50){
                    drone.controller.pitchAmount = -0.06;
                }
                
                // if you're too close, back off
                else if(face.width > 90){
                    drone.controller.pitchAmount = 0.06;
                }
                
                else{
                    drone.controller.pitchAmount = 0;
                }
                
                // If you dive down, the drone will follow.
                float liftSpd = ofMap(fac.y, 0, imgMat.rows, .4, -.4);// - (drone.state.getVy()/2);
                if((liftSpd < 0.09 && liftSpd > 0) || (liftSpd > - 0.09 && liftSpd < 0)){
                    liftSpd = 0;
                }
                // send commands to controller
                drone.controller.spinSpeed = spinSpd;
                drone.controller.liftSpeed = liftSpd;
            }
        }
        }
    }
    
    // if in search mode, and autopilot, start looking for faces.
    else if (searchMode && autopilot)  {
        drone.controller.spinSpeed = searchSpeed;
    }
  
    }
    //Drone manual controller always on + overrule
    if(doPause) return;
    
    {
        float s = 0.1;
        
        if(keys[OF_KEY_UP]) drone.controller.pitchAmount -= s;
        else if(keys[OF_KEY_DOWN]) drone.controller.pitchAmount += s;
        
        if(keys['a']) drone.controller.spinSpeed -= s;
        else if(keys['d']) drone.controller.spinSpeed += s;
        
        if(keys['w']) drone.controller.liftSpeed += s;
        else if(keys['s']) drone.controller.liftSpeed -= s;
        
        if(keys[OF_KEY_LEFT]) drone.controller.rollAmount -= s;
        else if(keys[OF_KEY_RIGHT]) drone.controller.rollAmount += s;
        
    }
    
    // update the drone (process and send queued commands to drone, receive commands from drone and update state
    drone.update();

}



//--------------------------------------------------------------
void ofApp::draw(){
    ofBackground(255,255,255);

    ofSetHexColor(0xFFFFFF);

    if(dronecam.isConnected()){
        //drawMat(imgMat, 0, 0);
        dronecam.draw(0, 0);
        for(int i = 0; i < finder.size(); i++){
            ofPushMatrix();
            ofTranslate(fac.x, fac.y);
            ofSetColor(100,0,0);
            ofFill();
            ofCircle(0, 0, 20);
            ofPopMatrix();
        }
        ofPushMatrix();
        ofTranslate(dronecam.getWidth()/2, dronecam.getHeight()/2);
        ofCircle(0, horizon, 30);
        ofPopMatrix();
    }
    ofSetColor(0, 0, 0);
    int y = 15;
    int x = 650;
    
    ofDrawBitmapString("Streamer Receiver Example", 650, y);
    ofDrawBitmapString("Frame Num: \t\t"+ofToString(dronecam.frameNum), 650, y+=20);
    ofDrawBitmapString("Frame Rate: "+ofToString(dronecam.frameRate,1)+" fps", 650, y+=15);
    ofDrawBitmapString("bitrate: "+ofToString(dronecam.bitrate)+" kbits/s", 650, y+=15);
    ofDrawBitmapString("URL: "+dronecam.url, 650, y+=35);
    ofDrawBitmapString("Faces found = "+ofToString(finder.size()), 650, y+=35);
    ofDrawBitmapString("Search mode = "+ofToString(searchMode), 650, y+=35);
    ofDrawBitmapString("xF ="+ofToString(fac.x), 650, y+= 35);
    ofDrawBitmapString("xK ="+ofToString(point.x), 650, y+= 35);
    ofDrawBitmapString("yF ="+ofToString(fac.y), 650, y+= 35);
    ofDrawBitmapString("yK ="+ofToString(point.y), 650, y+= 35);
    ofDrawBitmapString("faceWidth ="+ofToString(fac.z), 650, y+= 35);
    
    
    // ARDRONE draw debug strings
    string controllerString = "fps: " + ofToString(ofGetFrameRate()) + "\n";
    controllerString += "millisSinceLastSend: " + ofToString(drone.controller.getMillisSinceLastSend()) + "\n";
    controllerString += "\n";
    controllerString += "takeOff (t)\n";
    controllerString += "land (l)\n";
    controllerString += "calibrateHorizontal (c)\n";
    controllerString += "calibrateMagnetometer (m)\n";
    controllerString += "EMERGENCY (E)\n";
    controllerString += "\n";
    controllerString += "roll (a/d)        : " + ofToString(drone.controller.rollAmount) + "\n";
    controllerString += "pitch (up/down)   : " + ofToString(drone.controller.pitchAmount) + "\n";
    controllerString += "lift (w/s)        : " + ofToString(drone.controller.liftSpeed) + "\n";
    controllerString += "spin (left/right) : " + ofToString(drone.controller.spinSpeed) + "\n";
    controllerString += "\n";
    controllerString += "reset droneSimulator (r)\n";
    controllerString += "debug history (h)\n";
    controllerString += "fullscreen (f)\n";
    controllerString += "PAUSE (p)\n";
    
    ofxARDrone::State &state = drone.state;
    string stateString = "";
    stateString += "isFlying : " + ofToString(state.isFlying()) + "\n";
    stateString += "isTakingOff : " + ofToString(state.isTakingOff()) + ", " + ofToString(state.isTakingOffMillis()) + "\n";
    stateString += "isLanding : " + ofToString(state.isLanding()) + ", " + ofToString(state.isLandingMillis()) + "\n";
    /*stateString += "isCalibratingHorizontal : " + ofToString(state.isCalibratingHorizontal()) + ", " + ofToString(state.isCalibratingHorizontalMillis()) + "\n";
     stateString += "isCalibratingMagnetometer : " + ofToString(state.isCalibratingMagnetometer()) + ", " + ofToString(state.isCalibratingMagnetometerMillis()) + "\n";
     */
    stateString += "Autopilot: " + ofToString(autopilot) + "\n";
    stateString += "\n\nisConnected: " + ofToString(state.isConnected()) + ", " + ofToString(state.isCalibratingMagnetometerMillis()) + "\n";
    stateString += "altitude: "+ ofToString(state.getAltitude())+"\n";
    stateString += "emergency state: "+ ofToString(state.inEmergencyMode())+"\n";
    stateString += "battery level: "+ ofToString(state.getBatteryPercentage())+"%\n";
    stateString += "vx: "+ ofToString(state.getVx())+" vy: "+ ofToString(state.getVy())+" vz: "+ ofToString(state.getVz())+"\n";
    stateString += "pitch: "+ ofToString(state.getPitch())+" roll: "+ ofToString(state.getRoll())+" rot: "+ ofToString(state.getYaw())+"\n";
    ofDrawBitmapString(controllerString, 650, y+=35);
    ofDrawBitmapString(stateString, 650, y+=300);
    
    ofDrawBitmapString(drone.controller.commandHistory.getAsString(), 10, 440);
    ofDrawBitmapString(drone.dataReceiver.commandHistory.getAsString("\n"), ofGetWidth()-300, 440);
    ////ARDRONE END

}

//--------------------------------------------------------------
void ofApp::exit(){
    dronecam.close();
}

void ofApp::startSequence(){
    ofxARDrone::State &state = drone.state;
    if(!state.isFlying()){
        drone.controller.calibrateHorizontal(!drone.state.isCalibratingHorizontal(), 3000);
        drone.controller.calibrateMagnetometer(!drone.state.isCalibratingMagnetometer(), 3000);
        drone.controller.takeOff(!state.isTakingOff(), 3000);
    }
    if(state.isFlying()){
        searchMode = true;
        while(state.getAltitude() < 1500){
            drone.controller.liftSpeed = 0.1;
        }
        
    }
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){
    switch(key) {
        case '1': drone.controller.exitBootstrap(); break;
        case '2': drone.controller.sendAck(); break;
        case '3': drone.dataReceiver.sendDummyPacket(); break;
        case '0': drone.controller.resetCommunicationWatchdog(); break;
            
        case 't': drone.controller.takeOff(!drone.state.isTakingOff(), 3000); break;
        case 'l': drone.controller.land(!drone.state.isLanding(), 3000); break;
        case 'c': drone.controller.calibrateHorizontal(!drone.state.isCalibratingHorizontal(), 3000); break;
        case 'm': drone.controller.calibrateMagnetometer(!drone.state.isCalibratingMagnetometer(), 3000); break;
        case 'p': doPause ^= true; break;
            
        case 'e': drone.controller.emergency(0); break;
        case 'E': drone.controller.emergency(1); break;
            

        case 'R': drone.resetSequenceNumber(); break;
            
        case 'h':
            drone.controller.commandHistory.setMaxLength(drone.controller.commandHistory.getMaxLength() ? 0 : (ofGetHeight()-280)/14);
            drone.dataReceiver.commandHistory.setMaxLength(drone.controller.commandHistory.getMaxLength());
            break;
            
//        case 'g': drone.controller.hovering(!drone.state.isHovering, 3000); break;
            
        case 'f': ofToggleFullscreen(); break;
//        case 'b': drone.controller.camera(1); break;
//        case 'n': drone.controller.camera(2); break;
        case 'o': autopilot = !autopilot;
            
            
    }
    keys[key] = true;
}

//--------------------------------------------------------------
void ofApp::keyReleased(int key){
    keys[key] = false;
    
    switch(key) {
        case OF_KEY_UP:
        case OF_KEY_DOWN:
            drone.controller.pitchAmount = 0;
            break;
            
        case OF_KEY_LEFT:
        case OF_KEY_RIGHT:
            drone.controller.rollAmount = 0;
            break;
            
        case 'w':
        case 's':
            drone.controller.liftSpeed = 0;
            break;
            
        case 'a':
        case 'd':
            drone.controller.spinSpeed = 0;
            break;
            
    }
    
}


//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y ){

}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h){

}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg){

}

//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo){ 

}
