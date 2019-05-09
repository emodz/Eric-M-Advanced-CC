#include "ofApp.h"

//--------------------------------------------------------------
void ofApp::setup(){
    ofSetBackgroundColor(0);
    ofSetFrameRate(60);
    ofSetVerticalSync(true);
    fft.setup();
    
    rd.allocate(ofGetWidth(), ofGetHeight());
    
    gui.setup();
    gui.add(minArea.set("Min area", 10, 1, 100));
    gui.add(maxArea.set("Max area", 200, 1, 500));
    gui.add(threshold.set("Threshold", 128, 0, 255));
    
    gui.setName("Parameters");
    gui.add(mode.setup("Mode", 0, 0, 2));
    gui.add(passes.setup("Passes", rd.getPasses(), 0.01, 2));
    gui.add(clearButton.setup("Clear"));
    clearButton.addListener(this, &ofApp::clear);
    

    
    gui.add(fhn.setup());
    fhn.setName("Fitz-Hugh Nagumo");
    fhn.add(a0.setup("a0", rd.getA0(), 0, 1.0));
    fhn.add(a1.setup("a1", rd.getA1(), 0, 1.0));
    fhn.add(epsilon.setup("Epsilon", rd.getEpsilon(), 0, 1.5));
    fhn.add(delta.setup("Delta", rd.getDelta(), 0, 1.0));
    fhn.add(k1.setup("K1", rd.getK1(), 0.001, 3.0));
    fhn.add(k2.setup("K2", rd.getK2(), 0.001, 3.0));
    fhn.add(k3.setup("K3", rd.getK3(), 0.001, 3.0));
    

    colors.setup();
    colors.setPosition(ofGetWidth() - colors.getWidth(), 0);
    colors.setName("Colors");
    colors.add(color1.setup("Color 1", 0.0 , 0, 1.0));
    colors.add(thresh1.setup("Thresh 1", 0.0 , 0, 1.0));
    colors.add(color2.setup("Color 2", 0.25, 0, 1.0));
    colors.add(thresh2.setup("Thresh 2", 0.2 , 0, 1.0));
    colors.add(color3.setup("Color 3", 0.5 , 0, 1.0));
    colors.add(thresh3.setup("Thresh 3", 0.4 , 0, 1.0));
    colors.add(color4.setup("Color 4", 0.75, 0, 1.0));
    colors.add(thresh4.setup("Thresh 4", 0.6 , 0, 1.0));
    colors.add(color5.setup("Color 5", 1.0 , 0, 1.0));
    colors.add(thresh5.setup("Thresh 5", 0.8 , 0, 1.0));
    
    color1 = rd.getColor1();
    color2 = rd.getColor2();
    color3 = rd.getColor3();
    color4 = rd.getColor4();
    color5 = rd.getColor5();
    
    kinect.setRegistration(true);
    
    kinect.init();
    //kinect.init(true); // shows infrared instead of RGB video image
    //kinect.init(false, false); // disable video image (faster fps)
    
    kinect.open();        // opens first available kinect

    
    
    colorImg.allocate(kinect.width, kinect.height);
    grayImage.allocate(kinect.width, kinect.height);
    grayThreshNear.allocate(kinect.width, kinect.height);
    grayThreshFar.allocate(kinect.width, kinect.height);
    
    nearThreshold = 230;
    farThreshold = 200;
    bThreshWithOpenCV = true;
    
    // zero the tilt on startup
    angle = 0;
    kinect.setCameraTiltAngle(angle);
    
}

void ofApp::clear() {
    rd.clearAll();
}

//--------------------------------------------------------------
void ofApp::update(){
    rd.setPasses(passes);
    float low = ofMap(fft.getLowVal(), 0.0, 1.0, 0.0, 3.0);
    float mid = ofMap(fft.getMidVal(), 0.0, 1.0, 0.0, 1.0);
    float high = ofMap(fft.getHighVal(), 0.0, 1.0, 0.0, 1.0);

    fft.update();
    rd.setMode(RD_MODE_FITZHUGH_NAGUMO);
    rd.setFhnParams(a0, mid, high, low, low, k2, k3);
    
    
    
    switch (mode) {
        case 0:
            rd.setMode(RD_MODE_FITZHUGH_NAGUMO);
            rd.setFhnParams(a0, mid, high, low, low, k2, k3);
            break;
        case 1:
            rd.setMode(RD_MODE_FITZHUGH_NAGUMO);
            rd.setFhnParams(a0, mid, high, low, low, k2, k3);
            break;
        case 2:
            rd.setMode(RD_MODE_BELOUSOV_ZHABOTINSKY);
            rd.setBzParams(alpha, beta, gamma);
            break;
        default:
            break;
    }
    
    rd.setColor1(color1);
    rd.setColor1Threshold(thresh1);
    rd.setColor2(color2);
    rd.setColor2Threshold(thresh2);
    rd.setColor3(color3);
    rd.setColor3Threshold(thresh3);
    rd.setColor4(color4);
    rd.setColor4Threshold(thresh4);
    rd.setColor5(color5);
    rd.setColor5Threshold(thresh5);
    
    rd.update();
    
    ofBackground(100, 100, 100);
    
    kinect.update();
    contourFinder.findContours(grayImage, 10, (kinect.width*kinect.height)/2, 20, false);

    // load grayscale depth image from the kinect source
    grayImage.setFromPixels(kinect.getDepthPixels());
    
    if(bThreshWithOpenCV) {
        grayThreshNear = grayImage;
        grayThreshFar = grayImage;
        grayThreshNear.threshold(nearThreshold, true);
        grayThreshFar.threshold(farThreshold);
        cvAnd(grayThreshNear.getCvImage(), grayThreshFar.getCvImage(), grayImage.getCvImage(), NULL);
    } else {
        
        ofPixels & pix = grayImage.getPixels();
        int numPixels = pix.size();
        for(int i = 0; i < numPixels; i++) {
            if(pix[i] < nearThreshold && pix[i] > farThreshold) {
                pix[i] = 255;
            } else {
                pix[i] = 0;
            }
        }
    }
    
    // update the cv images
    grayImage.flagImageChanged();
    
}

//--------------------------------------------------------------
void ofApp::draw(){

    rd.draw();
    gui.draw();
    colors.draw();
    
    if (blobs.size() > 0) {
        fft.setNormalize(true);
        for (int i = 0; i < blobs[0].pts.size(); i++) {
            ofVec2f coordinate_i_ofcontour = blobs[0].pts[i];
            rd.addSource(ofMap(coordinate_i_ofcontour.x, 0, 640, 0, 1024), ofMap(coordinate_i_ofcontour.y, 0, 480, 0, 768),ofMap(fft.getLowVal(), 0.0, 1.0, 0.0, 40.0));
     
        }
    }
}

//--------------------------------------------------------------
void ofApp::keyPressed (int key) {
    switch (key) {
        case ' ':
            bThreshWithOpenCV = !bThreshWithOpenCV;
            break;
            
        case '>':
        case '.':
            farThreshold ++;
            if (farThreshold > 255) farThreshold = 255;
            break;
            
        case '<':
        case ',':
            farThreshold --;
            if (farThreshold < 0) farThreshold = 0;
            break;
            
        case '+':
        case '=':
            nearThreshold ++;
            if (nearThreshold > 255) nearThreshold = 255;
            break;
            
        case '-':
            nearThreshold --;
            if (nearThreshold < 0) nearThreshold = 0;
            break;
            
        case 'w':
            kinect.enableDepthNearValueWhite(!kinect.isDepthNearValueWhite());
            break;
            
        case 'o':
            kinect.setCameraTiltAngle(angle); // go back to prev tilt
            kinect.open();
            break;

        case 'c':
            kinect.setCameraTiltAngle(0); // zero the tilt
            kinect.close();
            break;
            
        case OF_KEY_UP:
            angle++;
            if(angle>30) angle=30;
            kinect.setCameraTiltAngle(angle);
            break;
            
        case OF_KEY_DOWN:
            angle--;
            if(angle<-30) angle=-30;
            kinect.setCameraTiltAngle(angle);
            break;
    }
}

//--------------------------------------------------------------
void ofApp::keyReleased(int key){
    
}

//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y ){
    
}



//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button){
    
}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button){
    
}

//--------------------------------------------------------------
void ofApp::mouseEntered(int x, int y){
    
}

//--------------------------------------------------------------
void ofApp::mouseExited(int x, int y){
    
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
