#pragma once

#include "ofMain.h"
#include "ofxGui.h"
#include "ofxKinect.h"
#include "ofxOpenCv.h"
#include "ofxReactionDiffusion.h"
#include "ofxProcessFFT.h"

class ofApp : public ofBaseApp{
    
public:
    void setup();
    void update();
    void draw();
    
    void keyPressed(int key);
    void keyReleased(int key);
    void mouseMoved(int x, int y );

    void mousePressed(int x, int y, int button);
    void mouseReleased(int x, int y, int button);
    void mouseEntered(int x, int y);
    void mouseExited(int x, int y);
    void windowResized(int w, int h);
    void dragEvent(ofDragInfo dragInfo);
    void gotMessage(ofMessage msg);
    
    void clear();
    ofxKinect kinect;
    
    
    ofxCvColorImage colorImg;
    
    ofxCvGrayscaleImage grayImage; // grayscale depth image
    ofxCvGrayscaleImage grayThreshNear; // the near thresholded image
    ofxCvGrayscaleImage grayThreshFar; // the far thresholded image

    ofxCvContourFinder contourFinder;
    
    ofxPanel gui;
    ofParameter<float> minArea, maxArea, threshold;

    vector<ofxCvBlob>& blobs = contourFinder.blobs;
    
    bool bThreshWithOpenCV;
    
    int nearThreshold;
    int farThreshold;

    int angle;
    
    ofxReactionDiffusion rd;
    
    ofxIntSlider mode;
    ofxFloatSlider passes;
    ofxButton clearButton;
    
    ofxGuiGroup gs;
    ofxFloatSlider feed, kill, Du, Dv;
    
    ofxGuiGroup fhn;
    ofxFloatSlider a0, a1, epsilon, delta, k1, k2, k3;
    
    ofxGuiGroup bz;
    ofxFloatSlider alpha, beta, gamma;
    
    ofxGuiGroup worm;
    ofxFloatSlider constA, constB;
    
    ofxPanel colors;
    ofxFloatColorSlider color1, color2, color3, color4, color5;
    ofxFloatSlider thresh1, thresh2, thresh3, thresh4, thresh5;
    
    ProcessFFT fft;
};
