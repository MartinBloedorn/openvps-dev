#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include <vector>
#include <string>
#include <algorithm>
#include <iostream>
#include <iterator>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>

#include "vpsstorage.h"

using namespace cv;
using namespace std;
using namespace vps;

// If defined, intrisics will be computed first, with all images collected
// on a single node.
#define COMPUTE_MONO_WITH_ALL

/* NOTES:
 * After writing this, the naming standard changed - but this code didn't.
 * So, a Capture Device (cap) is also a Node;
 * A Node's ID informs its absolute position (0 is CW of 1, and so on).
 * A Node's Device ID (dev) informs which camera/IP should be fetched,
 * e.g. dev = 1 -> /dev/video1
 */

// * * * * * * CONSTANTS  * * * * * * * * * * * * * * * * * * * * * * * * * * //

// Globals... Shame on me.
float squareSize = 0.028;      // Set this to your actual square size (in m)
int minChessboardsPerCap = 15; // Minimum amount of chessboards that need to be seen on each cap
int pairingRange = 2;
bool improveIntrinsics = false; // If using external intrisics files, decide to improve them

// Constants
const int capWidth  = 640;
const int capHeigth = 480;
const int minCheesboards4Calib = 5;  // Minimum amount of chessboards required for calibration
const float rmsErrThers = 20.0;      // Max allowed error for stereoCalibrate
const int timeBetwChessboars = 250;  // ms


// * * * * * * HELP  * * * * * * * * * * * * * * * * * * * * * * * * * * * * //
//{cap||}{c|conf.yml|}{w|8|}{h|6|}{i||}{n||}{l||}{help||}
static int print_help()
{
    cout << "openVPS CLI Setup\n\n";
    cout << "Usage - Live Mode:\n ";
    cout <<  "./openvps-cli-setup -l -cap=<cameras> -w=<board_width=8> -h=<board_height=6> -c=<conf_file=conf.yml>\n\n";
    cout << "help            Print this help.\n";
    cout << "w=<int>         Specify calibration board width (8).\n";
    cout << "h=<int>         Specify calibration board heigth (6).\n";
    cout << "l               Use live capture (live mode).\n";
    cout << "c=<string>      Output configuration file path/name.\n";
    cout << "cap=<int>       Array of capture devices' IDs (e.g. cap=0213).\n";
    cout << "cparams=<path>  Use existing calibraton for each node. Base path/name for files.\n";
    cout << "ii              Improve intrinsics provided by external files during calibration.\n";
    cout << "pr=<int>        Pairing range for auto pair detection. (2)\n";
    cout << "sz=<float>      Calibration board's square size (meters, 0.028).\n";
    cout << "min=<int>       Minimum amount of calibration board views per camera. (15)\n";

    cout << "\nWhile running live mode:\n";
    cout << "s          Start calibration.\n";
    cout << "l          Arrange windows in linear fashion.\n";
    cout << "a          Arrange windows in grid fashion.\n";
    return 0;
}

// * * * * * * STEREO CALIB FUNCTION * * * * * * * * * * * * * * * * * * * * //

static void multiStereoCalib(vector<VideoCapture> & vCap, vector<int> vCapID,  Size boardSize,
                             bool displayCorners = false, string baseName = string(), bool useLive=false, int nImages=-1, string conffilepath = string(),
                             bool useIntrisicFiles = false, vector<Mat> eCamMat = vector<Mat>(), vector<Mat> eDistCoeffs = vector<Mat>(),
                             vector<double> eRmsErr = vector<double>())
{

    // USED VARIABLES <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
    // <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
    int nCap = vCapID.size();
    bool wrapCap = false, arrangeGrid = false, arrangeGridLinear = false;
    int nBoardSquares = boardSize.height*boardSize.width;
    int nPairs = pairingRange * nCap;
    if(!wrapCap) nPairs = (nPairs/2) + (nPairs%2) + 1; // Use rounded-up half, if pairs don't wrap around
    cout << "Considering " << nPairs << " possible pairs\n";

    int * chessboardsPerCap = new int[nCap];
    for(int i=0; i < nCap; i++) chessboardsPerCap[i] = minChessboardsPerCap;

    bool keepRunning = true;    // Gets set to false when all elements in chessboardsPerCap == 0

    // One vector of vectors for each capture node
    vector<Mat> frame;
    vector<vector<Point2f> > corners;               // Caps< Pts_in_img <> >
    vector<vector<vector<Point2f> > > imagePoints;  // Caps< Imgs < Pts_in_img <> > >
    vector<vector<vector<Point3f> > > objectPoints; // Caps< Imgs < Pts_in_obj <> > >
    Size imageSize;

    frame.resize(nCap);
    corners.resize(nCap);
    imagePoints.resize(nCap);
    objectPoints.resize(nCap);

    bool * found = new bool(nCap);
    vector<bool> done(nCap, false);
    bool startCalibration = !useLive;
    int  fImageCount = nImages;

    // CAPTURE AND DETECT LOOP <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
    // <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
    while(keepRunning) {
        int nFound = 0;
        fImageCount--; // start at 0, so begin decrementing
        // Get frames from all capture devices (or files)
        for(int i = 0; i < nCap; i++) {
            found[i] = false;

            if(useLive) {
                vCap[i].read(frame[i]);
                if(!frame[i].data) break;
            } else {
                string fileName = baseName + to_string(i) + "_" + to_string(fImageCount) + ".jpg";
                frame[i] = imread(fileName, CV_LOAD_IMAGE_COLOR);
                if(!frame[i].data) {
                    cout << "Can't read " << fileName << ", skipping\n";
                    break;
                }
            }

            if(imageSize == Size())
                imageSize = frame[i].size();

            if(startCalibration)
                found[i] = findChessboardCorners(frame[i], boardSize, corners[i], CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE | CALIB_CB_FAST_CHECK );
            else
                putText(frame[i], "Press 's' to start", Point2f(50,50), cv::FONT_HERSHEY_SIMPLEX, 1.0, Scalar(0,255,0), 2);

            if(useLive) {
                string winName;
                winName += "Node " + to_string(i) + " (Cap " + to_string(vCapID[i]) + ")";
                if(displayCorners && startCalibration)
                    drawChessboardCorners(frame[i], boardSize, corners[i], found[i]);
                if(done[i] && startCalibration) // Notify that node is ok
                    putText(frame[i], "Ok!", Point2f(50,50), cv::FONT_HERSHEY_SIMPLEX, 1.0, Scalar(0,255,0), 2);
                imshow(winName, frame[i]);

                if(arrangeGrid) {
                    arrangeGrid = (i != nCap-1);
                    moveWindow(winName, 150 + 700*(i/2), 20 + 520*(i%2));
                }
                if(arrangeGridLinear) {
                    arrangeGridLinear = (i != nCap-1);
                    moveWindow(winName, 520 + 750*i, 20);
                }
            }

            if(found[i]) nFound++;
        }
        // If there's at least one pair, archive the corners
        if(nFound >= 2) {
            keepRunning = false;
            for(int i = 0; i < nCap; i++) {
                if(found[i]) {
                    cvtColor(frame[i], frame[i], COLOR_BGR2GRAY);
                    cornerSubPix(frame[i], corners[i], Size(11,11), Size(-1,-1), TermCriteria(TermCriteria::COUNT+TermCriteria::EPS, 30, 0.01));
                }
                imagePoints[i].push_back(corners[i]);
                if(useLive) {
                    if(found[i]) chessboardsPerCap[i]--;
                    keepRunning |= (chessboardsPerCap[i] > 0);
                    if(chessboardsPerCap[i] > 0) cout << "Still " << chessboardsPerCap[i] << " on " << i << "\n"; else cout << "Done on " << i << "\n";
                    done[i] = chessboardsPerCap[i] <= 0;
                }
                // Hang on a second before capturing next frame...
                waitKey(timeBetwChessboars);
            }
        }

        // Hit 's' to begin calibration
        int keyIn = waitKey(30);
        if(keyIn >= 0) {
            if(keyIn == 's') startCalibration = true;
            else if(keyIn == 'a') arrangeGrid = true;
            else if(keyIn == 'l') arrangeGridLinear = true;
            else break; }

        if(!useLive) keepRunning = !(fImageCount == 0);

        if(!keepRunning) {
            destroyAllWindows();
            cout << "Saw all required chessboards!\n";
            cout << "Collected " << imagePoints[0].size() << " points\n";
        }
    }

    // PROCESS POINTS <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
    // <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
    if(useLive) nImages = imagePoints[0].size();
    if(nImages < nCap * minCheesboards4Calib) {
        cout << "Insufficient images, aborting\n";
        return;
    }

    //cout << "Processing collected points...\n";

    vector<pair_t> pairs;
    pairs.resize(nPairs);
    vector<node_t> nodes;
    node_t defaultNode; defaultNode.a = 0.;
    nodes.resize(nCap, defaultNode);

    vector<Point3f> boardPts;
    for(int j = 0; j < boardSize.height; j++ )
        for(int k = 0; k < boardSize.width; k++ )
            boardPts.push_back(Point3f(k*squareSize, j*squareSize, 0.0));

#ifdef COMPUTE_MONO_WITH_ALL
    for(int iC = 0; iC < nCap; iC++) {
        cout << "Performing monocular calibration...\n";
        if(!useIntrisicFiles) {
            vector<vector<Point2f> > ipts;
            vector<vector<Point3f> > opts;
            int nValid = 0;
            for(int iI = 0; iI < nImages; iI++)
                if(!imagePoints[iC][iI].empty() && (imagePoints[iC][iI].size() == nBoardSquares)) {
                    InputArray & imgin = imagePoints[iC][iI];
                    InputArray & optin = boardPts;
                    // Required black magic...
                    if(imgin.getMat().checkVector(2, CV_32F) == optin.getMat().checkVector(3, CV_32F)) {
                        ipts.push_back(imagePoints[iC][iI]);
                        opts.push_back(boardPts);
                        nValid++;
                    }
                }
            nodes[iC].rmsErr = calibrateCamera(opts, ipts, imageSize, nodes[iC].camMat,
                                               nodes[iC].distCoeffs, noArray(), noArray());
            nodes[iC].id     = iC;
            nodes[iC].devId  = vCapID[iC];
            nodes[iC].a      = 0.;
            nodes[iC].width  = capWidth;
            nodes[iC].heigth = capHeigth;
            cout << "Calibrated node " << iC << " with " << nValid << " images (rmsErr = " << nodes[iC].rmsErr << ")\n";
        } else {
            nodes[iC].rmsErr = eRmsErr[iC];
            nodes[iC].id     = iC;
            nodes[iC].devId  = vCapID[iC];
            nodes[iC].a      = 0.;
            nodes[iC].width  = capWidth;
            nodes[iC].heigth = capHeigth;
            eCamMat[iC].copyTo(nodes[iC].camMat);
            eDistCoeffs[iC].copyTo(nodes[iC].distCoeffs);
        }
    }
#endif

    cout << "Performing stereo calibration...\n";
    // Verify valid pairs, calibrate stereo correspondence amongst them
    for(int iC = 0; iC < nCap; iC++) {
        for(int iP = 1; iP <= pairingRange; iP++) {
            // Index of adjacent Cap
            int adjIdx = (wrapCap? (iP+iC)%nCap :
                                   (iP+iC < nCap)? iP+iC : nCap-1);
            if(iC == adjIdx) break;

            int pIdx = iC*pairingRange + (iP-1);
            int nValidImgs = 0;
            vector<vector<Point2f> > ipts[2]; // Buffer for points used for calibration of caps in a pair
            vector<vector<Point3f> > opts;

            // Count valid samples per cap[iC] and cap[adjIdx], push back corresponding object points
            for(int iI = 0; iI < nImages; iI++)
                if(!imagePoints[iC][iI].empty() && !imagePoints[adjIdx][iI].empty() &&
                        (!imagePoints[iC][iI].size() == !imagePoints[adjIdx][iI].size()) && (imagePoints[iC][iI].size() == nBoardSquares)) {
                    InputArray & imgin0 = imagePoints[iC][iI];
                    InputArray & imgin1 = imagePoints[adjIdx][iI];
                    InputArray & optsin = boardPts;
                    // Required black magic...
                    if(imgin0.getMat().checkVector(2, CV_32F) == optsin.getMat().checkVector(3, CV_32F) &&
                            imgin1.getMat().checkVector(2, CV_32F) == optsin.getMat().checkVector(3, CV_32F)) {
                        ipts[0].push_back(imagePoints[iC][iI]);
                        ipts[1].push_back(imagePoints[adjIdx][iI]);
                        opts.push_back(boardPts);
                        nValidImgs++;
                    } else
                        cout << "Skipping bad input vector...\n";
                }

            cout << "\nFound " << nValidImgs << " valid images for pair " << iC << "-" << adjIdx << "\n";
            pairs[pIdx].valid = nValidImgs > minCheesboards4Calib;

            // PERFORM MONO AND STEREO CALIBRATION <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
            // <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
            if(pairs[pIdx].valid) {
                cout << "Pair " << iC << "-" << adjIdx << " (pIdx = " << pIdx << ") is valid; calibrating...\n";
                pairs[pIdx].id   = pIdx;
                pairs[pIdx].n[0] = iC;
                pairs[pIdx].n[1] = adjIdx;
                float rmsErr[2];
#ifndef COMPUTE_MONO_WITH_ALL
                Mat camMat[2], distCoeffs[2];
                rmsErr[0] = calibrateCamera(opts, ipts[0], imageSize, camMat[0], distCoeffs[0], noArray(), noArray());
                rmsErr[1] = calibrateCamera(opts, ipts[1], imageSize, camMat[1], distCoeffs[1], noArray(), noArray());
                pairs[pIdx].rmsErr = stereoCalibrate(opts, ipts[0], ipts[1],
                        camMat[0], distCoeffs[0],
                        camMat[1], distCoeffs[1],
        #else
                pairs[pIdx].rmsErr = stereoCalibrate(opts, ipts[0], ipts[1],
                        nodes[iC].camMat,     nodes[iC].distCoeffs,
                        nodes[adjIdx].camMat, nodes[adjIdx].distCoeffs,
        #endif
                        imageSize, pairs[pIdx].R, pairs[pIdx].T,
                        pairs[pIdx].E, pairs[pIdx].F,
                        CALIB_FIX_ASPECT_RATIO +
                        CALIB_ZERO_TANGENT_DIST +
                        //(useIntrisicFiles && !improveIntrinsics? CALIB_FIX_INTRINSIC : CALIB_USE_INTRINSIC_GUESS) +
                        CALIB_FIX_INTRINSIC +
                        CALIB_SAME_FOCAL_LENGTH +
                        CALIB_RATIONAL_MODEL +
                        CALIB_FIX_K3 + CALIB_FIX_K4 + CALIB_FIX_K5,
                        TermCriteria(TermCriteria::COUNT+TermCriteria::EPS, 100, 1e-5) );
                cout << "Done with RMS ERR = " << pairs[pIdx].rmsErr << "\n";

                if(pairs[pIdx].rmsErr > rmsErrThers) {
                    cout << "RMS error too large! Discarding pair\n";
                    pairs[pIdx].valid = false;
                    break;
                }

#ifndef COMPUTE_MONO_WITH_ALL
                // Update camera's intrisics and distortion coeffs if accuracy is improved
                float _a[2];
                _a[0] = pow(1. + (float)nImages/rmsErr[0], 2.);
                _a[1] = pow(1. + (float)nImages/rmsErr[1], 2.);
                if(_a[0] > nodes[iC].a) {
                    nodes[iC].id     = iC;
                    nodes[iC].devId  = vCapID[iC];
                    nodes[iC].a      = _a[0];
                    nodes[iC].rmsErr = rmsErr[0];
                    distCoeffs[0].copyTo(nodes[iC].distCoeffs);
                    camMat[0].copyTo(nodes[iC].camMat);
                    nodes[iC].width   = capWidth;
                    nodes[iC].heigth  = capHeigth;
                    cout << "Updated Node " << iC << " (a = " << _a[0] << ")\n";
                }
                if(_a[1] > nodes[adjIdx].a) {
                    nodes[adjIdx].id     = adjIdx;
                    nodes[adjIdx].devId  = vCapID[adjIdx];
                    nodes[adjIdx].a      = _a[1];
                    nodes[adjIdx].rmsErr = rmsErr[1];
                    distCoeffs[1].copyTo(nodes[adjIdx].distCoeffs);
                    camMat[1].copyTo(nodes[adjIdx].camMat);
                    nodes[adjIdx].width   = capWidth;
                    nodes[adjIdx].heigth  = capHeigth;
                    cout << "Updated Node " << adjIdx << " (a = " << _a[1] << ")\n";
                }
#endif
            }

            if(!wrapCap && adjIdx == nCap-1) break;
        }
    }

    // SAVE RESULTS <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
    // <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
    FileStorage fs(conffilepath, FileStorage::WRITE);
    if(!fs.isOpened()) { cout << "Can't write configuration file!\n"; return; }

    fs << "vCapID" << vCapID;
    fs << "pairingRange" << pairingRange;
    fs << "wrapCap" << wrapCap;
    fs << "nPairs"  << nPairs;
    fs << "nNodes"  << nCap;
    fs << "squareSize" << squareSize;

    for(int iC = 0; iC < nCap; iC++) {
        string nodeName;
        nodeName += "node_" + to_string(iC);
        fs << nodeName << nodes[iC];
    }
    for(int iP = 0; iP < nPairs; iP++) {
        string pairName;
        pairName += "pair_" + to_string(iP);
        fs << pairName << pairs[iP];
    }
    fs.release();
}


// * * * * * * MAIN  * * * * * * * * * * * * * * * * * * * * * * * * * * * * //

int main(int argc, char** argv) {
    vector<int> vCapID; //= {1,2,0,3};
    vector<VideoCapture> vCap;

    Size boardSize;
    string baseName, confFilePath;
    bool useLive;
    int nImages = -1, nCap;

    cv::CommandLineParser parser(argc, argv, "{cap||}{c|conf.yml|}{w|8|}{h|6|}{i||}{n||}{l||}{min||}{pr||}{sz||}{help||}{cparams||}{ii||}");
    if (!parser.check()) {
        parser.printErrors();
        return 1;
    }
    if (parser.has("help") || (!parser.has("i") && !parser.has("l"))) return print_help();

    // Changing some globals...
    squareSize   = (parser.has("sz")? parser.get<float>("sz") : squareSize);
    pairingRange = (parser.has("pr")? parser.get<int>("pr") : pairingRange);
    minChessboardsPerCap = (parser.has("min")? parser.get<int>("min") : minChessboardsPerCap);
    improveIntrinsics = parser.has("ii");

    boardSize.width = parser.get<int>("w");
    boardSize.height = parser.get<int>("h");
    confFilePath = parser.get<string>("c");
    useLive = parser.has("l");

    if(useLive) {
        if(!parser.has("cap")) {
            cout << "Specify capture devices! (e.g., -cap=0123)\n";
            return print_help();
        }
        string capstr = parser.get<string>("cap");
        for(int iS=0; iS < capstr.length(); iS++)
            vCapID.push_back((int)(capstr.c_str()[iS] - '0'));
    } else if(parser.has("i")) {
        if(!parser.has("n")) {
            cout << "Specify how many images are to be read! (e.g., -n=10)\n";
            return print_help();
        }
        cout << "Using image base name = " << parser.get<string>("i") << "\n";
        baseName = parser.get<string>("i");
        nImages = parser.get<int>("n");
    } else return print_help();

    nCap = vCapID.size();
    vCap.resize(nCap);

    // If required, read intrisics from external files
    bool useExtCamParams = parser.has("cparams");
    vector<Mat> distCoeffs(nCap), camMat(nCap);
    vector<double> eRmsErr(nCap);

    if(useExtCamParams) {
        string iconfBaseName = parser.get<string>("cparams");
        for(int i=0; i<nCap; i++) {
            string iconfFileName = iconfBaseName + to_string(i) + ".yml";
            FileStorage fs(iconfFileName, FileStorage::READ);
            cout << "Reading intrinsics: " << iconfFileName << "\n";
            if(fs["distortion_coefficients"].empty() || fs["camera_matrix"].empty() || fs["avg_reprojection_error"].empty() ) {
                cout << "Invalid configuration file for " << iconfBaseName << to_string(i) << ". Aborting.\n";
                exit(-1);
            }
            fs["distortion_coefficients"] >> distCoeffs[i];
            fs["camera_matrix"] >> camMat[i];
            eRmsErr[i] = (double)fs["avg_reprojection_error"];
            fs.release();
        }
    }

    // Initialize capture devices
    if(useLive) {
        cout << "Using live mode!\n";
        for(size_t i = 0; i < nCap; i++)  {
            if(!vCap[i].open(vCapID[i])) {
                cout << "Could not open capture " << vCapID[i] << "; aborting!\n";
                for(size_t j = 0; j <= i; j++) vCap[j].release();
                return -1;
            }
            cout << "/dev/video" << vCapID[i] << ": ok\n";
            //cout << "Mode: " << vCap[i].get(CV_CAP_PROP_MODE) << "\n";
            vCap[i].set(CV_CAP_PROP_FPS, 15);
            vCap[i].set(CV_CAP_PROP_FRAME_WIDTH,  capWidth);
            vCap[i].set(CV_CAP_PROP_FRAME_HEIGHT, capHeigth);
        }
    }

    multiStereoCalib(vCap, vCapID, boardSize, true, baseName, useLive, nImages, confFilePath, useExtCamParams, camMat, distCoeffs, eRmsErr);
    destroyAllWindows();

    // Release cap devs
    if(useLive) {
        cout << "Releasing...\n";
        for(size_t i = 0; i < nCap; i++) vCap[i].release();
    }

    return 0;
}
