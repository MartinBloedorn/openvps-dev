#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/aruco.hpp"

#include <vector>
#include <thread>
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

typedef struct {
    vector<int>     vCapID;
    int             pairingRange;
    bool            wrapCap;
    int             nPairs;
    int             nNodes;
    double          squareSize;
    //
    bool            hasExto;
} confparams_t;

// * * * * * * CONSTANTS  * * * * * * * * * * * * * * * * * * * * * * * * * * //
const double defaultSquareSize = 0.028; //m
const int    arucoMarkerId     = 1;
const int    arucoMarkerDict   = aruco::DICT_6X6_250;
const int    arucoMarkerSize   = 500;   // Pixel size at creation
const float  arucoMarkerLength = 0.124; // Length at detection

const int    extoNumOfSamples  = 40;

// * * * * * * HELP  * * * * * * * * * * * * * * * * * * * * * * * * * * * * //
// "{@c||}{gm||}{id||}{sz||}{out||}{help||}"
static int print_help()
{
    cout << "openVPS CLI Exto - External Origin Creator\n\n";
    cout << "Usage:\n./openvps-cli-exto conf_file_in.yml <-out=conf_file_out.yml>\n";
    return 0;
}

static void generateCalibrationMarker(string filename, int id) {
    Mat marker;
    aruco::Dictionary dictionary = aruco::getPredefinedDictionary((aruco::PREDEFINED_DICTIONARY_NAME)arucoMarkerDict);
    aruco::drawMarker(dictionary, id, arucoMarkerSize, marker, 1);
    if(imwrite(filename, marker))
        cout << "Saved marker to " << filename << "\n";
    else
        cout << "Unable to save to " << filename << "\n";
    return;
}

static void readConfigurationFile(string filename, vector<node_t>& node, vector<pair_t>& pair, confparams_t& conf) {
    FileStorage fs(filename, FileStorage::READ);
    if(!fs.isOpened() || fs["vCapID"].empty() || fs["nPairs"].empty() || fs["nNodes"].empty()) {
        cout << "Invalid file : " << filename << " . Aborting!\n";
        exit(-1);
    }
    cout << "Reading " << filename << "...\n";
    fs["vCapID"] >> conf.vCapID;
    conf.pairingRange = (int)fs["pairingRange"];
    conf.wrapCap      = (int)fs["wrapCap"];
    conf.nPairs       = (int)fs["nPairs"];
    conf.nNodes       = (int)fs["nNodes"];
    conf.hasExto      = !fs["hasExto"].empty();
    //conf.squareSize   = (fs["squareSize"].empty()? defaultSquareSize : (double)fs["squareSize"]);
    node.resize(conf.nNodes);
    pair.resize(conf.nPairs);

    if(conf.nNodes < 2 || conf.nNodes < 2) {
        cout << "Insufficent pairs/nodes!\n"; exit(-1); }
    for(int iN=0; iN<conf.nNodes; iN++) {
        string nn = "node_" + to_string(iN);
        if(fs[nn].empty()) { cout << "No configuration for " << nn << "\n"; exit(-1); }
        fs[nn] >> node[iN];
    }
    for(int iP=0; iP<conf.nPairs; iP++) {
        string np = "pair_" + to_string(iP);
        if(fs[np].empty()) { cout << "No configuration for " << np << "\n"; exit(-1); }
        fs[np] >> pair[iP];
    }
}

static void writeConfigurationFile(string filename, vector<node_t>& node, vector<pair_t>& pair, vector<exto_t>& exto, confparams_t& conf) {
    FileStorage fs(filename, FileStorage::WRITE);
    if(!fs.isOpened()) { cout << "Can't write to file...\n"; exit(-1); }

    fs << "vCapID"       << conf.vCapID;
    fs << "pairingRange" << conf.pairingRange;
    fs << "wrapCap"      << conf.wrapCap;
    fs << "nPairs"       << conf.nPairs;
    fs << "nNodes"       << conf.nNodes;
    fs << "nExtos"       << (int)exto.size();

    for(int iN=0; iN<conf.nNodes; iN++) {
        string nn = "node_" + to_string(iN);
        fs << nn << node[iN];
    }
    for(int iP=0; iP<conf.nPairs; iP++) {
        string np = "pair_" + to_string(iP);
        fs << np << pair[iP];
    }
    for(int iE=0; iE<exto.size(); iE++) {
        string ne = "exto_" + to_string(iE);
        fs << ne << exto[iE];
    }

    cout << "Done saving to " << filename << "\n";
}

static bool isLEQ(Vec3d v, Scalar s) {
   for(size_t i=0; i<3; i++)
       if(v[i] > s[i]) return false;
   return true;
}

static void computeExto(vector<vector<Vec3d>>& extotvecs, vector<vector<Vec3d>>& extorvecs, vector<exto_t>& extos) {
    assert(extotvecs.size() == extorvecs.size());
    int nNodes   = extotvecs.size();
    int nSamples = extotvecs[0].size();
    double sg = 1.25;// sigma of tolerance
    extos.resize(nNodes);

    cout << "Computing Exto with " << nSamples << " samples\n";
    vector<vector<Vec3d>> tavg(nNodes, vector<Vec3d>()), ravg(nNodes, vector<Vec3d>());
    vector<Scalar> ut(nNodes), st(nNodes), ur(nNodes), sr(nNodes);
    vector<Mat> vT(nNodes), vR(nNodes);

    for(int iN=0; iN<nNodes; iN++) {
        meanStdDev(extotvecs[iN], ut[iN], st[iN]);
        meanStdDev(extorvecs[iN], ur[iN], sr[iN]);
        /*
        for(int iS=0; iS<nSamples; iS++) {
            Vec3d difft, diffr;
            ut[iN] = sg*ut[iN]; ur[iN] = sg*ur[iN];
            absdiff(extotvecs[iN][iS], Vec3d(ut[iN][0], ut[iN][1], ut[iN][2]), difft);
            absdiff(extorvecs[iN][iS], Vec3d(ur[iN][0], ur[iN][1], ur[iN][2]), diffr);
            if(isLEQ(difft,st[iN])) tavg[iN].push_back(extotvecs[iN][iS]);
            if(isLEQ(diffr,sr[iN])) ravg[iN].push_back(extorvecs[iN][iS]);
        }
        // Recompute without outliers
        meanStdDev(tavg[iN], ut[iN], st[iN]);
        meanStdDev(ravg[iN], ur[iN], sr[iN]);
        */
        // Compose transformation
        extos[iN].valid = true;
        extos[iN].id    = iN;
        Rodrigues(Vec3d(ur[iN][0], ur[iN][1], ur[iN][2]), vR[iN]);
        Mat _T(Vec3d(ut[iN][0], ut[iN][1], ut[iN][2]));
        _T.copyTo(vT[iN]);
    }

    extos[0].T = (Mat_<double>(3,1) << 0., 0., 0.);
    extos[0].R = (Mat_<double>(3,3) << 1., 0., 0., 0., 1., 0., 0., 0., 1.);

    for(int iN=1; iN<nNodes; iN++) {
        //Mat RR = vR[0].inv()*vR[iN];
        Mat RR = vR[0]*vR[iN].inv();
        RR.copyTo(extos[iN].R);

        extos[iN].T = -(RR*vT[iN]) + vT[0];


        cout << "extos for node " << iN << ":\n";
        cout << "T = \n" << extos[iN].T << "\n";
        cout << "R = \n" << extos[iN].R << "\n";
    }

}

static void extoRun(vector<node_t>& node, vector<pair_t>& pair, confparams_t& conf, float markerSz, string filename) {
    int nPairs   = pair.size();
    int nNodes   = node.size();
    int nSamples = extoNumOfSamples;
    Size imageSize;
    bool allFound, startExto = false;

    Mat* frame   = new Mat[nNodes];
    VideoCapture* capdev = new VideoCapture[nNodes];
    aruco::Dictionary dictionary = aruco::getPredefinedDictionary((aruco::PREDEFINED_DICTIONARY_NAME)arucoMarkerDict);

    vector<int> mids(nNodes);
    vector<exto_t> exto;
    vector<vector<Vec3d>> extotvecs(nNodes), extorvecs(nNodes);
    vector<Vec3d> tvecs, rvecs;
    vector<vector<Point2f> > corners;

    imageSize.height = node[0].heigth;
    imageSize.width  = node[0].width;

    // Initialize video captures
    for(int iN=0; iN < nNodes; iN++) {
        capdev[iN].set(CV_CAP_PROP_FRAME_HEIGHT, node[iN].heigth);
        capdev[iN].set(CV_CAP_PROP_FRAME_WIDTH,  node[iN].width);
        if(!capdev[iN].open(node[iN].devId)) {
            cout << "Cant' open devid = " << !node[iN].devId << " for node "
                 << node[iN].id << "\n"; return; }
    }

    // Main loop
    for(;;) {
        // Capture loop
        for(int iN=0; iN<nNodes; iN++) {
            capdev[iN].read(frame[iN]);
            if(!frame[iN].data) break;
            if(frame[iN].size() != imageSize) {
                cout << "Frame size mismatch, skipping\n"; break; }
        }

        allFound = true;
        // Detection loop
        for(int iN=0; iN<nNodes; iN++) {
            //undistort(frame[iN], frame[iN], node[iN].camMat, node[iN].distCoeffs);
            aruco::detectMarkers(frame[iN], dictionary, corners, mids);
            if(mids.size() == 1) {
                allFound &= allFound;
                aruco::drawDetectedMarkers(frame[iN], corners, mids);
                aruco::estimatePoseSingleMarkers(corners, arucoMarkerLength, node[iN].camMat, node[iN].distCoeffs, rvecs, tvecs);
                for(int iM=0; iM<mids.size(); iM++) {
                    aruco::drawAxis(frame[iN], node[iN].camMat, node[iN].distCoeffs, rvecs[iM], tvecs[iM], 0.1);
                }
                //if(iN == 1)
                //    cout << tvecs[0] << "\t" << rvecs[0] << "\n";
                // Store samples
                if(startExto){
                    extotvecs[iN].push_back(tvecs[0]);
                    extorvecs[iN].push_back(rvecs[0]);
                }
            } else allFound = false;
        }

        // Display loop
        for(int iN=0; iN<nNodes; iN++) {
            if(allFound && !startExto)
                putText(frame[iN], "'s' to set origin!", Point2f(50,50), cv::FONT_HERSHEY_SIMPLEX, 1.0, Scalar(0,255,0), 2);
            if(startExto)
                putText(frame[iN], "Capturing...", Point2f(50,50), cv::FONT_HERSHEY_SIMPLEX, 1.0, Scalar(255,0,0), 2);

            string winname;
            winname += "Node " + to_string(node[iN].id) + " (Dev " + to_string(node[iN].devId) + ")";
            imshow(winname, frame[iN]);
        }
        char c = waitKey(30);
        if(allFound && c == 's') startExto = true;
        else if(c >= 0) break;

        // Break if collected all samples
        if(nSamples <= 0) break;
        if(startExto) nSamples--;
    }

    // Release video captures
    cout << "Releasing...\n";
    destroyAllWindows();
    for(int iN=0; iN < nNodes; iN++)
        capdev[iN].release();

    // If samples exist, compute and save the exto
    if(nSamples <= 0) {
        cout << "Computing external origin...\n";
        computeExto(extotvecs, extorvecs, exto);
        writeConfigurationFile(filename, node, pair, exto, conf);
    }

    return;
}

// * * * * * * MAIN  * * * * * * * * * * * * * * * * * * * * * * * * * * * * //


int main(int argc, char** argv) {

    cv::CommandLineParser parser(argc, argv, "{@c||}{gm||}{id||}{sz||}{out||}{help||}");
    if (!parser.check()) {
        parser.printErrors();
        return 1;
    }

    if(parser.has("help") || !parser.has("@c"))
        return print_help();

    string filename = parser.get<string>("@c");
    string outname  = (parser.has("out")? parser.get<string>("out") : filename);
    int    markerid = (parser.has("id")?  parser.get<int>("id") : arucoMarkerId);
    float  markersz = (parser.has("sz")?  parser.get<int>("sz") : arucoMarkerLength);

    vector<node_t> nodes;
    vector<pair_t> pairs;
    confparams_t conf;

    if(parser.has("gm")) {
        generateCalibrationMarker(filename, markerid);
        return 0;
    }

    readConfigurationFile(filename, nodes, pairs, conf);

    extoRun(nodes, pairs, conf, markersz, outname);

    return 0;
}
