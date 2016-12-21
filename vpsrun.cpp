#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include <vector>
#include <thread>
#include <string>
#include <algorithm>
#include <iostream>
#include <iterator>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>

#include "vpssocket.h"
#include "vpsstorage.h"
#include "vpsmath.h"

using namespace cv;
using namespace std;
using namespace vps;

// * * * * * * CONSTANTS  * * * * * * * * * * * * * * * * * * * * * * * * * * //
const int cg_maxTCPPoints= 20;      // Number of points that will be send over TCP

float g_pt3dAcceptThresh = .1;     // 3D error matching threshold (m)
float g_pt2dEpiErrThresh = 10.;     // Epiline reprojection error threshold (px)

int   g_blobMinArea = 200;
int   g_blobMaxArea = 10000;

// * * * * * * HELP  * * * * * * * * * * * * * * * * * * * * * * * * * * * * //
//{c|conf.yml|}{exto||}{epith||}{3dth||}{port||}{help||}
static int print_help()
{
    cout << "openVPS CLI Viewer\n\n";
    cout << "Usage - Live Mode:\n ";
    cout <<  "./openvps-cli-epilines -c=<conf_file=conf.yml>\n\n";
    cout << "help           Print this help.\n";
    cout << "c=<string>     Input configuration file path/name.\n";
    cout << "port=<int>     Stream matched pairs over TCP.\n";
    cout << "exto           Use extrinsic origin definition (Recommended!)\n";
    cout << "epith=<float>  Epiline matching tolerance (10.)\n";
    cout << "3dth=<float>   3D matching reprojection (0.1)\n";
    cout << "bmin=<int>     Minimum area for blob detection (200)\n";
    cout << "bmax=<int>     Maximum area for blob detection (Inf)\n";
    cout << "hotswap=<int>  Change devId for nodes on the fly (e.g., hotswap=0123)\n";
    return 0;
}

static void drawMarkerIdentifier(Mat* frames, vector<Point2f>& pts, int nNodes, int id) {
    for(size_t iN=0; iN<nNodes; iN++) {
        if(pts[iN] == Point2f(-1.,-1.)) continue;
        putText(frames[iN], to_string(id), pts[iN], FONT_HERSHEY_SIMPLEX, 1.0, Scalar(0,0,255), 2);
    }
}

// Input points must all be rectified on node of origin
static bool findMultiViewPoint(vector<node_t>& nodes, vector<Point2f>& matchPts,
                               vector<Point3d>& tvecs, vector<Mat>& rmats, Point3d& ptout) {
    assert(nodes.size() == matchPts.size());

    Point2f pt2Empty(-1.,-1.);
    vector<Point3d> dvec, _tvecs;

    for(size_t iN=0; iN<nodes.size(); iN++) {
        if(matchPts[iN] == pt2Empty) continue;
        Point3d d;
        computeNormalVectorFromPoint(matchPts[iN], nodes[iN].camMat, d);
        //cout << d << "\n";
        Mat td = rmats[iN]*Mat(d);
        dvec.push_back(Point3d(td));
        _tvecs.push_back(tvecs[iN]);
    }

    if(dvec.size() < 2) return false;
    return intersectLinesLSQ(_tvecs, dvec, ptout, true, g_pt3dAcceptThresh);
}

static void vpsRun(vector<node_t>& node, vector<pair_t>& pair, vector<Point3d>& tvecs, vector<Mat>& rmats, int pRange, bool streamTCP=false) {
    int nPairs = pair.size();
    int nNodes = node.size();
    Size imageSize;

    Mat* frame   = new Mat[nNodes];
    Mat* wkframe = new Mat[nNodes];
    Mat  kernel  = getStructuringElement(MORPH_RECT, Size(3,3), Point(1,1));
    VideoCapture* capdev = new VideoCapture[nNodes];

    vector<Point3f> points3d;
    vector<KeyPoint>* keypoints = new vector<KeyPoint>[nNodes]; // Keypoints on each node
    vector<Point2f>*  imgpoints = new vector<Point2f>[nNodes];  // Undistorted keypoints on each node

    imageSize.height = node[0].heigth;
    imageSize.width  = node[0].width;

    SimpleBlobDetector::Params blobParams;
    blobParams.filterByCircularity = true;
    blobParams.minCircularity = 0.7;
    blobParams.maxCircularity = 1.0;
    blobParams.filterByConvexity = true;
    blobParams.minConvexity = 0.7;
    blobParams.maxConvexity = 1.0;
    blobParams.filterByArea = true;
    blobParams.minArea = g_blobMinArea;
    blobParams.maxArea = g_blobMaxArea;
    Ptr<SimpleBlobDetector> pBlobDetector = SimpleBlobDetector::create(blobParams);

    // Initialize stereo rectification if needed
    for(int iP=0; iP < nPairs; iP++) {
        if(!pair[iP].valid) continue;
        int n0i = pair[iP].n[0];
        int n1i = pair[iP].n[1];
        if(pair[iP].Q.empty() || pair[iP].P1.empty() || pair[iP].R1.empty()) {
            //cout << "Initializing stereoRectify for pair " << iP << "\n";
            stereoRectify(node[n0i].camMat, node[n0i].distCoeffs, node[n1i].camMat, node[n1i].distCoeffs,
                          imageSize, pair[iP].R, pair[iP].T,
                          pair[iP].R1, pair[iP].R2, pair[iP].P1, pair[iP].P2, pair[iP].Q);
        }
    }

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
        // Capture
        for(int iN=0; iN<nNodes; iN++) {
            capdev[iN].read(frame[iN]);
            if(!frame[iN].data) break;
            if(frame[iN].size() != imageSize) {
                cout << "Frame size mismatch, skipping\n"; break; }
            // Detect blobs
            keypoints[iN].clear();
            imgpoints[iN].clear();
            cvtColor(frame[iN], wkframe[iN], COLOR_RGB2GRAY);
            dilate(wkframe[iN], wkframe[iN], kernel);
            pBlobDetector->detect(wkframe[iN], keypoints[iN]);
            drawKeypoints(wkframe[iN], keypoints[iN], wkframe[iN], Scalar(0,0,255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
            // Sort, convert and rectify any found keypoints
            if(!keypoints[iN].empty()) {
                KeyPoint::convert(keypoints[iN], imgpoints[iN]);
                undistortPoints(imgpoints[iN], imgpoints[iN], node[iN].camMat, node[iN].distCoeffs, noArray(), node[iN].camMat);
                sort(imgpoints[iN].begin(), imgpoints[iN].end(), [](Point2f& a, Point2f& b){ return a.x < b.x; });
            }
        }

        // Find matches and compute 3d points
        points3d.clear();
        int matchPtId = 0;
        // Iterate over nodes (using each one as base node at a time)
        for(int baseN=0; baseN<nNodes-1; baseN++) {
            int iN = baseN, iA = baseN+1, iK = 0, iP = 0; // base, adjacent, keypoint, pair
            int matchIdx = -1;

            //cout << "\nAvailable pts on node " << baseN << " = " << imgpoints[baseN].size() << "\n";
            Point2f pt2Empty(-1.,-1.);
            vector<Point2f> matchpt2(nNodes, pt2Empty);
            vector<bool>    isfound(nNodes, false);
            // Iterate over keypoints
            while(iK < imgpoints[baseN].size()) {
                if(!isfound[baseN]) {
                    matchpt2[baseN] = imgpoints[baseN][iK];
                    isfound[baseN] = true;
                }
                // If search window's base has a point and window's edge doesn't, search...
                if(isfound[iN] && !isfound[iA]) {
                    for(iP=0; iP<nPairs; iP++) {
                        if(pair[iP].valid && pair[iP].n[0] == iN && pair[iP].n[1] == iA) {
                            if(matchPointViaEpiline(pair[iP], matchpt2[iN], imgpoints[iA], matchIdx, imageSize, g_pt2dEpiErrThresh)) {
                                // If match is found, store it and remove image point from corresponding node
                                isfound[iA] = true;
                                matchpt2[iA] = imgpoints[iA][matchIdx];
                                imgpoints[iA].erase(imgpoints[iA].begin() + matchIdx);
                            }
                        }
                    }
                }
                // If reached edge of search, finish it: compute point and display any found markers
                if(iN == nNodes-1 && iA == nNodes) {
                    Point3d pt3d;
                    if(findMultiViewPoint(node,matchpt2,tvecs,rmats,pt3d)) {
                        points3d.push_back(pt3d);
                    }
                    // Draw identifiers (regardless of successfull multiview computation
                    drawMarkerIdentifier(wkframe, matchpt2, nNodes, matchPtId);
                    matchPtId++;
                    // Move to next keypoint; restart search on base pair
                    iK++; iN=baseN; iA=baseN+1;
                    for(int iReset=0; iReset<nNodes; iReset++) {
                        matchpt2[iReset] = pt2Empty;
                        isfound[iReset] = false;
                    }
                } else {
                    // Increase window, if possible
                    if(iA - iN < pRange) iA++;
                    // Move window's base to next available node
                    else {
                        do { iN++; } while(iN < nNodes-1 && !isfound[iN]);
                        iA = iN + 1;
                    }
                }
            }
        }

        // Stream over TCP
        if(streamTCP) {
            const float _inf = -1000.;
            float tcppoints[cg_maxTCPPoints*3];
            int   ps = points3d.size();
            for(int i=0; i<cg_maxTCPPoints; i++) {
                tcppoints[i*3 + 0] = (i < ps? points3d[i].x : -_inf);
                tcppoints[i*3 + 1] = (i < ps? points3d[i].y : -_inf);
                tcppoints[i*3 + 2] = (i < ps? points3d[i].z : -_inf);
            }
            writeToConnection(tcppoints, cg_maxTCPPoints*12);
        }

        // Display
        for(int iN=0; iN<nNodes; iN++) {
            string winname;
            winname += "Node " + to_string(node[iN].id) + " (Dev " + to_string(node[iN].devId) + ")";
            imshow(winname, wkframe[iN]);
        }
        if(waitKey(30) >= 0) break;
    }

    // Release video captures
    cout << "Releasing...\n";
    destroyAllWindows();
    for(int iN=0; iN < nNodes; iN++)
        capdev[iN].release();
}

// * * * * * * MAIN  * * * * * * * * * * * * * * * * * * * * * * * * * * * * //


int main(int argc, char** argv) {

    cv::CommandLineParser parser(argc, argv, "{c|conf.yml|}{exto||}{epith||}{3dth||}{bmin||}{bmax||}{hotswap||}{port||}{help||}");
    if (!parser.check()) {
        parser.printErrors();
        return 1;
    }

    if(parser.has("help"))
        return print_help();

    string filename = parser.get<string>("c");
    cout << "Reading " << filename << "\n";
    FileStorage fs(filename, FileStorage::READ);
    if(!fs.isOpened()) { cout << "Can't read configuration file!\n"; return -1; }
    if(fs["nPairs"].empty() || fs["nNodes"].empty() || fs["pairingRange"].empty()) {
        cout << "Invalid configuraton file!\n"; return -1; }

    int port   = (parser.has("port")? parser.get<int>("port") : -1);
    int nPairs = (int)fs["nPairs"];
    int nNodes = (int)fs["nNodes"];
    int pRange = (int)fs["pairingRange"];
    bool hotsw = parser.has("hotswap");
    vector<pair_t> pairs;
    vector<node_t> nodes;
    vector<int> hotswapi;
    pairs.resize(nPairs);
    nodes.resize(nNodes);

    // Changing some globals (globals? shame on me...)
    g_pt2dEpiErrThresh = parser.has("epith")? parser.get<float>("epith") : g_pt2dEpiErrThresh;
    g_pt3dAcceptThresh = parser.has("3dth")?  parser.get<float>("3dth")  : g_pt3dAcceptThresh;
    g_blobMinArea      = parser.has("bmin")?  parser.get<int>("bmin")    : g_blobMinArea;
    g_blobMaxArea      = parser.has("bmax")?  parser.get<int>("bmax")    : g_blobMaxArea;

    if(nNodes < 2 || nPairs < 2) {
        cout << "Insufficent pairs/nodes!\n"; return -1; }

    // Load hotswap if available
    if(hotsw) {
        string hotswstr = parser.get<string>("hotswap");
        if(hotswstr.length() != nNodes) {
            cout << "Incorrect hotswap definition!\n"; return -1; }
        for(int iS=0; iS < nNodes; iS++)
            hotswapi.push_back((int)(hotswstr.c_str()[iS] - '0'));
    }

    // Load nodes
    for(int iN=0; iN<nNodes; iN++) {
        string nodename = "node_" + to_string(iN);
        if(fs[nodename].empty()) {
            cout << "No configuration for " << nodename << "!\n"; return -1; }
        fs[nodename] >> nodes[iN];
        if(hotsw) nodes[iN].devId = hotswapi[iN];
    }

    // Load pairs
    for(int iP=0; iP<nPairs; iP++) {
        string pairname = "pair_" + to_string(iP);
        if(fs[pairname].empty()) {
            cout << "No configuration for " << pairname << "!\n"; return -1; }
        fs[pairname] >> pairs[iP];
    }

    // Load extos or initialize transformations
    vector<Point3d> translations;
    vector<Mat>     rotations;
    if(parser.has("exto")) {
        for(int iE=0; iE<nNodes; iE++) {
            exto_t e;
            string extoname = "exto_" + to_string(iE);
            if(fs[extoname].empty()) {
                cout << "No configuration for " << extoname << "!\n"; return -1; }
            fs[extoname] >> e;
            translations.push_back(Point3d(e.T));
            rotations.push_back(Mat(e.R));
        }
        cout << "Loaded external origin definitions\n";
    } else if(!initializeNodeTransformations(pairs, pRange, nNodes, translations, rotations)) {
        cout << "Unable to initialize transformations. Try recalibrating system.\n";
        return -1;
    }

    // Print transformations/extos
    for(int i=0; i<nNodes; i++) {
        cout << "T_0^" << i << " =\n" << translations[i] << "\n";
        cout << "R_0^" << i << " =\n" << rotations[i] << "\n";
    }

    if(port != -1) openConnection(port);
    vpsRun(nodes, pairs, translations, rotations, pRange, (port != -1));
    if(port != -1) closeConnection();

    return 0;
}
