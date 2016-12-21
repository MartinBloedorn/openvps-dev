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

// Socket must be included before 'using namespace cv',
// or there'll be naming conflicts.
#include "vpssocket.h"
#include "vpsstorage.h"
#include "vpsmath.h"

using namespace cv;
using namespace std;
using namespace vps;

// Number of points that will be send over TCP
#define MAX_TCP_NPOINTS 10

// * * * * * * HELP  * * * * * * * * * * * * * * * * * * * * * * * * * * * * //
//{c|conf.yml|}{p|0|}{help||}
static int print_help()
{
    cout << "openVPS CLI Epiline Viewer\n\n";
    cout << "Usage - Live Mode:\n ";
    cout <<  "./openvps-cli-epilines -c=<conf_file=conf.yml> -p=<number_of_pair=0> <-m>\n\n";
    cout << "help       Print this help.\n";
    cout << "c=<string> Input configuration file path/name.\n";
    cout << "p=<int>    Index of pair to be visualized.\n";
    cout << "m          Enable computation and visualization of matched pairs.\n";
    cout << "port=<int> Stream matched pairs over TCP. Works only if `m` is enabled.\n";
    return 0;
}

// * * * * * * DETECT AND COMPUTE EPILINES * * * * * * * * * * * * * * * * * //

static void detectAndComputeEpilines(pair_t & pair, node_t * node, bool viewMatches = false,  bool streamTCP = false) {
    const float pt2dEpiErrThresh = 25.;     // Epiline reprojection error threshold (px)

    VideoCapture capdev[2];
    Mat frame[2], bwframe[2], undframe[2];
    Size imageSize;
    vector<KeyPoint> keypoints[2];
    vector<Point2f> imgpts[2];
    vector<Vec3f> lines[2];
    vector<Point3d> points3d;
    float tcppoints[MAX_TCP_NPOINTS*3];

    Mat kernel = getStructuringElement(MORPH_RECT, Size(3,3), Point(1,1));

    // Initialize video captures
    for(int iC=0; iC < 2; iC++) {
        capdev[iC].set(CV_CAP_PROP_FRAME_HEIGHT, node[iC].heigth);
        capdev[iC].set(CV_CAP_PROP_FRAME_WIDTH,  node[iC].width);
        if(!capdev[iC].open(node[iC].devId)) {
            cout << "Cant' open devid = " << !node[iC].devId << " for node "
                 << node[iC].id << "\n"; return; }
    }

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
    blobParams.minArea = 100;
    Ptr<SimpleBlobDetector> pBlobDetector = SimpleBlobDetector::create(blobParams);

    // Initialize rectification
    Mat R1, R2, P1, P2, Q;
    Mat map[2][2];
    stereoRectify(node[0].camMat, node[0].distCoeffs, node[1].camMat, node[1].distCoeffs, imageSize, pair.R, pair.T, R1, R2, P1, P2, Q);
    initUndistortRectifyMap(node[0].camMat, node[0].distCoeffs, R1, P1, imageSize, CV_16SC2, map[0][0], map[0][1]);
    initUndistortRectifyMap(node[1].camMat, node[1].distCoeffs, R2, P2, imageSize, CV_16SC2, map[1][0], map[1][1]);

    // Main loop
    for(;;) {
        // Capture on each device
        for(int iC=0; iC < 2; iC++) {
            capdev[iC].read(frame[iC]);
            if(!frame[iC].data) break;

            if(frame[iC].size() != imageSize) {
                cout << "Frame size mismatch, skipping\n"; break; }

            //remap(frame[iC], undframe[iC], map[iC][0], map[iC][1], INTER_LINEAR);
            undistort(frame[iC], bwframe[iC], node[iC].camMat, node[iC].distCoeffs);
            cvtColor(bwframe[iC], bwframe[iC], COLOR_RGB2GRAY);
            //threshold(bwframe[iC], bwframe[iC], 235, 255, cv::THRESH_TOZERO_INV);
            dilate(bwframe[iC], bwframe[iC], kernel);
            pBlobDetector->detect(bwframe[iC], keypoints[iC]);
            drawKeypoints(bwframe[iC], keypoints[iC], bwframe[iC], Scalar(0,0,255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
            std::sort(keypoints[iC].begin(), keypoints[iC].end(), [](KeyPoint& a, KeyPoint& b){ return a.pt.x < b.pt.x; });
            KeyPoint::convert(keypoints[iC], imgpts[iC]);
        }

        // View epilines from left node onto right node
        if(!imgpts[0].empty()) {
            //undistortPoints(imgpts[0], imgpts[0], node[0].camMat, node[0].distCoeffs, noArray(), node[0].camMat);
            computeCorrespondEpilines(imgpts[0], 1, (1? pair.F:pair.F.t()), lines[1]);
            for(size_t iL=0; iL < lines[1].size(); iL++) {
                Point pts[2];
                pts[0].x = 0;
                pts[0].y = lines[1][iL][2];
                pts[1].x = node[0].width;
                pts[1].y = -(lines[1][iL][0]*pts[1].x + lines[1][iL][2])/lines[1][iL][1];
                line(bwframe[1], pts[0], pts[1], Scalar(0,255,0), 1);
                //err += fabs(imgpts[oIdx][iL].x*lines[wI][iL][0] + imgpts[oIdx][iL].y*lines[wI][iL][1] + lines[wI][iL][2])
            }
        }

        // Compute and visualize matches
        if(viewMatches && !imgpts[0].empty() && !imgpts[1].empty()) {
            //undistortPoints(imgpts[1], imgpts[1], node[1].camMat, node[1].distCoeffs, noArray(), node[1].camMat);
            Point2f usedp(-1,-1); // used point marker
            points3d.clear();
            int npt3d = 0;

            for(int i=0; i<MAX_TCP_NPOINTS*3; i++)
                tcppoints[i] = -100.f;

            for(int iK0=0; iK0 < keypoints[0].size(); iK0++) {
                putText(bwframe[0], to_string(iK0), keypoints[0][iK0].pt, FONT_HERSHEY_SIMPLEX, 1.0, Scalar(0,0,255), 2);
                float minerr = 1000.;
                int   matchI = -1;
                for(int iK1=0; iK1 < keypoints[1].size(); iK1++) {
                    if(keypoints[1][iK1].pt == usedp) continue;
                    // To-do : how to properly compute a threshold value for points?
                    float err = fabs(keypoints[1][iK1].pt.x * lines[1][iK0][0] +
                                     keypoints[1][iK1].pt.y * lines[1][iK0][1] + lines[1][iK0][2])
                                     + 10.*keypoints[1][iK1].pt.x/(float)imageSize.width;
                    //cout << "Error: line " << iK0 << " to point " << iK1 << " = " << err << "\n";
                    if(err < pt2dEpiErrThresh && err < minerr) { minerr = err; matchI = iK1; }
                }
                if(matchI != -1) {
                    putText(bwframe[1], to_string(iK0), keypoints[1][matchI].pt, FONT_HERSHEY_SIMPLEX, 1.0, Scalar(0,0,255), 2);
                    // Triangulate points
                    vector<Point3d> nvec(2), tvec;
                    Point3d pt3d;
                    computeNormalVectorFromPoint(keypoints[0][iK0].pt,    node[0].camMat, nvec[0]);
                    computeNormalVectorFromPoint(keypoints[1][matchI].pt, node[1].camMat, nvec[1]);
                    //cout << "Point " << iK0 << ": " << nvec[0] << "\t" << nvec[1] << "\n";
                    Mat nr  = pair.R.inv()*Mat(nvec[1]);
                    nvec[1] = Point3d(nr);
                    Point3d pt_t = Point3d(pair.T);
                    // Don't know why I have to swap this...
                    pt_t.x = -pt_t.x; pt_t.y = -pt_t.y;
                    tvec.push_back(Point3d(0,0,0));
                    tvec.push_back(pt_t);
                    intersectLinesLSQ(tvec, nvec, pt3d, true);

                    keypoints[1][matchI].pt = usedp;
                    points3d.push_back(pt3d);
                    if(npt3d < MAX_TCP_NPOINTS) {
                        tcppoints[npt3d*3 + 0] = pt3d.x;
                        tcppoints[npt3d*3 + 1] = pt3d.y;
                        tcppoints[npt3d*3 + 2] = pt3d.z;
                        npt3d++;
                    }
                    //cout << "Point " << iK0 << " = " << pt3d << "\n";
                }
            }
            if(streamTCP)
                writeToConnection(tcppoints, MAX_TCP_NPOINTS*12);
        }

        for(int iC=0; iC < 2; iC++) {
            string winname;
            winname += "Node " + to_string(node[iC].id) + " " + (iC? "(Right)":"(Left)");
            imshow(winname, bwframe[iC]);
        }

        if(waitKey(30) >= 0) break;
    }

    // Release video captures
    cout << "Releasing...\n";
    destroyAllWindows();
    for(int iC=0; iC < 2; iC++)
        capdev[iC].release();
}



// * * * * * * MAIN  * * * * * * * * * * * * * * * * * * * * * * * * * * * * //


int main(int argc, char** argv) {

    cv::CommandLineParser parser(argc, argv, "{c|conf.yml|}{p|0|}{m||}{port||}{help||}");
    if (!parser.check()) {
        parser.printErrors();
        return 1;
    }

    if(!parser.has("c") && !parser.has("p") || parser.has("help"))
        return print_help();

    int port = (parser.has("port")? parser.get<int>("port") : -1);
    string filename = parser.get<string>("c");
    cout << "Reading " << filename << "\n";
    FileStorage fs(filename, FileStorage::READ);
    if(!fs.isOpened()) { cout << "Can't read configuration file!\n"; return -1; }
    if(fs["nPairs"].empty()) { cout << "Invalid configuraton file!\n"; return -1; }

    int selecPair = parser.get<int>("p");
    string pairname = "pair_" + to_string(selecPair);
    cout << "Detected " << (int)fs["nPairs"] << " pairs. Running " << pairname << "...\n";
    if(fs[pairname].empty()) { cout << "No configuration for pair!\n"; return -1; }

    pair_t pair;
    node_t node[2];

    fs[pairname] >> pair;
    if(!pair.valid) { cout << "Invalid pair!\n"; return -1; }
    //cout << "Loaded pair:\n" << pair << "\n";

    // Load both nodes of the pair
    for(int iN=0; iN < 2; iN++) {
        string nodename = "node_" + to_string(pair.n[iN]);
        if(fs[nodename].empty()) {
            cout << "No configuration for " << nodename << "!\n"; return -1; }
        fs[nodename] >> node[iN];
        cout << "Loaded " << nodename << "\n";// << node[iN];
    }

    if(port != -1) openConnection(port);

    detectAndComputeEpilines(pair, node, parser.has("m"), (port != -1));

    if(port != -1) closeConnection();

    return 0;
}
