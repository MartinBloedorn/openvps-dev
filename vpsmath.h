#ifndef VPSMATH_H
#define VPSMATH_H

#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include <vector>
#include <math.h>
#include <thread>
#include <string>
#include <algorithm>
#include <iostream>
#include <iterator>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>

using namespace cv;
using namespace std;

//#define VPS_TRANSFORM_USE_ADJACENTONLY

namespace vps {

// Finds successive transformations from origin node (0) to all others; i.e. T_0^1|R_0^1, T_0^2|R_0^2, ...
static bool initializeNodeTransformations(vector<pair_t> pairs, int pRange, int nNodes, vector<Point3d>& tout, vector<Mat>& rout) {
    tout.resize(nNodes);
    rout.resize(nNodes);
    vector<bool> vfound(nNodes, false);

    // Add identity transformations first
    tout[0] = Point3d(0.,0.,0.);
    rout[0] = (Mat_<double>(3,3) << 1., 0., 0., 0., 1., 0., 0., 0., 1.);
    vfound[0] = true;

    int nPairs = pairs.size();
    int base=0, adj=1, iP=0;

    while(adj < nNodes) {
        // Search over pairs for transformations between base and adj nodes
        for(iP=0; iP<nPairs; iP++) {
            if(pairs[iP].valid && pairs[iP].n[0] == base && pairs[iP].n[1] == adj) {
                cout << "Found transformation for " << base << "-" << adj << "\n";
                vfound[adj] = true; break; }
        }
        if(vfound[adj]) {
            // From stereoCalibrate, rotation needs to be inverted and translation has negative X and Y
            Mat lR = Mat(pairs[iP].R.inv()); // n <- n+1
            Point3d lT = Point3d(-pairs[iP].T.at<double>(0), -pairs[iP].T.at<double>(1), pairs[iP].T.at<double>(2));
            // Apply succesive transformations if needed (i.e. search window's base node not at 0)
            if(base > 0)
                for(int iST=base; iST>=0; iST--) {
                    lT += tout[iST];
                    lR  = lR*rout[iST];
                }
            tout[adj] = lT;
            rout[adj] = lR;
            // Move pair forward (try to find only adjacent pairs)
            base++; adj++;
        } else {
            // If adjacent pairs are not available, move base back
            if(base > 0) base--; else return false;
        }
        // If no transform was found for a previous position, abort
        if(base > 0 && !vfound[base-1]) return false;
    }
    return true;
}

// Matches pt2match to one op the pair's points; returns true and puts the point's idx in idxout
static bool matchPointViaEpiline(pair_t& pair, Point2f& pt2match, vector<Point2f>& points, int& idxout,
                                 Size imageSize=Size(640,480), float thres=25.0) {
    if(points.empty()) return false;

    vector<Vec3f> line;
    vector<Point2f> pti; pti.push_back(pt2match);
    computeCorrespondEpilines(pti, 1, pair.F, line);

    idxout = -1;
    float minerr = thres;
    // Compute error of each point to epiline
    for(size_t iP=0; iP<points.size(); iP++) {
        float err = fabs(points[iP].x * line[0][0] + points[iP].y * line[0][1] +
                line[0][2]);// + 10.*points[iP].x/(float)imageSize.width;
        if(err < minerr) { minerr = err; idxout = iP; break; }
    }
    return (idxout != -1);
}


// http://stackoverflow.com/questions/12977980/in-opencv-converting-2d-image-point-to-3d-world-unit-vector
void computeNormalVectorFromPoint(Point2f& imgpt, Mat& camMat, Point3d& dirout) {
    Mat homogpt = (Mat_<double>(3,1) << imgpt.x, imgpt.y, 1.);
    homogpt = camMat.inv()*homogpt;
    dirout  = Point3f(homogpt);
    dirout  = dirout/norm(dirout);
}


// http://math.stackexchange.com/questions/61719/finding-the-intersection-point-of-many-lines-in-3d-point-closest-to-all-lines
bool intersectLinesLSQ(vector<Point3d>& linepts, vector<Point3d>& linedir, Point3d& ptout, bool normalizedDir=false, float thres=-1.) {
    assert(linepts.size() == linedir.size());

    int nLines = linedir.size();
    if(!normalizedDir)
        for(size_t iLD=0; iLD<nLines; iLD++)
            linedir[iLD] *= 1./norm(linedir[iLD]);

    vector<vector<double>> n(3, vector<double>(nLines));
    double sxx = 0., syy = 0., szz = 0., sxy = 0., sxz = 0., syz = 0.;
    double cx =0., cy = 0., cz = 0.;
    double nsq[3], nnn[3];

    for(int iL=0; iL<nLines; iL++) {
        //cout << "Line " << iL << " = " <<linedir[iL] << "\n";
        n[0][iL] = linedir[iL].x;
        n[1][iL] = linedir[iL].y;
        n[2][iL] = linedir[iL].z;
        for(int iN=0; iN<3; iN++) {
            nsq[iN] = pow(n[iN][iL], 2.) - 1.;
            nnn[iN] = n[iN==2?1:0][iL] * n[iN==0?1:2][iL]; // 0*1, 0*2, 1*2
        }
        sxx += nsq[0]; syy += nsq[1]; szz += nsq[2];
        sxy += nnn[0]; sxz += nnn[1]; syz += nnn[2];
        cx += linepts[iL].x*nsq[0] + linepts[iL].y*nnn[0] + linepts[iL].z*nnn[1];
        cy += linepts[iL].x*nnn[0] + linepts[iL].y*nsq[1] + linepts[iL].z*nnn[2];
        cz += linepts[iL].x*nnn[1] + linepts[iL].y*nnn[2] + linepts[iL].z*nsq[2];
    }

    Mat S = (Mat_<double>(3,3) << sxx, sxy, sxz, sxy, syy, syz, sxz, syz, szz);
    Mat C = (Mat_<double>(3,1) << cx, cy, cz);
    Mat Y = S.inv()*C;
    ptout = Point3d(Y);
    //cout << "S =\n" << S << "\n";
    //cout << "C =\n" << C << "\n";
    //cout << "Y =\n" << Y << "\n";

    // Verify if point lies within thres for all input lines
    if(thres > 0.) {
        bool recompute = false;
        vector<double> distances(nLines, 0.);
        vector<Point3d> gooddir, goodpts;
        int good = 0;
        Mat mpt(ptout);
        cout << "Distances =\t";
        for(int iD=0; iD<nLines; iD++) {
            Mat lpt(linepts[iD]), ldi(linedir[iD]);
            double ui = norm((mpt.t() - lpt.t())*ldi);
            distances[iD] = norm(mpt - lpt - ui*ldi);
            cout << distances[iD] << "\t";
            if(distances[iD] <= thres) {
                gooddir.push_back(linedir[iD]);
                goodpts.push_back(linepts[iD]);
                good++;
            } else
                recompute = true;
        }
        cout << "\n";
        if(!recompute) return true;
        else if(good > 1) return intersectLinesLSQ(goodpts, gooddir, ptout, normalizedDir, thres);
        else return false;
    } else
        return true;
}

} // namespace vps;

#endif // VPSMATH_H
