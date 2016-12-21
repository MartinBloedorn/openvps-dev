#ifndef VPSSTORAGE_H
#define VPSSTORAGE_H

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

using namespace cv;
using namespace std;

namespace vps {

// * * * * * * TYPES * * * * * * * * * * * * * * * * * * * * * * * * * * * * //

typedef struct {
    int     id; // node's positional id
    int     devId; // node's device id
    Mat     camMat, distCoeffs;
    int     width, heigth, fps; // capture settings
    float   rmsErr;
    float   a;
} node_t;

typedef struct {
    bool    valid;
    int     id;
    int     c[2]; // Indexes of capture devices
    int     n[2]; // Indexes of nodes
    // Rotation, Translation, Essential, Fundamental, disparity-to-depth
    Mat     R, T, E, F, Q;
    // Projection, Rectification
    Mat     P1, P2, R1, R2;
    float   rmsErr;
    //node_t * node[2]; // Pointer to pair's nodes
} pair_t;

typedef struct {
    bool  valid;
    int   id;
    float rmsErr;
    Mat T, R;
} exto_t;

// * * * * * * STORAGE FUNCTIONS * * * * * * * * * * * * * * * * * * * * * * //


// PAIR_T < < < < < < < < < < < < < < < < < < < < < < < < < < < < < < < < < < <

static std::ostream& operator <<(std::ostream& out, const pair_t& p) {
    out << "{ id : " << p.id   << "\n";
    out << "  valid : " << p.valid << "\n";
    out << "  n0 : " << p.c[0] << " - n1: " << p.c[1] << "\n";
    out << "  R :\n"  << p.R << "\n  T :\n" << p.T << "\n";
    out << "  F :\n"  << p.F << "\n  E :\n" << p.E << "\n";
    out << "  rmsErr : " << p.rmsErr << " }\n";
    return out;
}

static void write(cv::FileStorage& fs, const std::string&, const pair_t& p) {
    fs << "{" << "id" << p.id << "valid" << p.valid;
    fs << "n0" << p.n[0] << "n1" << p.n[1];
    fs << "R" << p.R << "T" << p.T << "E" << p.E << "F" << p.F;
    fs << "rmsErr" << p.rmsErr << "}";
}

static void read(const cv::FileNode & n, pair_t& p, const pair_t&) {
    p = pair_t();
    if(n.empty()) return;
    p.id        = (int)n["id"];
    p.valid     = (bool)(int)n["valid"];
    p.n[0]      = (int)n["n0"];
    p.n[1]      = (int)n["n1"];
    n["R"] >> p.R; n["T"] >> p.T;
    n["E"] >> p.E; n["F"] >> p.F;
    p.rmsErr    = (float)n["rmsErr"];
}

// NODE_T < < < < < < < < < < < < < < < < < < < < < < < < < < < < < < < < < < <

static void write(cv::FileStorage& fs, const std::string&, const node_t& n) {
    fs << "{" << "id" << n.id << "devId" << n.devId;
    fs << "camMat" << n.camMat << "distCoeffs" << n.distCoeffs;
    fs << "width" << n.width << "height" << n.heigth << "fps" << n.fps;
    fs << "rmsErr" << n.rmsErr << "a" << n.a << "}";
 }

static void read(const cv::FileNode & fn, node_t& n, const node_t&) {
    n = node_t();
    if(fn.empty()) return;
    n.id        = (int)fn["id"];
    n.devId     = (int)fn["devId"];
    n.width     = (int)fn["width"];
    n.heigth    = (int)fn["height"];
    n.fps       = (int)fn["fps"];
    n.rmsErr    = (float)fn["rmsErr"];
    n.a         = (float)fn["a"];
    fn["camMat"]     >> n.camMat;
    fn["distCoeffs"] >> n.distCoeffs;
}

static std::ostream& operator <<(std::ostream& out, const node_t& n) {
    out << "{ id : " << n.id   << "\n";
    out << "  devId : " << n.devId   << "\n";
    out << "  camMat:\n" << n.camMat << "\n";
    out << "  distCoeffs:\n" << n.distCoeffs << "\n";
    out << "  w : " << n.width << "\n";
    out << "  h : " << n.heigth << "\n";
    out << "  a : " << n.a << "\n";
    out << "  rmsErr : " << n.rmsErr << " }\n";
    return out;
}

// EXTO_T < < < < < < < < < < < < < < < < < < < < < < < < < < < < < < < < < < <

static void write(cv::FileStorage& fs, const std::string&, const exto_t& e) {
    fs << "{" << "id" << e.id << "valid" << e.valid << "rmsErr" << e.rmsErr;
    fs << "T" << e.T << "R" << e.R << "}";
}

static void read(const cv::FileNode & fn, exto_t& e, const exto_t&) {
    e = exto_t();
    e.id        = (int)fn["id"];
    e.valid     = (int)fn["valid"];
    e.rmsErr    = (float)fn["rmsErr"];
    fn["T"] >> e.T;
    fn["R"] >> e.R;
}

static std::ostream& operator <<(std::ostream& out, const exto_t& e) {
    out << "{ id : " << e.id << "\n";
    out << "  valid : " << e.valid << "\n";
    out << "  rmsErr: " << e.rmsErr << "\n";
    out << "  T : " << e.T << "\n";
    out << "  R : " << e.R << "}\n";
}

} // namespace vps;

#endif // VPSSTORAGE_H
