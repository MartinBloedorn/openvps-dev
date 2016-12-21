# openvps-dev
Some (very bad) code snippets I'm putting together while attempting to develop an visual positioning system (a.k.a. Motion Capture software). This is nowhere near a finished, used-frendly thing - I'm just dumping some ideas. Feel free to join in. 

# Wait, what?

This is an early attempt at creating a simple "visual positioning system" (VPS). A more detailed blog post can be found [here](http://martinvb.com/wp/openvps-poor-mans-motion-capture/). 

[![openVPS dev test](http://img.youtube.com/vi/KRzVlsY8vOo/0.jpg)](http://www.youtube.com/watch?v=KRzVlsY8vOo)

# Using it 

## Compiling 

You'll need [OpenCV 3.1.0](http://opencv.org) and the aRUco [contrib_module](https://github.com/opencv/opencv_contrib) - which is totally overkill but I was lazy. I've used *Qt Designer* (that's why I've kept the `.pro` files there) but there's no dependency to the Qt libraries.

Each `.cpp` file is a separate utility:

- `setup.cpp` : Computes the rotation and translation between adjacent pairs of cameras. Will compute intrisics, but it's best if they're provided. Saves all to a configuration file.
- `epilines.cpp` : Based on the configuration file, draws epilines on the right view, corresponding to markers seen in the left view of any given pair of cameras.
- `exto.cpp` : Show an "extrinsic origin" (exto) so that the system can compute the position of the cameras relative to each other.
- `vpsrun.cpp` : Runs the whole thing, an optionally streams points over TCP if you want to view them (e.g., with the provided MATLAB script). 

Most probably, nothing here will work out of the box, I'm telling you. 

## Using it 

I may never get around writing this. Who knows. [Get in touch if you care](http://martinvb.com/wp/about-me/).
