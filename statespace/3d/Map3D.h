#ifndef RRT_MAP3D_H
#define RRT_MAP3D_H

#include "State3D.h"
#include <string>
#include <png.h>

#define MAP3D_MAX_OBJECTS 1000

class Map3DObject {
public:
    State3D bound_lower;
    State3D bound_upper;
};

class Map3D {

friend class State3DMath;

public:
    Map3D(std::string datafile);

    void getBounds(State3D* minimums, State3D* maximums);

    bool pointInObstacle(State3D* point);
    bool edgeInObstacle(State3D* pointA, State3D* pointB);
    float pointCost(State3D* point);
    float edgeCost(State3D* pointA, State3D* pointB);

    void configureVis(int width, int height);
    void resetVis();
    void addVisPoint(State3D* point, int color, bool big=false);
    void addVisLine(State3D* pointA, State3D* pointB, int color);
    void addGoalDetail(State3D* source, State3D* dest);
    void renderVis(std::string filename_prefix);
    void takeScreenshot(std::string filename_prefix);
    void renderFinalVis(std::string filename_prefix);

protected:
    void write_video(std::string filename_prefix);
    std::string ReplaceString(std::string subject, const std::string& search, const std::string& replace);
    void add_image_to_list(std::string filename_prefix);

    Map3DObject border;
    Map3DObject objects[MAP3D_MAX_OBJECTS];
    int object_count = 0;

    std::string html = "";

    std::string filelist = "";
    std::string last_screenshot_filename_prefix = "";

    int vis_width = 640;
    int vis_height = 480;

    int frames = 0;

};


#endif
