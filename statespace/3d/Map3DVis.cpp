#include "Map3D.h"
#include <cstdio>
#include <cmath>
#include <cstdint>
#include <sstream>
#include <ios>
#include <iomanip>
#include <iostream>
#include <fstream>
#include <unistd.h>

void Map3D::resetVis() {
    html = R"(
<head>
    <style>
    body { margin: 0; }
    </style>
    <script async src="https://unpkg.com/es-module-shims@1.8.0/dist/es-module-shims.js"></script>

    <script type="importmap">
      {
        "imports": {
          "three": "https://unpkg.com/three@0.156.1/build/three.module.js",
          "three/addons/": "https://unpkg.com/three@0.156.1/examples/jsm/"
        }
      }
    </script>
    <script type="module" src="../res/3d-simple.js"></script>
</head>
<body>
    <div id="3d-graph"></div>
    <script>
    let nodes = [];
    let edges = [];
    )";
}

void Map3D::addVisPoint(State3D *point, int color) {
    std::stringstream colorstream;
    colorstream << "0x" << std::setfill('0') << std::setw(6) << std::hex << color;
    html += "nodes.push({id: " + std::to_string((size_t)point) + ", x:" + std::to_string(point->x ) + ", y:" + std::to_string(point->y) + ", z:" + std::to_string(point->z) + ", color:" + colorstream.str() + "});\n";
}

void Map3D::addVisLine(State3D *pointA, State3D *pointB, int color) {
    std::stringstream colorstream;
    colorstream << "0x" << std::setfill('0') << std::setw(6) << std::hex << color;
    html += "edges.push({source: " + std::to_string((size_t)pointA) + ", target: " + std::to_string((size_t)pointB) + ", color:" + colorstream.str() + "});\n";
}

void Map3D::renderVis(std::string filename_prefix) {

    // map objects
    html += "let obstacles = [];\n";
    for (int i=0; i<object_count; i++) {
        float dx = objects[i].bound_upper.x - objects[i].bound_lower.x;
        float dy = objects[i].bound_upper.y - objects[i].bound_lower.y;
        float dz = objects[i].bound_upper.z - objects[i].bound_lower.z;
        html += "obstacles.push({origin:[" + std::to_string(objects[i].bound_lower.x) + ", " + std::to_string(objects[i].bound_lower.y) + ", " + std::to_string(objects[i].bound_lower.z) + "], size:[" + std::to_string(dx) + "," + std::to_string(dy) + "," + std::to_string(dz) + "]});\n";
    }

    // map border
    float dx = border.bound_upper.x - border.bound_lower.x;
    float dy = border.bound_upper.y - border.bound_lower.y;
    float dz = border.bound_upper.z - border.bound_lower.z;
    html += "let border = {origin:["+ std::to_string(border.bound_lower.x) + ", " + std::to_string(border.bound_lower.y) + ", " + std::to_string(border.bound_lower.z) +"], size:[" + std::to_string(dx) + ", " + std::to_string(dy) + ", " + std::to_string(dz) + "]}\n";

    // finish html document
    html += "</script>\n";
    html += "</body>\n";

    FILE* fp = fopen((filename_prefix + ".html").c_str(), "w");
    fputs(html.c_str(), fp);
    fclose(fp);

    // take screenshot of rendered scene
    takeScreenshot(filename_prefix);
}

void Map3D::takeScreenshot(std::string filename_prefix) {
    // Firefox needs to have CORS disabled for local files:
    // Navigate to about:config -> security.fileuri.strict_origin_policy -> False

    char cwd[1024] = {0};
    getcwd(cwd, sizeof(cwd));
    std::string url = "file:/" + (std::string)cwd + "/" + filename_prefix + ".html";
    url = ReplaceString(url, "\\", "/");
    url = ReplaceString(url, "//", "/");
    std::string sspath = (std::string)cwd + "/" + filename_prefix + ".png";
    sspath = ReplaceString(sspath, "/", "\\");
    std::string cmd = "firefox.exe --screenshot " + sspath + " --window-size=1920,1080 " + url;
    std::cout << cmd << std::endl;
    system(cmd.c_str());

    add_image_to_list(filename_prefix);
    last_screenshot_filename_prefix = filename_prefix;
}

void Map3D::add_image_to_list(std::string filename_prefix) {
    std::string base_filename = filename_prefix.substr(filename_prefix.find_last_of("/\\") + 1) + ".png";
    filelist += "file '" + base_filename + "'\n";
    bool is_sample = base_filename.find("sample") != std::string::npos;
    float duration = is_sample ? 0.5 : 1.0;
    filelist += "duration " + std::to_string(duration) + "\n";
}

void Map3D::getBounds(State3D *minimums, State3D *maximums) {
    *minimums = border.bound_lower;
    *maximums = border.bound_upper;
}

void Map3D::renderFinalVis(std::string filename_prefix) {
    // retrigger the last screenshot, because firefox runs asynchronously and giving it a new command
    // forces us to block until the last one finishes, meaning the last screenshot will actually be written
    // to disk so that we can use it in our video.
    takeScreenshot(last_screenshot_filename_prefix);

    write_video(filename_prefix);
}

void Map3D::write_video(std::string filename_prefix) {
    std::ofstream outfile(filename_prefix + ".txt");
    outfile << filelist;
    outfile.close();

    std::string cmd = "ffmpeg -y -f concat -i " + filename_prefix + ".txt -vf format=yuv420p -movflags +faststart " + filename_prefix + ".mp4";
    system(cmd.c_str());
}

std::string Map3D::ReplaceString(std::string subject, const std::string& search,
                          const std::string& replace) {
    size_t pos = 0;
    while ((pos = subject.find(search, pos)) != std::string::npos) {
        subject.replace(pos, search.length(), replace);
        pos += replace.length();
    }
    return subject;
}