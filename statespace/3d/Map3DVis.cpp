#include "Map3D.h"
#include <cstdio>
#include <cmath>
#include <cstdint>

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
    html += "nodes.push({id: " + std::to_string((size_t)point) + ", x:" + std::to_string(point->x ) + ", y:" + std::to_string(point->y) + ", z:" + std::to_string(point->z) + "});\n";
}

void Map3D::addVisLine(State3D *pointA, State3D *pointB, int color) {
    html += "edges.push({source: " + std::to_string((size_t)pointA) + ", target: " + std::to_string((size_t)pointB) + "});\n";
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
}

void Map3D::getBounds(State3D *minimums, State3D *maximums) {
    *minimums = border.bound_lower;
    *maximums = border.bound_upper;
}

void Map3D::renderFinalVis(std::string filename_prefix) {

}