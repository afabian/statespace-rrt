#include "Map3D.h"
#include <cstdio>
#include <cmath>
#include <cstdint>

void Map3D::resetVis() {
    html = "";
    html += "<head>";
    html += "<style>body { margin: 0; } </style>";
    html += "<script src=\"../res/three.js\"></script>";
    html += "<script src=\"../res/3d-force-graph.js\"></script>";
    html += "</head>\n";
    html += "<body><div id=\"3d-graph\"></div>\n";
    html += "<script>\n";
    html += "let nodes = []; let links = [];\n";
}

void Map3D::addVisPoint(State3D *point, int color) {
    html += "nodes.push({id: " + std::to_string((size_t)point) + ", x:" + std::to_string(point->x ) + ", y:" + std::to_string(point->y) + ", z:" + std::to_string(point->z) + "});\n";
}

void Map3D::addVisLine(State3D *pointA, State3D *pointB, int color) {
    html += "links.push({source: " + std::to_string((size_t)pointA) + ", target: " + std::to_string((size_t)pointB) + "});\n";
}

void Map3D::renderVis(std::string filename_prefix) {

    // node graph
    html += "let gData = { nodes: nodes, links: links };\n";
    html += "const graph = ForceGraph3D() (document.getElementById('3d-graph')).graphData(gData);\n";

    // map objects
    html += "const material_object = new THREE.MeshLambertMaterial({color: 0x888888, side: THREE.DoubleSide});\n";
    html += "var geometry;\n";
    html += "var cube;\n";
    for (int i=0; i<object_count; i++) {
        float dx = objects[i].bound_upper.x - objects[i].bound_lower.x;
        float dy = objects[i].bound_upper.y - objects[i].bound_lower.y;
        float dz = objects[i].bound_upper.z - objects[i].bound_lower.z;
        html += "geometry = new THREE.BoxGeometry(" + std::to_string(dx) + ", " + std::to_string(dy) + ", " + std::to_string(dz) + ");\n";
        html += "cube = new THREE.Mesh(geometry, material_object);\n";
        html += "cube.position.set(" + std::to_string(objects[i].bound_lower.x) + ", " + std::to_string(objects[i].bound_lower.y) + ", " + std::to_string(objects[i].bound_lower.z) + ");\n";
        html += "graph.scene().add(cube);\n";
    }

    // map border
    html += "const material_border = new THREE.MeshLambertMaterial({color: 0xbb4444, side: THREE.DoubleSide});\n";
    html += "material_border.transparent = true; material_border.opacity = 0.5;\n";
    float dx = border.bound_upper.x - border.bound_lower.x;
    float dy = border.bound_upper.y - border.bound_lower.y;
    float dz = border.bound_upper.z - border.bound_lower.z;
    html += "geometry = new THREE.BoxGeometry(" + std::to_string(dx) + ", " + std::to_string(dy) + ", " + std::to_string(dz) + ");\n";
    html += "cube = new THREE.Mesh(geometry, material_border);\n";
    html += "cube.position.set(" + std::to_string(border.bound_lower.x) + ", " + std::to_string(border.bound_lower.y) + ", " + std::to_string(border.bound_lower.z) + ");\n";
    html += "graph.scene().add(cube);\n";

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
