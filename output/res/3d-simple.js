import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';

function d3_simple_setup() {
    window.scene = new THREE.Scene();
    window.camera = new THREE.PerspectiveCamera( 75, window.innerWidth / window.innerHeight, 0.1, 1000 );

    window.renderer = new THREE.WebGLRenderer({antialias: true});
    renderer.setSize( window.innerWidth, window.innerHeight );
    document.body.appendChild( renderer.domElement );

    window.camera.position.set(-62, 21, 50);
    window.camera.lookAt(new THREE.Vector3(50, 50, 50));

    window.controls = new OrbitControls(camera, renderer.domElement);
    window.controls.target = new THREE.Vector3(50, 50, 50);
}

function d3_simple_world() {
    var light = new THREE.AmbientLight(0xaaaaaa);
    window.scene.add(light);
    var dlight = new THREE.DirectionalLight(0xffffff);
    dlight.position.set(10, 10, 10);
    window.scene.add(dlight);
    var dlight2 = new THREE.DirectionalLight(0xffffff);
    dlight2.position.set(-10, -10, 0);
    window.scene.add(dlight2);
    window.scene.add(new THREE.AxesHelper(100))
    window.scene.background = new THREE.Color( 0xffffff );
}

function d3_simple_nodes() {
    for (const node_index in nodes) {
        const node = nodes[node_index];
        const geometry = new THREE.SphereGeometry(0.1, 8, 8);
        const material = new THREE.MeshStandardMaterial( { color: node.color } );
        const sphere = new THREE.Mesh(geometry, material);
        sphere.position.set(node.x, node.y, node.z);
        window.scene.add(sphere);
    }
}

function d3_simple_edges() {
    let node_lookup = {};
    for (const node_index in nodes) {
        node_lookup[nodes[node_index].id] = node_index;
    }
    for (const edge_index in edges) {
        const edge = edges[edge_index];
        const pointA = nodes[node_lookup[edge.source]];
        const pointB = nodes[node_lookup[edge.target]];
        const points = [];
        points.push(new THREE.Vector3(pointA.x, pointA.y, pointA.z));
        points.push(new THREE.Vector3(pointB.x, pointB.y, pointB.z));
        const geometry = new THREE.BufferGeometry().setFromPoints(points);
        const material = new THREE.LineBasicMaterial( { color: edge.color, transparent: true, opacity: 0.5 } );
        const line = new THREE.Line(geometry, material);
        window.scene.add(line);
    }
}

function d3_simple_obstacles() {
    const material = new THREE.MeshStandardMaterial( { color: 0x888888 } );
    for (const obstacle_index in obstacles) {
        const obstacle = obstacles[obstacle_index];
        const geometry = new THREE.BoxGeometry( obstacle.size[0], obstacle.size[1], obstacle.size[2] );
        const cube = new THREE.Mesh( geometry, material );
        cube.position.set(obstacle.origin[0]+0.5*obstacle.size[0], obstacle.origin[1]+0.5*obstacle.size[1], obstacle.origin[2]+0.5*obstacle.size[2]);
        window.scene.add(cube);
    }
}

function d3_simple_path() {

}

function d3_simple_border() {

}

function d3_simple_init() {
    d3_simple_setup();
    d3_simple_world();
    d3_simple_nodes();
    d3_simple_edges();
    d3_simple_path();
    d3_simple_obstacles();
    d3_simple_border();

    d3_simple_animate();
}

function d3_simple_animate() {
    requestAnimationFrame( d3_simple_animate );
    window.controls.update();
    window.renderer.render( window.scene, window.camera );
}

document.addEventListener('DOMContentLoaded', d3_simple_init);
