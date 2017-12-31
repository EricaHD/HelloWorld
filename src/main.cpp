// README:
//   'U' scale the earth up
//   'D' scale the earth down
//   'R' to rotate the earth
//   'S' to spin the scene with the camera
//   '-' to zoom out with the camera (perspective mode only)
//   'B' toggle between terrain bump map to bricks bump map
//   Can change boolean at the top of the file to go between orthographic and perspective

// ACKNOWLEDGEMENTS:
//   Object data made by Glenn Campbell, ideastudio on www.free3d.com (https://free3d.com/3d-model/planet-earth-99065.html)
//   Maps supplied by NASA

// Libraries
#include "Helpers.h"
#include <GLFW/glfw3.h>
#include <Eigen/Dense>
#include <iostream>
#include <fstream>

// Globals: orthographic or perspective
bool orthographic = true;

// Globals: large data arrays and associated VBOs
Eigen::MatrixXf V(3,0);
VertexBufferObject VBO_V;
Eigen::MatrixXf N(3,0);
VertexBufferObject VBO_N;
Eigen::MatrixXf UV(2,0);
VertexBufferObject VBO_UV;
Eigen::MatrixXf OU(3,0);
VertexBufferObject VBO_OU;
Eigen::MatrixXf OV(3,0);
VertexBufferObject VBO_OV;

// Globals: for reading in .obj data
int num_vertices = 1986;
int num_uvcoords = 2143;
int num_triangles = 128;
int num_quads = 1920;
int num_faces = num_triangles + num_quads; // 2048
Eigen::MatrixXf vertices(3, num_vertices);
Eigen::MatrixXf normals(3, num_vertices);
Eigen::MatrixXf uvcoords(2, num_uvcoords);
Eigen::MatrixXi faces(12, num_faces);
Eigen::MatrixXf OU_partial_deriv(3, num_vertices);
Eigen::MatrixXf OV_partial_deriv(3, num_vertices);

// Globals: uniforms for shader, size 4x4
Eigen::Matrix4f view(4,4);
Eigen::Matrix4f model(4,4);
Eigen::Matrix4f projection(4,4);

// Globals: positions
Eigen::Vector3f light_position(2.6f, -0.1f, 1.5f); // (1.0f, 0.65f, 0.1f);
Eigen::Vector3f camera_position(0.0f, -0.1f, -2.0f);
Eigen::Vector3f gaze_direction(0.0f, 0.0f, 0.0f);
Eigen::Vector3f which_way_up(0.0f, 1.0f, 0.0f);

// Globals: .bmp image data
unsigned int img_width = 4096;
unsigned int img_height = 2048;
unsigned int imageSize = img_width * img_height * 3; // 3 because one byte for red, green, and blue component

// Globals: changing camera position
float tau = 6.28318531f;
float t = 3.0f * tau / 4.0f; // because camera starts at (0, 0, -2)

// Globals: epsilon
float epsilon_u = 0.003;
float epsilon_v = 0.005;

// Globals: starting using terrain bump map
bool use_terrain = true;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Load in data from .obj file into vertices, normals, uvcoords, and faces
// Assumes that faces come in the following order: all triangles, then all quadrilaterals
void load_obj_data(std::string& filename) {

    // Open file
    std::ifstream file;
    file.open(filename);

    // Trackers
    int vertex_i = 0;
    int normal_i = 0;
    int uv_i = 0;
    int face_i = 0;

    // Read lines
    while (!file.eof()) {
        for (std::string line; std::getline(file, line); ) {  // read stream line by line

            // Make a stream for the line, read the first white-space separated element
            std::istringstream in(line);
            std::string marker; in >> marker;

            // Read in vertices, vertex normals, texture coordinates, and polygonal faces
            if (marker == "v") {
                float x, y, z;
                in >> x >> y >> z;
                vertices.col(vertex_i) << x, y, z;
                vertex_i++;
            }
            else if (marker == "vn") {
                float x, y, z;
                in >> x >> y >> z;
                normals.col(normal_i) << x, y, z;
                normal_i++;
            }
            else if (marker == "vt") {
                float u, v;
                in >> u >> v;
                uvcoords.col(uv_i) << u, v;
                uv_i++;
            }
            else if (marker == "f") {
                int v1, t1, n1, v2, t2, n2, v3, t3, n3, v4, t4, n4;
                v4 = 0; t4 = 0; n4 = 0; // clear out garbage values in case there's no fourth point of face
                char slash;
                in >> v1 >> slash >> t1 >> slash >> n1 >> v2 >> slash >> t2 >> slash >> n2
                   >> v3 >> slash >> t3 >> slash >> n3 >> v4 >> slash >> t4 >> slash >> n4;
                faces.col(face_i) << v1-1, v2-1, v3-1, v4-1, t1-1, t2-1, t3-1, t4-1, n1-1, n2-1, n3-1, n4-1; // adjusting for 1-indexing
                face_i++;
            }

        } // end for
    } // end while

    // Close file
    file.close();

}

// Load in data from .bmp image file
void load_bmp_data(const char* imagename, unsigned char* data) {

    // Open the file
    FILE *file = std::fopen(imagename, "rb");
    if (!file) {
        printf("Image could not be opened\n");
        return;
    }

    // Read in 54 byte header
    unsigned char header[54];
    if (fread(header, 1, 54, file) != 54){
        printf("Not a correct BMP file\n");
        return;
    }
    if (header[0] != 'B' || header[1] != 'M'){
        printf("Not a correct BMP file\n");
        return;
    }

    // Read in actual data
    fread(data, 1, imageSize, file);

    // Close file
    fclose(file);

}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Solves for alpha, beta, gamma using area method
Eigen::Vector3f area_solver(Eigen::Vector2f p, Eigen::Vector2f v1_uv, Eigen::Vector2f v2_uv, Eigen::Vector2f v3_uv) {

    // Alias vertices for name
    float ax = v1_uv(0);
    float ay = v1_uv(1);
    float bx = v2_uv(0);
    float by = v2_uv(1);
    float cx = v3_uv(0);
    float cy = v3_uv(1);

    // Compute areas
    float total_area = abs(((bx - ax) * (cy - ay) - (cx - ax) * (by - ay)) / 2.0f);
    float alpha_area = abs(((bx - p(0)) * (cy - p(1)) - (cx - p(0)) * (by - p(1))) / 2.0f);
    float beta_area = abs(((p(0) - ax) * (cy - ay) - (cx - ax) * (p(1) - ay)) / 2.0f);
    float gamma_area = abs(((bx - ax) * (p(1) - ay) - (p(0) - ax) * (by - ay)) / 2.0f);

    // Solve system for alpha, beta, and gamma
    float alpha = alpha_area / total_area;
    float beta = beta_area / total_area;
    float gamma = gamma_area / total_area;

    // Load alpha, beta, gamma into result vector and return
    Eigen::Vector3f result;
    result << alpha, beta, gamma;
    return result;

}

// Populate V, N, UV
void populate_V_N_UV() {

    // Resize large data arrays to 3 * num_triangles + 6 * num_quads
    V.conservativeResize(3, 3 * num_triangles + 6 * num_quads);
    N.conservativeResize(3, 3 * num_triangles + 6 * num_quads);
    UV.conservativeResize(2, 3 * num_triangles + 6 * num_quads);

    // Tracker
    int V_i = 0;

    // Add triangular faces to vertex array V
    for (int i = 0; i < num_triangles; i++) {

        // Vertex 1
        int v1_num = faces(0, i);
        int n1_num = faces(8, i);
        int t1_num = faces(4, i);
        // Vertex 2
        int v2_num = faces(1, i);
        int n2_num = faces(9, i);
        int t2_num = faces(5, i);
        // Vertex 3
        int v3_num = faces(2, i);
        int n3_num = faces(10, i);
        int t3_num = faces(6, i);

        // Fill in V, N, UV
        V.col(V_i) = vertices.col(v1_num);
        N.col(V_i) = normals.col(n1_num);
        UV.col(V_i) = uvcoords.col(t1_num);
        V_i++;
        V.col(V_i) = vertices.col(v2_num);
        N.col(V_i) = normals.col(n2_num);
        UV.col(V_i) = uvcoords.col(t2_num);
        V_i++;
        V.col(V_i) = vertices.col(v3_num);
        N.col(V_i) = normals.col(n3_num);
        UV.col(V_i) = uvcoords.col(t3_num);
        V_i++;

    }

    // Add quadrilateral faces (by splitting it into two triangles) to vertex array V
    for (int i = num_triangles; i < num_faces; i++) {

        // Vertex 1
        int v1_num = faces(0, i);
        int n1_num = faces(8, i);
        int t1_num = faces(4, i);
        // Vertex 2
        int v2_num = faces(1, i);
        int n2_num = faces(9, i);
        int t2_num = faces(5, i);
        // Vertex 3
        int v3_num = faces(2, i);
        int n3_num = faces(10, i);
        int t3_num = faces(6, i);
        // Vertex 4
        int v4_num = faces(3, i);
        int n4_num = faces(11, i);
        int t4_num = faces(7, i);

        // Triangle v1 -> v2 -> v3
        V.col(V_i) = vertices.col(v1_num);
        N.col(V_i) = normals.col(n1_num);
        UV.col(V_i) = uvcoords.col(t1_num);
        V_i++;
        V.col(V_i) = vertices.col(v2_num);
        N.col(V_i) = normals.col(n2_num);
        UV.col(V_i) = uvcoords.col(t2_num);
        V_i++;
        V.col(V_i) = vertices.col(v3_num);
        N.col(V_i) = normals.col(n3_num);
        UV.col(V_i) = uvcoords.col(t3_num);
        V_i++;

        // Triangle v1 -> v3 -> v4
        V.col(V_i) = vertices.col(v1_num);
        N.col(V_i) = normals.col(n1_num);
        UV.col(V_i) = uvcoords.col(t1_num);
        V_i++;
        V.col(V_i) = vertices.col(v3_num);
        N.col(V_i) = normals.col(n3_num);
        UV.col(V_i) = uvcoords.col(t3_num);
        V_i++;
        V.col(V_i) = vertices.col(v4_num);
        N.col(V_i) = normals.col(n4_num);
        UV.col(V_i) = uvcoords.col(t4_num);
        V_i++;

    }

    // Upload the change to the GPU
    VBO_V.update(V);
    VBO_N.update(N);
    VBO_UV.update(UV);

}

// Preprae OU, OV by filling OU_partial_deriv and OV_partial_deriv
void prepare_OU_OV() {

    // Add triangular faces to vertex array V
    for (int i = 0; i < num_triangles; i++) {

        // Vertex 1
        int v1_num = faces(0, i);
        int t1_num = faces(4, i);
        // Vertex 2
        int v2_num = faces(1, i);
        int t2_num = faces(5, i);
        // Vertex 3
        int v3_num = faces(2, i);
        int t3_num = faces(6, i);

        // Pick three points in UV coordinates
        Eigen::Vector2f v1_uv = uvcoords.col(t1_num);
        Eigen::Vector2f v2_uv = uvcoords.col(t2_num);
        Eigen::Vector2f v3_uv = uvcoords.col(t3_num);

        // Pick three points in UV coordinates
        Eigen::Vector2f p = (v1_uv + v2_uv + v3_uv) / 3.0;
        Eigen::Vector2f p_u = p; p_u(0) += epsilon_u;
        Eigen::Vector2f p_v = p; p_v(1) += epsilon_v;

        // Solve for alpha, beta, gamma that solve system
        Eigen::Vector3f abg = area_solver(p, v1_uv, v2_uv, v3_uv);
        Eigen::Vector3f abg_u = area_solver(p_u, v1_uv, v2_uv, v3_uv);
        Eigen::Vector3f abg_v = area_solver(p_v, v1_uv, v2_uv, v3_uv);

        // Compute corresponding vectors in XYZ coordinates
        Eigen::Vector3f v = abg(0) * vertices.col(v1_num) + abg(1) * vertices.col(v2_num) + abg(2) * vertices.col(v3_num);
        Eigen::Vector3f v_u = abg_u(0) * vertices.col(v1_num) + abg_u(1) * vertices.col(v2_num) + abg_u(2) * vertices.col(v3_num);
        Eigen::Vector3f v_v = abg_v(0) * vertices.col(v1_num) + abg_v(1) * vertices.col(v2_num) + abg_v(2) * vertices.col(v3_num);

        // Compute partial derivatives
        Eigen::Vector3f O_u = v_u - v;
        Eigen::Vector3f O_v = v_v - v;

        // Attribute these partial derivatives to v1_num, v2_num, v3_num
        OU_partial_deriv.col(v1_num) += O_u;
        OU_partial_deriv.col(v2_num) += O_u;
        OU_partial_deriv.col(v3_num) += O_u;
        OV_partial_deriv.col(v1_num) += O_v;
        OV_partial_deriv.col(v2_num) += O_v;
        OV_partial_deriv.col(v3_num) += O_v;

    }

    // Add quadrilateral faces (by splitting it into two triangles) to vertex array V
    for (int i = num_triangles; i < num_faces; i++) {

        // Vertex 1
        int v1_num = faces(0, i);
        int t1_num = faces(4, i);
        // Vertex 2
        int v2_num = faces(1, i);
        int t2_num = faces(5, i);
        // Vertex 3
        int v3_num = faces(2, i);
        int t3_num = faces(6, i);
        // Vertex 4
        int v4_num = faces(3, i);
        int t4_num = faces(7, i);

        // Triangle v1 -> v2 -> v3

        // Pick three points in UV coordinates
        Eigen::Vector2f v1_uv = uvcoords.col(t1_num);
        Eigen::Vector2f v2_uv = uvcoords.col(t2_num);
        Eigen::Vector2f v3_uv = uvcoords.col(t3_num);

        // Pick three points in UV coordinates
        Eigen::Vector2f p = (v1_uv + v2_uv + v3_uv) / 3.0;
        Eigen::Vector2f p_u = p; p_u(0) += epsilon_u;
        Eigen::Vector2f p_v = p; p_v(1) += epsilon_v;

        // Solve for alpha, beta, gamma that solve system
        Eigen::Vector3f abg = area_solver(p, v1_uv, v2_uv, v3_uv);
        Eigen::Vector3f abg_u = area_solver(p_u, v1_uv, v2_uv, v3_uv);
        Eigen::Vector3f abg_v = area_solver(p_v, v1_uv, v2_uv, v3_uv);

        // Compute corresponding vectors in XYZ coordinates
        Eigen::Vector3f v = abg(0) * vertices.col(v1_num) + abg(1) * vertices.col(v2_num) + abg(2) * vertices.col(v3_num);
        Eigen::Vector3f v_u = abg_u(0) * vertices.col(v1_num) + abg_u(1) * vertices.col(v2_num) + abg_u(2) * vertices.col(v3_num);
        Eigen::Vector3f v_v = abg_v(0) * vertices.col(v1_num) + abg_v(1) * vertices.col(v2_num) + abg_v(2) * vertices.col(v3_num);

        // Compute partial derivatives
        Eigen::Vector3f O_u = v_u - v;
        Eigen::Vector3f O_v = v_v - v;

        // Attribute these partial derivatives to v1_num, v2_num, v3_num
        OU_partial_deriv.col(v1_num) += O_u;
        OU_partial_deriv.col(v2_num) += O_u;
        OU_partial_deriv.col(v3_num) += O_u;
        OV_partial_deriv.col(v1_num) += O_v;
        OV_partial_deriv.col(v2_num) += O_v;
        OV_partial_deriv.col(v3_num) += O_v;

        // Triangle v1 -> v3 -> v4

        // Pick three points in UV coordinates
        v1_uv = uvcoords.col(t1_num);
        v3_uv = uvcoords.col(t3_num);
        Eigen::Vector2f v4_uv = uvcoords.col(t4_num);

        // Pick three points in UV coordinates
        p = (v1_uv + v3_uv + v4_uv) / 3.0;
        p_u = p; p_u(0) += epsilon_u;
        p_v = p; p_v(1) += epsilon_v;

        // Solve for alpha, beta, gamma that solve system
        abg = area_solver(p, v1_uv, v3_uv, v4_uv);
        abg_u = area_solver(p_u, v1_uv, v3_uv, v4_uv);
        abg_v = area_solver(p_v, v1_uv, v3_uv, v4_uv);

        // Compute corresponding vectors in XYZ coordinates
        v = abg(0) * vertices.col(v1_num) + abg(1) * vertices.col(v3_num) + abg(2) * vertices.col(v4_num);
        v_u = abg_u(0) * vertices.col(v1_num) + abg_u(1) * vertices.col(v3_num) + abg_u(2) * vertices.col(v4_num);
        v_v = abg_v(0) * vertices.col(v1_num) + abg_v(1) * vertices.col(v3_num) + abg_v(2) * vertices.col(v4_num);

        // Compute partial derivatives
        O_u = v_u - v;
        O_v = v_v - v;

        // Attribute these partial derivatives to v1_num, v2_num, v3_num
        OU_partial_deriv.col(v1_num) += O_u;
        OU_partial_deriv.col(v3_num) += O_u;
        OU_partial_deriv.col(v4_num) += O_u;
        OV_partial_deriv.col(v1_num) += O_v;
        OV_partial_deriv.col(v3_num) += O_v;
        OV_partial_deriv.col(v4_num) += O_v;

    }

    // Normalize everything
    for (int i = 0; i < num_vertices; i++) {
        (OU_partial_deriv.col(i)).normalized();
        (OV_partial_deriv.col(i)).normalized();
    }

}

// Populate OU, OV
void populate_OU_OV() {

    // Resize large data arrays to 3 * num_triangles + 6 * num_quads
    OU.conservativeResize(3, 3 * num_triangles + 6 * num_quads);
    OV.conservativeResize(3, 3 * num_triangles + 6 * num_quads);

    // Tracker
    int V_i = 0;

    // Add triangular faces to vertex array V
    for (int i = 0; i < num_triangles; i++) {

        // Vertex 1
        int v1_num = faces(0, i);
        // Vertex 2
        int v2_num = faces(1, i);
        // Vertex 3
        int v3_num = faces(2, i);

        // Fill in V, N, UV
        OU.col(V_i) = OU_partial_deriv.col(v1_num);
        OV.col(V_i) = OV_partial_deriv.col(v1_num);
        V_i++;
        OU.col(V_i) = OU_partial_deriv.col(v2_num);
        OV.col(V_i) = OV_partial_deriv.col(v2_num);
        V_i++;
        OU.col(V_i) = OU_partial_deriv.col(v3_num);
        OV.col(V_i) = OU_partial_deriv.col(v3_num);
        V_i++;

    }

    // Add quadrilateral faces (by splitting it into two triangles) to vertex array V
    for (int i = num_triangles; i < num_faces; i++) {

        // Vertex 1
        int v1_num = faces(0, i);
        // Vertex 2
        int v2_num = faces(1, i);
        // Vertex 3
        int v3_num = faces(2, i);
        // Vertex 4
        int v4_num = faces(3, i);

        // Triangle v1 -> v2 -> v3
        OU.col(V_i) = OU_partial_deriv.col(v1_num);
        OV.col(V_i) = OV_partial_deriv.col(v1_num);
        V_i++;
        OU.col(V_i) = OU_partial_deriv.col(v2_num);
        OV.col(V_i) = OV_partial_deriv.col(v2_num);
        V_i++;
        OU.col(V_i) = OU_partial_deriv.col(v3_num);
        OV.col(V_i) = OV_partial_deriv.col(v3_num);
        V_i++;

        // Triangle v1 -> v3 -> v4
        OU.col(V_i) = OU_partial_deriv.col(v1_num);
        OV.col(V_i) = OV_partial_deriv.col(v1_num);
        V_i++;
        OU.col(V_i) = OU_partial_deriv.col(v3_num);
        OV.col(V_i) = OV_partial_deriv.col(v3_num);
        V_i++;
        OU.col(V_i) = OU_partial_deriv.col(v4_num);
        OV.col(V_i) = OV_partial_deriv.col(v4_num);
        V_i++;

    }

    // Upload the change to the GPU
    VBO_OU.update(OU);
    VBO_OV.update(OV);

}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Whenever a key is pressed
void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods) {

    // Avoid unused variable warnings
    (void) window;
    (void) scancode;
    (void) mods;

    // Which key is pressed?
    switch (key) {
        case  GLFW_KEY_S: // spin the scene via the camera
            if (action == GLFW_REPEAT) {
                t += 0.07;
                if (t > tau) {
                    t -= tau;
                }
                float dist = sqrt(pow(camera_position(0), 2.0f) + pow(camera_position(2), 2.0f));
                camera_position(0) = dist * cos(t);
                camera_position(2) = dist * sin(t);
            }
            break;
        case GLFW_KEY_MINUS: // zoom camera out; only works in perspective
            if (action == GLFW_REPEAT) {
                if (!orthographic) {
                    camera_position(2) -= 0.2f;
                }
            }
            break;
        case GLFW_KEY_R: // rotate the earth around Y axis
            if (action == GLFW_REPEAT) {
                double radians = -0.03;
                Eigen::MatrixXf rotation_matrix(4,4);
                rotation_matrix << cos(radians), 0.0f, sin(radians), 0.0f,
                        0.0f, 1.0f, 0.0f, 0.0f,
                        -1*sin(radians), 0.0f, cos(radians), 0.0f,
                        0.0f, 0.0f, 0.0f, 1.0f;
                model = rotation_matrix * model;
            }
            break;
        case GLFW_KEY_U: // scale up
            if (action == GLFW_REPEAT) {
                double scale_factor = 1.1;
                Eigen::MatrixXf scaling_matrix(4,4);
                scaling_matrix << scale_factor, 0.0f, 0.0f, 0.0f,
                        0.0f, scale_factor, 0.0f, 0.0f,
                        0.0f, 0.0f, scale_factor, 0.0f,
                        0.0f, 0.0f, 0.0f, 1.0f;
                model = scaling_matrix * model;
            }
            break;
        case GLFW_KEY_D: // scale down
            if (action == GLFW_REPEAT) {
                double scale_factor = 0.9;
                Eigen::MatrixXf scaling_matrix(4,4);
                scaling_matrix << scale_factor, 0.0f, 0.0f, 0.0f,
                        0.0f, scale_factor, 0.0f, 0.0f,
                        0.0f, 0.0f, scale_factor, 0.0f,
                        0.0f, 0.0f, 0.0f, 1.0f;
                model = scaling_matrix * model;
            }
            break;
        case GLFW_KEY_B: // toggle bump mapping on/off
            if (action == GLFW_RELEASE) {
                use_terrain = !use_terrain;
            }
            break;
        default:
            break;
    }

}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Main
int main() {

    GLFWwindow *window;

    // Initialize the library
    if (!glfwInit())
        return -1;

    // Activate supersampling
    glfwWindowHint(GLFW_SAMPLES, 8);

    // Ensure that we get at least a 3.2 context
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 2);

    // On apple we have to load a core profile with forward compatibility
#ifdef __APPLE__
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
#endif

    // Create a windowed mode window and its OpenGL context
    window = glfwCreateWindow(640, 480, "Hello World", nullptr, nullptr);
    if (!window) {
        glfwTerminate();
        return -1;
    }

    // Make the window's context current
    glfwMakeContextCurrent(window);

#ifndef __APPLE__
    glewExperimental = true;
      GLenum err = glewInit();
      if(GLEW_OK != err)
      {
        /* Problem: glewInit failed, something is seriously wrong. */
       fprintf(stderr, "Error: %s\n", glewGetErrorString(err));
      }
      glGetError(); // pull and savely ignonre unhandled errors like GL_INVALID_ENUM
      fprintf(stdout, "Status: Using GLEW %s\n", glewGetString(GLEW_VERSION));
#endif

    int major, minor, rev;
    major = glfwGetWindowAttrib(window, GLFW_CONTEXT_VERSION_MAJOR);
    minor = glfwGetWindowAttrib(window, GLFW_CONTEXT_VERSION_MINOR);
    rev = glfwGetWindowAttrib(window, GLFW_CONTEXT_REVISION);
    printf("OpenGL version recieved: %d.%d.%d\n", major, minor, rev);
    printf("Supported OpenGL is %s\n", (const char *) glGetString(GL_VERSION));
    printf("Supported GLSL is %s\n", (const char *) glGetString(GL_SHADING_LANGUAGE_VERSION));

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // VAO /////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    // Initialize the VAO
    // A Vertex Array Object (or VAO) is an object that describes how the vertex attributes are stored in a Vertex
    // Buffer Object (or VBO). This means that the VAO is not the actual object storing the vertex data, but the
    // descriptor of the vertex data.
    VertexArrayObject VAO;
    VAO.init();
    VAO.bind();

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // LARGE DATA ARRAYS AND ASSOCIATED VBOS ///////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    // Initialize the VBO with the vertices data
    VBO_V.init();
    V.resize(3, 0);
    VBO_V.update(V);

    // Initialize the VBO with the vertex normal data
    VBO_N.init();
    N.resize(3, 0);
    VBO_N.update(N);

    // Initialize the VBO with the UV coordinates data
    VBO_UV.init();
    UV.resize(2, 0);
    VBO_UV.update(UV);

    VBO_OU.init();
    OU.resize(3,0);
    VBO_OU.update(OU);

    VBO_OV.init();
    OV.resize(3,0);
    VBO_OV.update(OV);

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // LOADING DATA AND POPULATE VBO DATA ARRAYS ///////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    // Make sure arrays have their space in memory
    vertices.resize(3, num_vertices);
    normals.resize(3, num_vertices);
    uvcoords.resize(2, num_uvcoords);
    faces.resize(12, num_faces);
    OU_partial_deriv.resize(3, num_vertices);
    OV_partial_deriv.resize(3, num_vertices);

    // Clear out garbage values in partial derivative holders, because we do +=
    for (int i = 0; i < num_vertices; i++) {
        OU_partial_deriv.col(i) << 0.0f, 0.0f, 0.0f;
        OV_partial_deriv.col(i) << 0.0f, 0.0f, 0.0f;
    }

    // Load in data
    std::string filename = "../data/sphere1.obj";
    load_obj_data(filename);

    // Populate V, N, UV
    populate_V_N_UV();

    // Prepare to file OU, OV
    prepare_OU_OV();

    // Populate OU, OV
    populate_OU_OV();

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // MODEL ///////////////////////////////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    // Update model
    if (orthographic) {
        model << 1.0 / 10000.0, 0.0, 0.0, 0.0,
                0.0, 1.0 / 10000.0, 0.0, 0.0,
                0.0, 0.0, 1.0 / 10000.0, 0.0,
                0.0, 0.0, 0.0, 1.0;
    }
    else {
        model << 1.0 / 500.0, 0.0, 0.0, 0.0,
                0.0, 1.0 / 500.0, 0.0, 0.0,
                0.0, 0.0, 1.0 / 500.0, 0.0,
                0.0, 0.0, 0.0, 1.0;
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // SHADERS /////////////////////////////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    // Initialize the OpenGL Program with vertex shader and fragment shader
    Program program;
    const GLchar *vertex_shader =
            "#version 150 core\n"
                    "in vec3 position;"
                    "in vec3 normal;"
                    "in vec2 UV;"
                    "in vec3 Ou;"
                    "in vec3 Ov;"

                    "out vec3 f_position;"
                    "out vec3 f_normal;"
                    "out vec2 f_UV;"
                    "out vec3 f_Ou;"
                    "out vec3 f_Ov;"

                    "uniform mat4 model;" // 4x4 model matrix
                    "uniform mat4 view;"
                    "uniform mat4 projection;"

                    "void main()"
                    "{"

                    // Set gl_Position
                    "    gl_Position = projection * view * model * vec4(position, 1.0);"

                    // Send out vectors
                    "    f_position = vec3(model * vec4(position, 1.0));"
                    "    f_normal = normal;"
                    "    f_UV = UV * vec2(-1.0, 1.0);" // adjusted
                    "    f_Ou = Ou;"
                    "    f_Ov = Ov;"

                    "}";
    const GLchar* fragment_shader =
            "#version 150 core\n"
                    "in vec3 f_position;"
                    "in vec3 f_normal;"
                    "in vec2 f_UV;"
                    "in vec3 f_Ou;"
                    "in vec3 f_Ov;"

                    "out vec4 outColor;"

                    "uniform mat3 inverseTranspose;" // (upper left 3x3 model matrix) inverse transpose
                    "uniform vec3 light_position;"
                    "uniform vec3 camera_position;"

                    "uniform bool ortho_bool;"
                    "uniform bool use_terrain_bool;"

                    "uniform sampler2D day;"
                    "uniform sampler2D night;"
                    "uniform sampler2D bump;"
                    "uniform sampler2D bricks;"

                    "void main()"
                    "{"

                    // Compute light direction, view direction, light bisector
                    "    vec3 light_direction = normalize(light_position - f_position);"
                    "    vec3 view_direction = normalize(-1.0 * camera_position);" // orthographic
                    "    if (ortho_bool == false) { view_direction = normalize(camera_position - f_position); }" // perspective
                    "    vec3 light_bisector = normalize(light_direction + view_direction);"

                    // Compute normal
                    // Without bump mapping, vec3 bump_normal = normalize(inverseTranspose * f_normal);
                    "    vec3 n = normalize(inverseTranspose * f_normal);"
                    "    vec3 O_u = normalize(inverseTranspose * f_Ou);"
                    "    vec3 O_v = normalize(inverseTranspose * f_Ov);"
                    "    float F_u = 0.0;"
                    "    float F_v = 0.0;"
                    "    if (use_terrain_bool == true) {"
                    "        F_u = dFdx(texture(bump, f_UV).x);"
                    "        F_v = dFdy(texture(bump, f_UV).y);"
                    "    }"
                    "    else {"
                    "        F_u = dFdx(texture(bricks, f_UV).x);"
                    "        F_v = dFdy(texture(bricks, f_UV).y);"
                    "    }"
                    "    vec3 bump_normal = n + F_u * (cross(n, O_v)) - F_v * (cross(n, O_u));"
                    "    bump_normal = normalize(bump_normal);"

                    // Compute light
                    "    vec3 diffuse = 1.2 * clamp(dot(light_direction, bump_normal), 0.0, 1.0) * vec3(1.0, 1.0, 1.0);"
                    "    vec3 specular = 0.4 * pow(clamp(dot(light_bisector, bump_normal), 0.0, 1.0), 100.0) * vec3(1.0, 1.0, 1.0);"
                    "    vec3 ambient = 0.25 * vec3(1.0, 1.0, 1.0);"
                    "    vec3 color = diffuse + specular + ambient;"

                    // Set outColor
                    "    outColor = texture(day, f_UV) * (color.x) * vec4(color, color.x);" // day
                    "    outColor += texture(night, f_UV) * (1.0 - color.x) * vec4(0.3, 0.3, 0.3, 1.0);" // night
                    "}";

    // Compile the two shaders and upload the binary to the GPU
    // Note that we have to explicitly specify that the output "slot" called outColor is the one that we want in the
    // fragment buffer (and thus on screen)
    program.init(vertex_shader, fragment_shader, "outColor");
    program.bind();

    // Bind VBOs and update uniforms
    program.bindVertexAttribArray("position", VBO_V);
    program.bindVertexAttribArray("normal", VBO_N);
    program.bindVertexAttribArray("UV", VBO_UV);
    program.bindVertexAttribArray("Ou", VBO_OU);
    program.bindVertexAttribArray("Ov", VBO_OV);
    glUniformMatrix4fv(program.uniform("model"), 1, GL_FALSE, model.data());

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // TEXTURES ////////////////////////////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    // Load in day image
    const char *dayimage = "../data/4096_earth.bmp";
    unsigned char *day_data;
    day_data = new unsigned char [imageSize];
    load_bmp_data(dayimage, day_data);

    // Load in night image
    const char *nightimage = "../data/4096_night_lights.bmp";
    unsigned char *night_data;
    night_data = new unsigned char [imageSize];
    load_bmp_data(nightimage, night_data);

    // Load in bump image
    const char *bumpimage = "../data/4096_bump.bmp";
    unsigned char *bump_data;
    bump_data = new unsigned char [imageSize];
    load_bmp_data(bumpimage, bump_data);

    // Load in bricks image
    const char *bricksimage = "../data/other_bump.bmp";
    unsigned char *bricks_data;
    bricks_data = new unsigned char [imageSize];
    load_bmp_data(bricksimage, bricks_data);

    // We will have two textures
    auto *textures = new GLuint[4];
    glGenTextures(4, textures);
    glUniform1i(program.uniform("day"), 0);
    glUniform1i(program.uniform("night"), 1);
    glUniform1i(program.uniform("bump"), 2);
    glUniform1i(program.uniform("bricks"), 3);

    // Create, bind, fill, configure day texture
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, textures[0]);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, img_width, img_height, 0, GL_BGR, GL_UNSIGNED_BYTE, day_data);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR); // other option: GL_NEAREST
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR); // other option: GL_NEAREST

    // Create, bind, fill, configure night texture
    glActiveTexture(GL_TEXTURE1);
    glBindTexture(GL_TEXTURE_2D, textures[1]);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, img_width, img_height, 0, GL_BGR, GL_UNSIGNED_BYTE, night_data);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR); // other option: GL_NEAREST
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR); // other option: GL_NEAREST

    // Create, bind, fill, configure terrain texture
    glActiveTexture(GL_TEXTURE2);
    glBindTexture(GL_TEXTURE_2D, textures[2]);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, img_width, img_height, 0, GL_BGR, GL_UNSIGNED_BYTE, bump_data);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR); // other option: GL_NEAREST
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR); // other option: GL_NEAREST

    // Create, bind, fill, configure bricks texture
    glActiveTexture(GL_TEXTURE3);
    glBindTexture(GL_TEXTURE_2D, textures[3]);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, img_width, img_height, 0, GL_BGR, GL_UNSIGNED_BYTE, bricks_data);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR); // other option: GL_NEAREST
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR); // other option: GL_NEAREST

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // LISTENERS ///////////////////////////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    // Register the keyboard callback
    glfwSetKeyCallback(window, key_callback);

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // WHILE LOOP //////////////////////////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    // Loop until the user closes the window
    while (!glfwWindowShouldClose(window)) {

        // Bind your VAO (not necessary if you have only one)
        VAO.bind();

        // Bind your program
        program.bind();

        // Clear the framebuffer; background color (black)
        glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // Enable depth test
        glEnable(GL_DEPTH_TEST);

        // Update model uniform
        glUniformMatrix4fv(program.uniform("model"), 1, GL_FALSE, model.data());

        // Compute inverse transpose of model matrix (which can be a 3x3)
        Eigen::Matrix3f temp_model(3,3);
        temp_model << model(0,0), model(0,1), model(0,2),
                model(1,0), model(1,1), model(1,2),
                model(2,0), model(2,1), model(2,2);
        Eigen::Matrix3f model_IT= (temp_model.inverse()).transpose();
        glUniformMatrix3fv(program.uniform("inverseTranspose"), 1, GL_FALSE, model_IT.data());

        // Update view (camera movement)
        Eigen::Vector3f w = -1 * (gaze_direction - camera_position).normalized();
        Eigen::Vector3f u = (which_way_up.cross(w)).normalized();
        Eigen::Vector3f v = w.cross(u);
        Eigen::Matrix4f view1(4,4);
        view1 << u(0), u(1), u(2), 0.0,
                v(0), v(1), v(2), 0.0,
                w(0), w(1), w(2), 0.0,
                0.0, 0.0, 0.0, 1.0;
        Eigen::Matrix4f view2(4,4);
        view2 << 1.0, 0.0, 0.0, -1 * camera_position(0),
                0.0, 1.0, 0.0, -1 * camera_position(1),
                0.0, 0.0, 1.0, -1 * camera_position(2),
                0.0, 0.0, 0.0, 1.0;
        view = view1 * view2;
        glUniformMatrix4fv(program.uniform("view"), 1, GL_FALSE, view.data());

        // Update projection
        int width, height;
        glfwGetWindowSize(window, &width, &height); // gets size of window
        float theta = 1.0472; // view angle = 60 degrees in radians
        float n = -0.1f; // z coordinate of front face
        float f = -100.0f;
        float t = (float) tan(theta / 2.0) * abs(n);
        float b = -1 * t;
        float r = t * width / height;
        float l = -1 * r;
        if (orthographic) {
            projection << 2.0 / (r - l), 0.0, 0.0, -(r + l) / (r - l),
                    0.0, 2.0 / (t - b), 0.0, -(t + b) / (t - b),
                    0.0, 0.0, 2.0 / (n - f), -(n + f) / (n - f),
                    0.0, 0.0, 0.0, 1.0;
        }
        else {
            projection << 2 * abs(n) / (r - l), 0, (r + l) / (r - l), 0,
                    0, 2 * abs(n) / (t - b), (t + b) / (t - b), 0,
                    0, 0, -1 * (abs(n) + abs(f)) / (abs(n) - abs(f)), -2 * abs(f) * abs(n) / (abs(n) - abs(f)),
                    0, 0, -1, 0;
        }
        glUniformMatrix4fv(program.uniform("projection"), 1, GL_FALSE, projection.data());

        // Update uniforms
        glUniform3f(program.uniform("light_position"), light_position(0), light_position(1), light_position(2));
        glUniform3f(program.uniform("camera_position"), camera_position(0), camera_position(1), camera_position(2));
        glUniform1i(program.uniform("ortho_bool"), orthographic);
        glUniform1i(program.uniform("use_terrain_bool"), use_terrain);

        // Draw triangles in white
        glDrawArrays(GL_TRIANGLES, 0, (int) V.cols()); // (GL_TRIANGLES, start, number vertices to be rendered)

        // Draw wireframe
//        for (int i = 0; i < V.cols(); i+=3) {
//            glDrawArrays(GL_LINE_LOOP, i, 3);
//        }

        // Swap front and back buffers
        glfwSwapBuffers(window);

        // Poll for and process events
        glfwPollEvents();

    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // FINISHED ////////////////////////////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    // Deallocate opengl memory
    program.free();
    VAO.free();
    VBO_V.free();
    VBO_N.free();
    VBO_UV.free();
    VBO_OU.free();
    VBO_OV.free();

    // Deallocate glfw internals
    glfwTerminate();
    return 0;

}