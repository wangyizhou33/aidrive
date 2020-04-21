// GL
#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>
#include <GL/glew.h> // Initialize with glewInit()

// std
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <chrono>
#include <deque>
#include <numeric>
#include <Eigen/Dense>

// imgui
#include <imgui.h>
#include "imgui_impl_opengl3.h"
#include "imgui_impl_glfw.h"

// aidrive module headers
#include <render/Render.hpp>
#include <model/Model.hpp>
#include <aidrive/Profile.hpp>

static void error_callback(int error, const char* description)
{
    fprintf(stderr, "Error: %s\n", description);
}

static void key_callback(GLFWwindow* window, int key, int scancode, int action,
                         int mods)
{
    if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS)
        glfwSetWindowShouldClose(window, GLFW_TRUE);
}

void testEigen()
{
    using Eigen::MatrixXd;
    MatrixXd m(2, 2);
    m(0, 0) = 3;
    m(1, 0) = 2.5;
    m(0, 1) = -1;
    m(1, 1) = m(1, 0) + m(0, 1);
    std::cout << m << std::endl;
}

constexpr uint32_t WINDOW_WIDTH  = 800u;
constexpr uint32_t WINDOW_HEIGHT = 600u;

int main(void)
{

    testEigen();

    GLFWwindow* window;

    glfwSetErrorCallback(error_callback);

    if (!glfwInit())
        exit(EXIT_FAILURE);

    const char* glsl_version = "#version 130";
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);

    window = glfwCreateWindow(WINDOW_WIDTH, WINDOW_HEIGHT, "Simple example", NULL, NULL);
    if (!window)
    {
        glfwTerminate();
        exit(EXIT_FAILURE);
    }

    glfwSetKeyCallback(window, key_callback);

    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    // Initialize OpenGL loader
    glewInit();

    // Setup Dear ImGui context
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    (void)io;
    //io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;     // Enable Keyboard Controls
    //io.ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad;      // Enable Gamepad Controls

    // Setup Dear ImGui style
    ImGui::StyleColorsLight(); // Dark, Classic

    // Setup Platform/Renderer bindings
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init(glsl_version);

    ImVec4 clear_color = ImVec4(0.45f, 0.55f, 0.60f, 1.00f);

    // vehicle pose
    aidrive::Vector3f pose{0.f, 0.f, 0.f};
    aidrive::Rect2f dim{5.0f, 2.3f};

    // trajectory
    aidrive::Trajectory traj{};
    float32_t curvature{0.0};

    // initialize modules
    aidrive::render::Renderer m_renderer{WINDOW_WIDTH, WINDOW_HEIGHT};
    constexpr float32_t PIXEL_PER_METER = 10.f;
    m_renderer.setPixelPerMeter(PIXEL_PER_METER);
    m_renderer.setEye({pose[0], pose[1]});

    // calc fps
    // the container contains samples to calc fps
    // the container acts like a ring buffer
    std::deque<float32_t> fps{};
    constexpr size_t MAX_DEQUE_SIZE = 50;

    while (!glfwWindowShouldClose(window))
    {
        auto start = std::chrono::high_resolution_clock::now();

        glfwPollEvents();

        // Start the Dear ImGui frame
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();
        {
            ImGui::SetNextWindowPos(ImVec2(0, 0), ImGuiCond_Always);
            ImGui::SetNextWindowSize(ImVec2(WINDOW_WIDTH, WINDOW_HEIGHT), ImGuiCond_Always);

            ImGui::Begin("Hello, world!"); // Create a window called "Hello, world!" and append into it.
            {
                float32_t framePerSecond = std::accumulate(fps.begin(), fps.end(), 0.0f) / static_cast<float32_t>(MAX_DEQUE_SIZE);
                ImGui::Text("fps: %f", framePerSecond);

                // custom rendering
                ImDrawList* drawList = ImGui::GetWindowDrawList();
                m_renderer.setImDrawList(drawList);

                ImGui::PushItemWidth(150.f);
                ImGui::SliderFloat("x", &pose[0], -50.0f, 50.0f, "%.1f");
                ImGui::SliderFloat("y", &pose[1], -50.0f, 50.0f, "%.1f");
                ImGui::SliderAngle("theta", &pose[2], 0.0f, 360.0f, "%.1f");
                ImGui::SliderFloat("k", &curvature, -0.3f, 0.3f, "%.3f");
                ImGui::PopItemWidth();

                // gen traj
                std::vector<aidrive::Vector3f> poly = aidrive::generatePolyline({0.0f, 0.0f, 0.0f},
                                                                                curvature,
                                                                                1.0f,
                                                                                50.0f);

                // convert
                traj.points.clear();
                for (const aidrive::Vector3f& pose : poly)
                {
                    traj.points.push_back({.pos = {pose[0], pose[1]},
                                           .hdg = pose[2],
                                           .v   = 5.0f,
                                           .a   = 0.0f,
                                           .t   = 0});
                }

                m_renderer.drawRect(pose, dim);
                m_renderer.drawTrajectory(traj.points, pose);
            }
            ImGui::End();
        }

        // Rendering
        ImGui::Render();

        int display_w, display_h;
        glfwGetFramebufferSize(window, &display_w, &display_h);
        glViewport(0, 0, display_w, display_h);
        glClearColor(clear_color.x, clear_color.y, clear_color.z, clear_color.w);
        glClear(GL_COLOR_BUFFER_BIT);
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        glfwSwapBuffers(window);

        auto elapsed = std::chrono::high_resolution_clock::now() - start;

        fps.push_back(1e6f / std::chrono::duration_cast<std::chrono::microseconds>(elapsed).count());

        if (fps.size() > MAX_DEQUE_SIZE)
        {
            fps.pop_front(); // make sure the queue has at most MAX_QUEUE_SIZE elements;
        }
    }
    // Cleanup
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();

    glfwDestroyWindow(window);
    glfwTerminate();

    exit(EXIT_SUCCESS);
}