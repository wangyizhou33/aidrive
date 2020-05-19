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
#include <control/Control.hpp>
#include <aidrive/Profile.hpp>
#include <planner/SpeedOpt.hpp>

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

constexpr uint32_t WINDOW_WIDTH  = 800u;
constexpr uint32_t WINDOW_HEIGHT = 600u;

int main(void)
{
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
    float32_t curvature{0.0f};
    float32_t initError{0.0f}; // cross-track error

    // initialize modules
    aidrive::render::Renderer m_renderer{WINDOW_WIDTH, WINDOW_HEIGHT};
    constexpr float32_t PIXEL_PER_METER = 10.f;
    m_renderer.setPixelPerMeter(PIXEL_PER_METER);
    m_renderer.setEye({pose[0], pose[1]});

    aidrive::control::Controller ctrl{};

    aidrive::planner::SpeedOpt speedOpt{};

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
            ImGui::SetNextWindowPos(ImVec2(0, 0), ImGuiCond_Appearing);
            ImGui::SetNextWindowSize(ImVec2(WINDOW_WIDTH, WINDOW_HEIGHT), ImGuiCond_Appearing);

            ImGui::Begin("Hello, world!"); // Create a window called "Hello, world!" and append into it.
            {
                float32_t framePerSecond =
                    std::accumulate(fps.begin(), fps.end(), 0.0f) / static_cast<float32_t>(fps.size());
                ImGui::Text("fps: %.1f", framePerSecond);

                if (ImGui::IsMousePosValid())
                {
                    ImVec2 mousePixel          = io.MousePos;
                    aidrive::Vector2f mousePos = m_renderer.getCartesianCoordinates(mousePixel);
                    ImGui::Text("Mouse pixel: (%g, %g), cartesian: (%.1f, %.1f) ",
                                mousePixel.x, mousePixel.y,
                                mousePos[0], mousePos[1]);
                }
                else
                {
                    ImGui::Text("Mouse pos: <INVALID>");
                }

                // custom rendering
                ImDrawList* drawList = ImGui::GetWindowDrawList();
                m_renderer.setImDrawList(drawList);

                ImGui::PushItemWidth(150.f);
                ImGui::SliderFloat("x", &pose[0], -50.0f, 50.0f, "%.1f");
                ImGui::SliderFloat("y", &pose[1], -50.0f, 50.0f, "%.1f");
                ImGui::SliderAngle("theta", &pose[2], 0.0f, 360.0f, "%.1f");
                ImGui::SliderFloat("k", &curvature, -0.3f, 0.3f, "%.3f");
                ImGui::SliderFloat("cte", &initError, -2.0f, 2.0f, "%.3f");
                ImGui::PopItemWidth();

                // gen traj
                std::vector<aidrive::Vector3f> poly =
                    aidrive::generatePolyline({0.0f, initError, 0.0f},
                                              curvature,
                                              0.5f,
                                              50.0f);

                // control
                std::vector<aidrive::Vector3f> predPoly{};
                TIME_IT("path opt", predPoly = ctrl.optimize(poly));

                // convert
                // traj.points.clear();
                // for (const aidrive::Vector3f& pose : predPoly)
                // {
                //     traj.points.push_back({.pos = {pose[0], pose[1]},
                //                            .hdg = pose[2],
                //                            .v   = 5.0f,
                //                            .a   = 0.0f,
                //                            .t   = 0});
                // }

                m_renderer.drawRect(pose, dim);
                m_renderer.drawPolyline(poly, pose, aidrive::render::COLOR_GREEN);
                m_renderer.drawPolyline(predPoly, pose, aidrive::render::COLOR_BLUE);
            }
            ImGui::End();

            TIME_IT("speed opt", speedOpt.optimize());

            ImGui::SetNextWindowPos(ImVec2(0, WINDOW_HEIGHT - 300), ImGuiCond_Appearing);
            ImGui::SetNextWindowSize(ImVec2(350, 300), ImGuiCond_Appearing);

            ImGui::Begin("2d plot");
            {
                const ImVec2 GRAPH_SIZE{300, 60};

                auto d = speedOpt.getD();
                ImGui::PlotLines("d", &d[0], d.size(), 0, nullptr, 0.0f, 50.0f, GRAPH_SIZE);

                auto v = speedOpt.getV();
                ImGui::PlotLines("v", &v[0], v.size(), 0, nullptr, 0.0f, 10.0f, GRAPH_SIZE);

                auto a = speedOpt.getA();
                ImGui::PlotLines("a", &a[0], a.size(), 0, nullptr, -3.0f, 3.0f, GRAPH_SIZE);

                auto j = speedOpt.getJ();
                ImGui::PlotLines("j", &j[0], j.size(), 0, nullptr, -10.0f, 10.0f, GRAPH_SIZE);
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