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
#include <newton/TrajectoryOptimizer.h>
#include "drivestates.hpp"
#include <unstructure/HybridAStar.hpp>

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

template <typename T, int size>
std::vector<aidrive::Vector3f> convertToTrajectory(T (&arr)[size])
{
    return convertToTrajectory<T, size>(&arr[0]);
}

template <typename T, int size>
std::vector<aidrive::Vector3f> convertToTrajectory(T* arr)
{
    std::vector<aidrive::Vector3f> out{};

    for (int i = 0; i < size / 2; ++i)
    {
        out.push_back(aidrive::Vector3f{static_cast<float32_t>(arr[2 * i]),
                                        static_cast<float32_t>(arr[2 * i + 1]),
                                        0.0f});
    }

    return out;
}

std::vector<aidrive::Vector3f> convertToTrajectory(const std::vector<aidrive::Vector2f>& in)
{
    std::vector<aidrive::Vector3f> out{};
    for (const auto& e : in)
    {
        out.push_back(aidrive::Vector3f{e[0], e[1], 0.0f});
    }
    return out;
}

void prepareContenderData(float64_t** XC, const aidrive::Vector3f p, const aidrive::Vector2f& v, size_t N)
{
    for (int i = 0; i < N; i++)
    {
        XC[i][0] = p[0] + v[0] * i;
        XC[i][1] = p[1] + v[1] * i;
        XC[i][2] = v[0];
        XC[i][3] = v[1];
    }
}

constexpr uint32_t WINDOW_WIDTH  = 1280u;
constexpr uint32_t WINDOW_HEIGHT = 720u;

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
    aidrive::Rect2f dim{5.0f, 2.3f}; // vehicle dimension

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
    float32_t kweight = 2.0f;
    bool piecewise{false};

    aidrive::planner::SpeedOpt speedOpt{};

    trajectoryOptimizer<float64_t, 30, 3> topt;
    topt.setMaxNrSteps(30);
    topt.setVerboseLevel(0);
    float64_t pointsIn[2 * (30 + 3)]{}; // in the format of {x1, y1, x2, y2} ...
                                        // {0...5} are history
                                        // {6...65} are inputs if something needs to be fixed
                                        // see topt.setFixed

    float32_t vInit{8.0f};
    float32_t vInitY{0.0f};
    float32_t aInit{0.0f};

    // state machine
    fsm_list::start();
    bool showSM{false};

    // calc fps
    // the container contains samples to calc fps
    // the container acts like a ring buffer
    std::deque<float32_t> fps{};
    constexpr size_t MAX_DEQUE_SIZE = 50;

    bool throttleOverrideOn{false};
    bool steerOverrideOn{false};
    bool waitingOn{false};

    // use TrajectoryOptimizer
    bool useTermCond{false}; // terminal condition
    float64_t v_long{SMOOTHNESS_COST_WEIGHT_LONG_VELOCITY_DEFAULT};
    float64_t a_long{SMOOTHNESS_COST_WEIGHT_LONG_ACCELERATION_DEFAULT};

    // obstacle data
    aidrive::Vector3f objP{40.0f, 0.0f, 0.0f};
    aidrive::Vector2f objV{0.0f, 0.0f};
    int nrXC[30];
    float64_t XC_mem[4 * 30]; // obstacle i @ time j
    float64_t* XC[30];

    for (int i = 0; i < 30; i++)
    {
        nrXC[i] = 1;
        XC[i]   = &XC_mem[4 * i];
    }

    // cost term default values
    bool progress_cost_on        = true;
    bool smoothness_cost_on      = true;
    bool limit_cost_on           = true;
    bool actor_avoidance_cost_on = true;
    bool prior_cost_on           = true;
    bool prior_exists            = false;
    bool lateral_cost_on         = true;

    // lateral cost
    scalarFunctionPolynomial<float64_t, 3> LatFun[30];
    scalarFunctionWeightedSquare<float64_t> LatWeight[30];
    int nrLaterals = 30;
    float64_t p[4]{}; // penalize distance to y = f(x) = 0;
    float64_t latWeight{0.0001};
    for (size_t i = 0; i < 30; ++i)
    {
        LatFun[i].set(p);
        LatWeight[i].setWeight(latWeight);
    }
    topt.setLaterals(LatFun, LatWeight, nrLaterals);

    HybridAStar astar{};
    HybridAStar::Params aStarParams{};

    int32_t numSteeringAngles = 1;
    float32_t angleTolerance  = 10.0f;
    astar.init(aStarParams.maxNumNodes, aStarParams.carTurningRadius, numSteeringAngles,
               aStarParams.cellSize, 1, angleTolerance,
               aStarParams.distWeight, aStarParams.dirSwitchCost,
               aStarParams.backwardsMultiplier, true, // true means using dstar heuristic, which speeds up the search hopefully. Right now it must be on
               aStarParams.maxNumCollisionCells, aStarParams.maxPathLength);
    // astar rendering
    std::vector<aidrive::Vector2f> searchLines{};
    std::vector<aidrive::Vector3f> dStarPath{};
    std::vector<aidrive::Vector3f> aStarPath{};

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

            ImGui::Begin("Hello, world!",
                         nullptr,
                         ImGuiWindowFlags_NoBringToFrontOnFocus | ImGuiWindowFlags_NoMove); // Create a window called "Hello, world!" and append into it.
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

                static ImGuiTabBarFlags tab_bar_flags = ImGuiTabBarFlags_Reorderable;
                if (ImGui::BeginTabBar("MyTabBar", tab_bar_flags))
                {
                    if (ImGui::BeginTabItem("main"))
                    {

                        // custom rendering
                        ImDrawList* drawList = ImGui::GetWindowDrawList();
                        m_renderer.setImDrawList(drawList);

                        ImGui::PushItemWidth(100.f);
                        ImGui::Checkbox("use terminal condition", &useTermCond);
                        ImGui::Checkbox("show state machine", &showSM);
                        ImGui::SliderFloat("x", &pose[0], -50.0f, 50.0f, "%.1f");
                        ImGui::SameLine();
                        ImGui::SliderFloat("y", &pose[1], -50.0f, 50.0f, "%.1f");
                        ImGui::SliderAngle("theta", &pose[2], 0.0f, 360.0f, "%.1f");
                        ImGui::SliderFloat("k", &curvature, -0.3f, 0.3f, "%.3f");
                        ImGui::SliderFloat("cte", &initError, -2.0f, 2.0f, "%.3f");
                        ImGui::SameLine();
                        ImGui::SliderFloat("kweight", &kweight, 0.0f, 50.0f, "%.1f");
                        ImGui::SameLine();
                        ImGui::Checkbox("peicewise", &piecewise);

                        // green curve
                        // gen traj

                        std::vector<aidrive::Vector3f> poly{};
                        if (!piecewise)
                        {
                            poly = aidrive::generatePolyline({0.0f, initError, 0.0f},
                                                             curvature,
                                                             0.5f,
                                                             30.0f);
                        }
                        else
                        {
                            // first piece
                            poly = aidrive::generatePolyline({0.0f, initError, 0.0f},
                                                             curvature,
                                                             0.5f,
                                                             15.0f);

                            // second piece
                            std::vector<aidrive::Vector3f> poly1 =
                                aidrive::generatePolyline({15.0f, 0, 0.0f},
                                                          curvature,
                                                          0.5f,
                                                          15.0f);
                            poly.insert(std::end(poly), std::begin(poly1), std::end(poly1));
                        }

                        m_renderer.drawPolyline(poly, aidrive::Vector3f{0.0f, 0.0f, 0.0f}, aidrive::render::COLOR_GREEN);

                        std::vector<aidrive::Vector3f> predPoly{};
                        ctrl.setCurvatureWeight(static_cast<float64_t>(kweight));
                        TIME_IT("path opt", predPoly = ctrl.optimize(poly));

                        // draw ego
                        m_renderer.drawRect(pose, dim);
                        // draw obstacle
                        m_renderer.drawRect(objP, dim);

                        m_renderer.drawPolyline(predPoly, pose, aidrive::render::COLOR_BLUE);

                        ImGui::EndTabItem();
                    }
                    if (ImGui::BeginTabItem("hybrid A*"))
                    {
                        ImGui::PushItemWidth(100);
                        // start pose
                        ImGui::SliderFloat("x", &pose[0], -50.0f, 50.0f, "%.1f");
                        ImGui::SameLine();
                        ImGui::SliderFloat("y", &pose[1], -50.0f, 50.0f, "%.1f");
                        ImGui::SameLine();
                        ImGui::SliderAngle("theta", &pose[2], 0.0f, 360.0f, "%.1f");

                        // end pose
                        ImGui::SliderFloat("obj x", &objP[0], 0.f, 50.0f, "%.1f");
                        ImGui::SameLine();
                        ImGui::SliderFloat("obj y", &objP[1], 0.f, 50.0f, "%.1f");
                        ImGui::SameLine();
                        ImGui::SliderFloat("obj theta", &objP[2], -3.14, 3.14, "%.1f");

                        ImGui::PopItemWidth();

                        // mock collision grid
                        std::shared_ptr<CollisionGrid> cg = astar.getDStarLite()->getCollisionGrid();
                        float32_t cellSize                = cg->getCellSize();
                        std::vector<float32_t> newCells{10.0f, 5.0f,
                                                        10.0f, 4.0f,
                                                        10.0f, -1.0f,
                                                        10.0f, 0.0f,
                                                        10.0f, 1.0f,
                                                        10.0f, 2.0f,
                                                        10.0f, 3.0f}; // format {x0, y0, x1, y1, x2, y2 ...}
                        cg->clear();
                        cg->addCells(&newCells[0], newCells.size() / 2);
                        cg->update();

                        aidrive::Rect2f cellDim{cellSize, cellSize}; // cell dimension
                        auto cells = cg->getCells();

                        for (auto it = cells->begin(); it != cells->end(); ++it)
                        {
                            aidrive::Vector2f cellCenter = it->getCenter(cellSize);
                            m_renderer.drawRect(aidrive::Vector3f(cellCenter.x(), cellCenter.y(), 0.0f),
                                                cellDim,
                                                aidrive::render::COLOR_BLACK);
                        }

                        if (ImGui::SmallButton("Plan A*"))
                        {
                            std::cout << "Plan A* now." << std::endl;
                            std::vector<aidrive::Vector3f> poly{};

                            searchLines.clear();
                            dStarPath.clear();
                            aStarPath.clear();

                            int32_t reedsSheppN = 100;
                            astar.findPath(aidrive::Vector2f{pose[0], pose[1]},
                                           aidrive::Vector2f{std::cos(pose[2]), std::sin(pose[2])},
                                           DrivingState::STANDING,
                                           aidrive::Vector2f{objP[0], objP[1]},
                                           aidrive::Vector2f{std::cos(objP[2]), std::sin(objP[2])}, // can expose this later
                                           DrivingState::STANDING,
                                           reedsSheppN,
                                           0);

                            uint32_t cnt = astar.getPathCount();
                            std::vector<aidrive::Vector2f> newPath(cnt);
                            std::vector<aidrive::Vector2f> newHeadings(cnt);
                            std::vector<DrivingState> newDirs(cnt);
                            astar.getPath(&newPath[0].x(), &newHeadings[0].x(), &newDirs[0], cnt);

                            aStarPath = convertToTrajectory(newPath);

                            constexpr bool renderSearchLine = true;
                            if (renderSearchLine)
                            {
                                std::cout << "renderSearchLine option is on" << std::endl;

                                uint32_t cnt = 0u;

                                astar.getSearchLines(nullptr, &cnt);
                                searchLines.resize(cnt * 2); // cnt is line count
                                                             // point count will be 2 * cnt
                                astar.getSearchLines(&searchLines[0].x(), &cnt);

                                cnt = 0u;
                                astar.getDStarLitePath(nullptr, &cnt);
                                newPath.resize(cnt);
                                astar.getDStarLitePath(&newPath[0].x(), &cnt);

                                dStarPath = convertToTrajectory(newPath);
                            }
                            else
                            {
                                std::cout << "renderSearchLine option is off" << std::endl;
                            }
                        }
                        m_renderer.drawSearchLines(searchLines, aidrive::render::COLOR_SILVER);

                        // draw ego
                        m_renderer.drawRect(pose, dim);
                        // draw obstacle
                        m_renderer.drawRect(objP, dim);
                        // draw reference
                        m_renderer.drawPolyline(aStarPath, aidrive::Vector3f{0.0f, 0.0f, 0.0f}, aidrive::render::COLOR_GREEN);
                        // don't need to tf the astar path to global.
                        m_renderer.drawPolyline(dStarPath, aidrive::Vector3f{0.0f, 0.0f, 0.0f}, aidrive::render::COLOR_ORANGE);

                        ImGui::EndTabItem();
                    }

                    if (ImGui::BeginTabItem("trajectory optimization"))
                    {
                        ImGui::PushItemWidth(100);
                        ImGui::SliderFloat("vInit", &vInit, 0.0f, 10.0f, "%.2f");
                        ImGui::SameLine();
                        ImGui::SliderFloat("vInitY", &vInitY, -10.0f, 10.0f, "%.2f");
                        ImGui::SameLine();
                        ImGui::SliderFloat("aInit", &aInit, -2.0f, 2.0f, "%.2f");
                        ImGui::SliderFloat("obj x", &objP[0], 0.f, 50.0f, "%.1f");
                        ImGui::SameLine();
                        ImGui::SliderFloat("obj y", &objP[1], 0.f, 50.0f, "%.1f");
                        ImGui::SameLine();
                        ImGui::SliderFloat("obj theta", &objP[2], -3.14, 3.14, "%.1f");
                        ImGui::SameLine();
                        ImGui::SliderFloat("obj vx", &objV[0], 0.f, 30.0f, "%.1f");
                        ImGui::SameLine();
                        ImGui::SliderFloat("obj vy", &objV[1], 0.f, 30.0f, "%.1f");

                        // parameters
                        float64_t delta_t =
                            topt.getDeltaT();
                        progressCostParameters<float64_t, 30>& progressP =
                            topt.getProgressCostParameters();
                        smoothnessCostParameters<float64_t, scalarFunctionWeightedSquare<float64_t>>&
                            smoothP = topt.getSmoothnessCostParameters();
                        limitCostParameters<float64_t, scalarFunctionSquaredBarrier<float64_t>>&
                            limitP = topt.getLimitCostParameters();
                        actorAvoidanceCostParameters<float64_t, scalarFunctionSquaredBarrier<float64_t>, scalarFunctionSquaredBarrier<float64_t>, scalarFunctionSquaredBarrier<float64_t>, scalarFunctionSquaredBarrier<float64_t>>
                            actorP = topt.getActorAvoidanceCostParameters();

                        ImGui::InputDouble("k_p", &progressP.k_p, 0.0, 0.0, "%.4f");
                        ImGui::SameLine();
                        ImGui::InputDouble("lat", &latWeight, 0.0, 0.0, "%0.4f");
                        ImGui::SameLine();
                        ImGui::InputDouble("v_long", &v_long, 0.0, 0.0, "%.7f");
                        ImGui::SameLine();
                        ImGui::InputDouble("a_long", &a_long, 0.0, 0.0, "%.7f");
                        ImGui::SameLine();
                        ImGui::InputDouble("v_ideal", &limitP.lon_v_ideal, 0.0, 0.0, "%.1f");
                        ImGui::SameLine();
                        ImGui::InputDouble("a_ideal", &limitP.lon_a_ideal, 0.0, 0.0, "%.1f");
                        ImGui::PopItemWidth();
                        smoothP.setDefault(1.0 / delta_t,
                                           SMOOTHNESS_COST_WEIGHT_LONG_POSITION_DEFAULT,
                                           SMOOTHNESS_COST_WEIGHT_LAT_POSITION_DEFAULT,
                                           v_long,
                                           SMOOTHNESS_COST_WEIGHT_LAT_VELOCITY_DEFAULT,
                                           a_long);
                        limitP.prepare(); // prepare must be called if any attribute of limitP is changed
                        actorP.setDefaultBySetPoint(progressP.k_p);
                        // this tunes time-gap
                        // actorP.printDistanceToVelocityTable(progressP.k_p);

                        // lateral weight
                        for (size_t i = 0; i < 30; ++i)
                        {
                            LatWeight[i].setWeight(latWeight);
                        }

                        // initial condition
                        pointsIn[4] = -vInit * delta_t;
                        pointsIn[5] = -vInitY * delta_t;
                        pointsIn[2] = -vInit * delta_t * 2;
                        pointsIn[3] = -vInitY * delta_t * 2;
                        pointsIn[0] = -vInit * delta_t * 3;
                        pointsIn[1] = -vInitY * delta_t * 3;

                        // terminal condition
                        int fixed[60]{0};
                        if (useTermCond)
                        {
                            pointsIn[64] = 20;
                            pointsIn[65] = -20;
                            pointsIn[62] = 20;
                            pointsIn[63] = -23;

                            fixed[59] = fixed[58] = fixed[57] = fixed[56] = 1;
                        }
                        topt.setFixed(fixed);
                        topt.setEgoTrajectory(&pointsIn[6]);

                        // obstacle
                        prepareContenderData(XC, objP, objV, 30);
                        topt.setContenderData(XC, nrXC);

                        // topt.set_all_off();
                        ImGui::Checkbox("actor", &actor_avoidance_cost_on);
                        if (actor_avoidance_cost_on)
                            topt.set_actor_avoidance_cost(true);
                        else
                            topt.set_actor_avoidance_cost(false);
                        ImGui::SameLine();

                        ImGui::Checkbox("progress", &progress_cost_on);
                        if (progress_cost_on)
                            topt.set_progress_cost(true);
                        else
                            topt.set_progress_cost(false);
                        ImGui::SameLine();

                        ImGui::Checkbox("smooth", &smoothness_cost_on);
                        if (smoothness_cost_on)
                            topt.set_smoothness_cost(true);
                        else
                            topt.set_smoothness_cost(false);
                        ImGui::SameLine();

                        ImGui::Checkbox("limit", &limit_cost_on);
                        if (limit_cost_on)
                            topt.set_limit_cost(true);
                        else
                            topt.set_limit_cost(false);
                        ImGui::SameLine();

                        ImGui::Checkbox("lateral", &lateral_cost_on);
                        if (lateral_cost_on)
                            topt.set_lateral_cost(true);
                        else
                            topt.set_lateral_cost(false);

                        // process traj opt
                        TIME_IT("traj opt", topt.run());
                        // note, the return excludes history (0 ... 5)
                        float64_t* result = topt.getEgoTrajectory();
                        // topt.printTrajectory();

                        // draw ego
                        m_renderer.drawRect(pose, dim);
                        // draw obstacle
                        m_renderer.drawRect(objP, dim);

                        std::vector<aidrive::Vector3f> predPoly = convertToTrajectory<float64_t, 60>(result);
                        m_renderer.drawPolyline(predPoly, pose, aidrive::render::COLOR_BLUE);

                        std::vector<float32_t> d(30, 0.0f);
                        std::vector<float32_t> v(30, 0.0f);
                        std::vector<float32_t> a(30, 0.0f);
                        std::vector<float32_t> j(30, 0.0f);

                        for (size_t i = 0; i < 30; ++i)
                        {
                            d[i] = static_cast<float32_t>(topt.getPosition(i));
                            v[i] = static_cast<float32_t>(topt.getVelocity(i));
                            a[i] = static_cast<float32_t>(topt.getAcceleration(i));
                            j[i] = static_cast<float32_t>(topt.getJerk(i));
                        }

                        const ImVec2 GRAPH_SIZE{400, 80};

                        ImGui::PlotLines("d", &d[0], d.size(), 0, nullptr, 0.0f, 50.0f, GRAPH_SIZE);
                        ImGui::PlotLines("v", &v[0], v.size(), 0, nullptr, 0.0f, 10.0f, GRAPH_SIZE);
                        ImGui::PlotLines("a", &a[0], a.size(), 0, nullptr, -3.0f, 3.0f, GRAPH_SIZE);
                        ImGui::PlotLines("j", &j[0], j.size(), 0, nullptr, -10.0f, 10.0f, GRAPH_SIZE);
                        ImGui::EndTabItem();
                    }

                    if (ImGui::BeginTabItem("state machine"))
                    {
                        {
                            ImGui::Text("UI events:");
                            ImGui::Indent();
                            if (ImGui::SmallButton("engage l1"))
                                send_event(EngageL1());
                            ImGui::SameLine();
                            if (ImGui::SmallButton("engage l2"))
                                send_event(EngageL2());
                            ImGui::SameLine();
                            if (ImGui::SmallButton("engage l3"))
                                send_event(EngageL3());
                            ImGui::SameLine();
                            if (ImGui::SmallButton("engage l4"))
                                send_event(EngageL4());
                            ImGui::SameLine();
                            if (ImGui::SmallButton("disengage"))
                                send_event(Disengage());
                            ImGui::SameLine();
                            if (ImGui::SmallButton("resume"))
                                send_event(Resume());
                            if (ImGui::SmallButton("est mission"))
                                send_event(EstablishMission());
                            ImGui::SameLine();
                            if (ImGui::SmallButton("act mission"))
                                send_event(ActivateMission());
                            ImGui::SameLine();
                            if (ImGui::SmallButton("deact mission"))
                                send_event(DeactivateMission());
                            ImGui::Unindent();
                        }
                        {
                            ImGui::Text("module events:");
                            ImGui::Indent();
                            if (ImGui::SmallButton("ReportLonReady"))
                                send_event(ReportLonReady());
                            ImGui::SameLine();
                            if (ImGui::SmallButton("ReportLatReady"))
                                send_event(ReportLatReady());
                            ImGui::SameLine();
                            if (ImGui::SmallButton("ReportL3Ready"))
                                send_event(ReportL3Ready());
                            ImGui::SameLine();
                            if (ImGui::SmallButton("ReportL3Fail"))
                                send_event(ReportL3Fail());
                            ImGui::SameLine();
                            if (ImGui::Checkbox("EnterWaiting", &waitingOn))
                            {
                                if (waitingOn)
                                    send_event(WaitingOnEvent());
                                else
                                    send_event(WaitingOffEvent());
                            }
                            ImGui::SameLine();
                            if (ImGui::SmallButton("EnterHolding"))
                                send_event(HoldingOnEvent());
                            if (ImGui::SmallButton("BrakeOverride"))
                                send_event(BrakeOverride());
                            ImGui::SameLine();
                            if (ImGui::Checkbox("ThrottleOverride", &throttleOverrideOn))
                            {
                                if (throttleOverrideOn)
                                    send_event(ThrottleOverride());
                                else
                                    send_event(StopThrottleOverride());
                            }
                            ImGui::SameLine();
                            if (ImGui::Checkbox("SteerOverride", &steerOverrideOn))
                            {
                                if (steerOverrideOn)
                                    send_event(SteerOverride());
                                else
                                    send_event(StopSteerOverride());
                            }
                            ImGui::SameLine();
                            if (ImGui::SmallButton("AEBTrigger"))
                                send_event(AEBTrigger());
                            ImGui::Unindent();
                        }
                        {
                            ImGui::Text("TOP:");
                            ImGui::Indent();
                            auto highlightByColor = [](TopEnum test) {
                                ImVec4 color;
                                color = (test == Top::getCurrentTopState())
                                            ? ImGui::ColorConvertU32ToFloat4(aidrive::render::COLOR_RED)
                                            : ImGui::ColorConvertU32ToFloat4(aidrive::render::COLOR_BLACK);
                                return color;
                            };

                            ImGui::TextColored(highlightByColor(TopEnum::MANUAL), "Man");
                            ImGui::SameLine();
                            ImGui::TextColored(highlightByColor(TopEnum::L1), "L1");
                            ImGui::SameLine();
                            ImGui::TextColored(highlightByColor(TopEnum::L2), "L2");
                            ImGui::SameLine();
                            ImGui::TextColored(highlightByColor(TopEnum::L3), "L3");
                            ImGui::SameLine();
                            ImGui::TextColored(highlightByColor(TopEnum::L4), "L4");
                            ImGui::Unindent();
                        }
                        {
                            ImGui::Text("LON:");
                            ImGui::Indent();
                            auto highlightByColor = [](LonEnum test) {
                                ImVec4 color;
                                color = (test == Lon::getCurrentLonState())
                                            ? ImGui::ColorConvertU32ToFloat4(aidrive::render::COLOR_RED)
                                            : ImGui::ColorConvertU32ToFloat4(aidrive::render::COLOR_BLACK);
                                return color;
                            };
                            ImGui::TextColored(highlightByColor(LonEnum::FAULT), "off");
                            ImGui::SameLine();
                            ImGui::TextColored(highlightByColor(LonEnum::READY), "ready");
                            ImGui::SameLine();
                            ImGui::TextColored(highlightByColor(LonEnum::CONTROLLING), "controlling");
                            ImGui::SameLine();
                            ImGui::TextColored(highlightByColor(LonEnum::OVERRIDEN), "override");
                            ImGui::SameLine();
                            ImGui::TextColored(highlightByColor(LonEnum::WAITING), "waiting");
                            ImGui::SameLine();
                            ImGui::TextColored(highlightByColor(LonEnum::HOLDING), "holding");
                            ImGui::Unindent();
                        }
                        {
                            ImGui::Text("LAT:");
                            ImGui::Indent();
                            auto highlightByColor = [](LatEnum test) {
                                ImVec4 color;
                                color = (test == Lat::getCurrentLatState())
                                            ? ImGui::ColorConvertU32ToFloat4(aidrive::render::COLOR_RED)
                                            : ImGui::ColorConvertU32ToFloat4(aidrive::render::COLOR_BLACK);
                                return color;
                            };
                            ImGui::TextColored(highlightByColor(LatEnum::FAULT), "off");
                            ImGui::SameLine();
                            ImGui::TextColored(highlightByColor(LatEnum::READY), "ready");
                            ImGui::SameLine();
                            ImGui::TextColored(highlightByColor(LatEnum::CONTROLLING), "controlling");
                            ImGui::SameLine();
                            ImGui::TextColored(highlightByColor(LatEnum::OVERRIDEN), "override");
                            ImGui::Unindent();
                        }
                        {
                            ImGui::Text("BP client:");
                            ImGui::Indent();
                            auto highlightByColor = [](BPEnum test) {
                                ImVec4 color;
                                color = (test == BP::getCurrentState())
                                            ? ImGui::ColorConvertU32ToFloat4(aidrive::render::COLOR_RED)
                                            : ImGui::ColorConvertU32ToFloat4(aidrive::render::COLOR_BLACK);
                                return color;
                            };
                            ImGui::TextColored(highlightByColor(BPEnum::FAULT), "off");
                            ImGui::SameLine();
                            ImGui::TextColored(highlightByColor(BPEnum::ACC), "ACC");
                            ImGui::SameLine();
                            ImGui::TextColored(highlightByColor(BPEnum::ACCLK), "ACCLK");
                            ImGui::SameLine();
                            ImGui::TextColored(highlightByColor(BPEnum::L3inTransit), "L3inTransit");
                            ImGui::SameLine();
                            ImGui::TextColored(highlightByColor(BPEnum::L3), "L3");
                            ImGui::Unindent();
                        }
                        {
                            ImGui::Text("Mission client:");
                            ImGui::Indent();
                            auto highlightByColor = [](MissionEnum test) {
                                ImVec4 color;
                                color = (test == Mission::getCurrentState())
                                            ? ImGui::ColorConvertU32ToFloat4(aidrive::render::COLOR_RED)
                                            : ImGui::ColorConvertU32ToFloat4(aidrive::render::COLOR_BLACK);
                                return color;
                            };
                            ImGui::TextColored(highlightByColor(MissionEnum::OFF), "off");
                            ImGui::SameLine();
                            ImGui::TextColored(highlightByColor(MissionEnum::ESTABLISHED), "EST");
                            ImGui::SameLine();
                            ImGui::TextColored(highlightByColor(MissionEnum::ACTIVATED), "ACT");
                            ImGui::SameLine();
                            ImGui::TextColored(highlightByColor(MissionEnum::ACTIVATED_NEW_ESTABLISHED), "ACT_NEW_EST");
                            ImGui::Unindent();
                        }
                        {
                            ImGui::Text("AEB client");
                            ImGui::Indent();
                            auto highlightByColor = [](AEBEnum test) {
                                ImVec4 color;
                                color = (test == AEB::getCurrentState())
                                            ? ImGui::ColorConvertU32ToFloat4(aidrive::render::COLOR_RED)
                                            : ImGui::ColorConvertU32ToFloat4(aidrive::render::COLOR_BLACK);
                                return color;
                            };
                            ImGui::TextColored(highlightByColor(AEBEnum::OFF), "off");
                            ImGui::SameLine();
                            ImGui::TextColored(highlightByColor(AEBEnum::MONITORING), "monitoring");
                            ImGui::SameLine();
                            ImGui::TextColored(highlightByColor(AEBEnum::ACTUATING), "actuating");
                            ImGui::SameLine();
                            ImGui::Unindent();
                        }

                        ImGui::EndTabItem();
                    }
                    if (ImGui::BeginTabItem("speed optimization"))
                    {
                        ImGui::PushItemWidth(100.f);

                        ImGui::SliderFloat("vInit", &vInit, 0.0f, 10.0f, "%.2f");
                        ImGui::SameLine();
                        ImGui::SliderFloat("aInit", &aInit, -2.0f, 2.0f, "%.2f");
                        ImGui::PopItemWidth();

                        ImGui::Checkbox("enable curve speed term", &speedOpt.getCurveSpeedToggle());

                        TIME_IT("speed opt", speedOpt.optimize(vInit, aInit));

                        const ImVec2 GRAPH_SIZE{600, 120};

                        auto d = speedOpt.getD();
                        ImGui::PlotLines("d", &d[0], d.size(), 0, nullptr, 0.0f, 50.0f, GRAPH_SIZE);

                        auto v = speedOpt.getV();
                        ImGui::PlotLines("v", &v[0], v.size(), 0, nullptr, 0.0f, 10.0f, GRAPH_SIZE);

                        auto a = speedOpt.getA();
                        ImGui::PlotLines("a", &a[0], a.size(), 0, nullptr, -3.0f, 3.0f, GRAPH_SIZE);

                        // auto j = speedOpt.getJ();
                        // ImGui::PlotLines("j", &j[0], j.size(), 0, nullptr, -10.0f, 10.0f, GRAPH_SIZE);

                        auto speedLimit = speedOpt.getSpeedLimit();
                        ImGui::PlotLines("v limit", &speedLimit[0], speedLimit.size(), 0, nullptr, 0.0f, 30.0f, GRAPH_SIZE);

                        // auto vAsFunctionOfD = speedOpt.getVAsFunctionOfD();
                        // ImGui::PlotLines("v", &vAsFunctionOfD[0], vAsFunctionOfD.size(), 0, nullptr, 0.0f, 30.0f, GRAPH_SIZE);
                        ImGui::EndTabItem();
                    }
                    ImGui::EndTabBar();
                } // BeginTabBar
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