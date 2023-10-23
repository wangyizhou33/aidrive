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
#include <random>
#include <Eigen/Dense>

// imgui
#include <imgui.h>
#include "imgui_impl_opengl3.h"
#include "imgui_impl_glfw.h"
#include <implot.h>

// aidrive module headers
#include <render/Render.hpp>
#include <model/Model.hpp>
#include <control/Control.hpp>
#include <aidrive/Profile.hpp>
#include <planner/SpeedOpt.hpp>
#include <newton/TrajectoryOptimizer.h>
#include "drivestates.hpp"
#include <unstructure/HybridAStar.hpp>
#include <estimator/histogram.hpp>
#include <estimator/holo_histogram.hpp>
#include <distance_following/DistanceFollowing.hpp>
#include <bezier/bezier.h>

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

void fromDequeToVector(const std::deque<float32_t>& dq,
                       const std::deque<float32_t>& dqW,
                       std::vector<float32_t>& hist,
                       float32_t binSize,
                       uint32_t numberOfBins,
                       float32_t limitMin)
{
    // Bin data from deque to histogram represented by a vector
    for (uint32_t i = 0; i < dq.size(); ++i)
    {
        if (dq[i] < limitMin)
            continue;

        const uint32_t bin = static_cast<uint32_t>(std::floor((dq[i] - limitMin) / binSize));

        if (bin >= numberOfBins)
            continue;

        hist[bin] += dqW[i];
    }
}

bezier::Bezier<3> fitBezier(float64_t x1,
                            float64_t y1,
                            float64_t t1,
                            float64_t k1,
                            float64_t x2,
                            float64_t y2,
                            float64_t t2,
                            float64_t k2)
{
    // first fix p0, p3
    bezier::Point p0(x1, y1);
    bezier::Point p3(x2, y2);
    bezier::Point p2(x2 - 1.0 * std::cos(t2), y2 - 1.0 * std::sin(t2));
    bezier::Point p1(0.0, 0.0);

    float64_t s1_min = 0.01;
    float64_t s1_max = 100.0;

    size_t cnt  = 0;
    float64_t k1Fit = 0.;
    do
    {
        float64_t s1 = 0.5 * (s1_min + s1_max);
    
        p1 = bezier::Point(x1 + s1 * std::cos(t1), y1 + s1 * std::sin(t1));
        bezier::Bezier<3> cubicBezier({p0, p1, p2, p3});

        k1Fit = cubicBezier.curvature(0.0);

        // k decreases when s increases
        if (k1Fit > 0.0)
        {
            if (k1 < k1Fit)
            {
                s1_min = s1;
            }
            else
            {
                s1_max = s1;
            }
        }
        else // k1Fit < 0
        {
            if (k1 > k1Fit)
            {
                s1_min = s1;
            }
            else
            {
                s1_max = s1;
            }
        }
        std::cout << "iter " << cnt << " target k: " << k1Fit << " current k: " << k1 <<  " current s: " << s1 << std::endl;
    }while(std::abs(k1 - k1Fit) > 0.001 && ++cnt < 100);

    return bezier::Bezier<3>({p0, p1, p2, p3});
}

constexpr uint32_t WINDOW_WIDTH  = 1280u;
constexpr uint32_t WINDOW_HEIGHT = 1080u;

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
    // io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;     // Enable Keyboard Controls
    // io.ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad;      // Enable Gamepad Controls

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

    // Estimator
    float32_t limitMin     = -1.0f;
    float32_t limitMax     = 1.0f;
    uint32_t numberOfBins  = 100u;
    uint32_t sizeHistogram = 10000u;
    float32_t binSize      = (limitMax - limitMin) / float32_t(numberOfBins);

    std::unique_ptr<RollingHistogram<float32_t>> rollingHistogram;
    rollingHistogram.reset(new RollingHistogram<float32_t>({limitMin, limitMax}, numberOfBins, sizeHistogram));

    std::vector<float> samples{};
    std::default_random_engine generator;
    float32_t guassianMean = 0.f;
    float32_t guassianStd  = 0.3f;
    float32_t uniformMin   = -1.0f;
    float32_t uniformMax   = 1.0f;

    // resulting real histogram to render
    std::vector<float32_t> histogramSimulator(numberOfBins, 0);
    std::deque<float32_t> dq;
    std::deque<float32_t> dqW;

    // slider variable
    int newNumSamples{10};

    //
    HoloHistogram::Parameters histParam("", binSize, std::pair<float32_t, float32_t>(limitMin, limitMax), 100.f, false);
    HoloHistogram holoHistogram(histParam);

    // lon sim
    aidrive::ODEModel<float32_t> lonModel{};
    float32_t& paramA       = lonModel.getParamA();
    float32_t& paramB       = lonModel.getParamB();
    float32_t& paramHeadway = lonModel.getParamHeadway();
    float32_t& paramDelta   = lonModel.getParamDelta();
    float32_t& paramV0      = lonModel.getParamV0();

    float32_t initEgoV{0.0};
    float32_t initObsV{0.0};
    float32_t initObsS{50.0};

    // bezier control points
    bezier::Point p0(0.0, 0.0);
    bezier::Point p3(10.0, 0.0);
    float32_t start_hdg(0.0);
    float32_t end_hdg(0.0);
    float32_t start_len(1.0f);
    float32_t end_len(-1.0f);

    bezier::Point p1(p0.x + start_len * std::cos(start_hdg), p0.y + start_len * std::sin(start_hdg));
    bezier::Point p2(p3.x + end_len * std::cos(end_hdg), p3.y + end_len * std::sin(end_hdg));

    float32_t start_k{0.0};
    float32_t end_k{0.0};

    float32_t k0{0.0};
    float32_t k1{0.0};
    float32_t ke{0.0};

    std::vector<aidrive::Vector3f> bezierCurve{};
    std::vector<aidrive::Vector3f> bezierBbox{};

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
                        ImGui::Text(" note: ReedsShepp is obstacle unaware!");
                        if (ImGui::SmallButton("Plan ReedsShepp"))
                        {
                            ReedsShepp reedsShepp{};
                            reedsShepp.setParameters(aStarParams.carTurningRadius,
                                                     aStarParams.backwardsMultiplier,
                                                     aStarParams.dirSwitchCost);
                            std::vector<RSPoint> RSPath;
                            RSPath.resize(100);
                            uint32_t pathPointCount = 0;
                            float64_t pathLength    = std::numeric_limits<float64_t>::max();

                            RSPoint startState;
                            startState.x           = pose[0];
                            startState.y           = pose[1];
                            startState.orientation = pose[2];
                            startState.direction   = static_cast<RSDirection>(DrivingState::STANDING);

                            RSPoint targetState;

                            targetState.x           = objP[0];
                            targetState.y           = objP[1];
                            targetState.orientation = objP[2];
                            targetState.direction   = static_cast<RSDirection>(DrivingState::STANDING);
                            reedsShepp.evaluateRS(&RSPath[0],
                                                  &pathPointCount,
                                                  &pathLength,
                                                  startState,
                                                  targetState,
                                                  aStarParams.cellSize,
                                                  static_cast<uint32_t>(RSPath.size()));

                            searchLines.clear();
                            dStarPath.clear();
                            aStarPath.clear();
                            for (size_t i = 0u; i < pathPointCount; ++i)
                            {
                                const RSPoint& p = RSPath.at(i);
                                aStarPath.emplace_back(p.x, p.y, p.orientation);
                            }
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
                        ImGui::PushItemWidth(50.f);

                        ImGui::SliderFloat("vInit", &vInit, 0.0f, 10.0f, "%.2f");
                        ImGui::SameLine();
                        ImGui::SliderFloat("aInit", &aInit, -2.0f, 2.0f, "%.2f");
                        ImGui::SameLine();
                        ImGui::SliderFloat("obj x", &objP[0], 0.f, 50.0f, "%.1f");
                        ImGui::SameLine();
                        ImGui::SliderFloat("obj vx", &objV[0], 0.f, 10.0f, "%.1f");
                        ImGui::SameLine();
                        ImGui::SliderFloat("rho", &speedOpt.getRho(), 0.01f, 0.5f, "%.2f");

                        ImGui::SliderFloat("eq", &speedOpt.getEqualityWeight(), 0.0f, 500000.0f, "%f");
                        ImGui::SameLine();
                        ImGui::SliderFloat("ineq", &speedOpt.getInequalityWeight(), 0.0f, 500000.0f, "%f");
                        ImGui::SameLine();
                        ImGui::SliderFloat("jerk", &speedOpt.getJerkWeight(), 0.0f, 2000.0f, "%.1f");
                        ImGui::SameLine();
                        ImGui::SliderFloat("accel", &speedOpt.getAccelWeight(), 0.0f, 2000.0f, "%.1f");
                        ImGui::SameLine();
                        ImGui::SliderFloat("vel", &speedOpt.getVelWeight(), 0.0f, 2000.0f, "%.1f");
                        ImGui::SameLine();
                        ImGui::SliderFloat("obs", &speedOpt.getObsWeight(), 0.0f, 20000.0f, "%.1f");
                        ImGui::SameLine();
                        ImGui::SliderInt("cut in ", &speedOpt.getCutInTime(), 0, 80.0f, "%d");

                        ImGui::PopItemWidth();

                        ImGui::Checkbox("enable curve speed term", &speedOpt.getCurveSpeedToggle());

                        // draw ego
                        m_renderer.drawRect(pose, dim);
                        // draw obstacle
                        m_renderer.drawRect(objP, dim);

                        TIME_IT("speed opt", speedOpt.optimize(vInit, aInit, objP[0], objV[0]));

                        const ImVec2 GRAPH_SIZE{600, 120};

                        auto t          = speedOpt.getT();
                        auto d          = speedOpt.getD();
                        auto v          = speedOpt.getV();
                        auto a          = speedOpt.getA();
                        auto j          = speedOpt.getJ();
                        auto od         = speedOpt.getObsD();
                        auto ov         = speedOpt.getObsV();
                        auto speedLimit = speedOpt.getSpeedLimit();

                        ImGui::Text("boundary: d: %f vs %f, v: %f vs %f, a: %f vs %f", 0.0f, d[0], vInit, v[0], aInit, a[0]);

                        static float xs1[1001], ys1[1001];
                        for (int i = 0; i < 1001; ++i)
                        {
                            xs1[i] = i * 0.001f;
                            ys1[i] = 0.5f + 0.5f * sin(50 * xs1[i]);
                        }
                        ImGui::BeginPlot("d", "", "", {-1, 250});
                        ImGui::PushPlotColor(ImPlotCol_Line,
                                             ImGui::ColorConvertU32ToFloat4(aidrive::render::COLOR_BLUE));
                        ImGui::Plot("ego d", t.data(), d.data(), t.size());
                        ImGui::PopPlotColor();
                        ImGui::PushPlotColor(ImPlotCol_Line,
                                             ImGui::ColorConvertU32ToFloat4(aidrive::render::COLOR_RED));
                        ImGui::Plot("obs d", t.data(), od.data(), t.size());
                        ImGui::PopPlotColor();
                        ImGui::EndPlot();

                        ImGui::BeginPlot("v", "", "", {-1, 250});
                        ImGui::PushPlotColor(ImPlotCol_Line,
                                             ImGui::ColorConvertU32ToFloat4(aidrive::render::COLOR_BLUE));
                        ImGui::Plot("ego v", t.data(), v.data(), t.size());
                        ImGui::PopPlotColor();
                        ImGui::PushPlotColor(ImPlotCol_Line,
                                             ImGui::ColorConvertU32ToFloat4(aidrive::render::COLOR_RED));
                        ImGui::Plot("obs v", t.data(), ov.data(), t.size());
                        ImGui::PopPlotColor();
                        ImGui::EndPlot();

                        ImGui::BeginPlot("a", "", "", {-1, 250});
                        ImGui::Plot("a", t.data(), a.data(), t.size());

                        ImGui::Plot("j", t.data(), j.data(), t.size());
                        ImGui::EndPlot();

                        // ImGui::BeginPlot("speed limit", "", "", {-1,300});
                        // ImGui::Plot("speedlimit", t.data(), speedLimit.data(), t.size());
                        // ImGui::EndPlot();

                        // auto vAsFunctionOfD = speedOpt.getVAsFunctionOfD();
                        // ImGui::PlotLines("v", &vAsFunctionOfD[0], vAsFunctionOfD.size(), 0, nullptr, 0.0f, 30.0f, GRAPH_SIZE);
                        ImGui::EndTabItem();
                    }
                    if (ImGui::BeginTabItem("Robust estimator"))
                    {

                        const ImVec2 GRAPH_SIZE{600, 400};
                        ImGui::PushItemWidth(100.f);

                        ImGui::SliderFloat("guassian mean", &guassianMean, -0.5, 0.5);
                        ImGui::SliderFloat("guassian std", &guassianStd, 0.0, 0.3);
                        ImGui::SliderFloat("uniform min", &uniformMin, -1.0, 1.0);
                        ImGui::SliderFloat("uniform max", &uniformMax, -1.0, 1.0);

                        std::normal_distribution<float32_t> normDistribution(guassianMean, guassianStd);
                        std::uniform_real_distribution<float32_t> uniformDistribution(uniformMin, uniformMax);

                        ImGui::SliderInt("add # sample", &newNumSamples, 0, 1000);

                        auto addSample = [&dq, &dqW, &rollingHistogram, &holoHistogram, sizeHistogram](float32_t sample) {
                            // print the random samples if you want
                            // std::cout << "sample " << sample << std::endl;
                            if (dq.size() == sizeHistogram)
                            {
                                dq.pop_front();
                                dqW.pop_front();
                            }

                            dq.push_back(sample);
                            dqW.push_back(1.0f);
                            if (rollingHistogram->insert(sample, 1.0f)) // uniform weight 1.0
                            {
                                // do nothing
                            }
                            holoHistogram.Run(sample);
                        };

                        if (ImGui::Button("draw from normal dist"))
                        {
                            for (size_t counter = 0u; counter < newNumSamples; ++counter)
                            {
                                float32_t sample = normDistribution(generator);
                                addSample(sample);
                            }
                            histogramSimulator = std::vector<float32_t>(numberOfBins, 0);
                            fromDequeToVector(dq, dqW, histogramSimulator, binSize, numberOfBins, limitMin);
                        }

                        if (ImGui::Button("draw from uniform dist"))
                        {
                            for (size_t counter = 0u; counter < newNumSamples; ++counter)
                            {
                                float32_t sample = uniformDistribution(generator);
                                addSample(sample);
                            }
                            histogramSimulator = std::vector<float32_t>(numberOfBins, 0);
                            fromDequeToVector(dq, dqW, histogramSimulator, binSize, numberOfBins, limitMin);
                        }

                        ImGui::PopItemWidth();
                        // histogram statistics
                        ImGui::Text("rolling mode: %lu", rollingHistogram->modeIdx());
                        ImGui::Text("rolling mean: %.2f", rollingHistogram->mean());
                        ImGui::Text("rolling kurtosis: %.2f", rollingHistogram->computeKurtosis());
                        ImGui::Text("rolling modeCountRatio: %.3f", rollingHistogram->computeModeCountRatio());
                        ImGui::Text("holo kurtosis: %.2f", holoHistogram.computeKurtosis());
                        ImGui::PlotHistogram("Histogram",
                                             histogramSimulator.data(),
                                             histogramSimulator.size(),
                                             0,
                                             NULL,
                                             0,
                                             rollingHistogram->modeCount() + 50, // so the histogram fills the plot
                                             GRAPH_SIZE);
                        ImGui::EndTabItem();
                    }

                    if (ImGui::BeginTabItem("lon sim"))
                    {
                        ImGui::PushItemWidth(100.f);

                        ImGui::SliderFloat("a", &paramA, 0.01, 1.0);
                        ImGui::SliderFloat("b", &paramB, 0.01, 3.0);
                        ImGui::SliderFloat("headway", &paramHeadway, 0.8, 2.0);
                        ImGui::SliderFloat("delta", &paramDelta, 0.0, 4.0);
                        ImGui::SliderFloat("v0", &paramV0, 20.0f, 50.0);

                        ImGui::SliderFloat("initEgoV", &initEgoV, 0.0, 5.0);
                        ImGui::SliderFloat("initObsV", &initObsV, 0.0, 10.0);
                        ImGui::SliderFloat("initObsS", &initObsS, 0.0, 80.0);

                        ImGui::PopItemWidth();

                        std::vector<float32_t> egoS{};
                        std::vector<float32_t> egoV{};
                        std::vector<float32_t> obsS{};
                        std::vector<float32_t> obsV{};
                        std::vector<float32_t> deltaS{};
                        std::vector<float32_t> deltaV{};

                        std::vector<float32_t> state{};
                        state.resize(4);
                        state[0] = 0.f;
                        state[1] = initEgoV;
                        state[2] = initObsS;
                        state[3] = initObsV;

                        float32_t deltaT = 0.1f;
                        float32_t finalT = 30.0f;

                        for (float32_t t = 0.0f; t < finalT; t += deltaT)
                        {
                            boost::numeric::odeint::integrate(lonModel,
                                                              state,
                                                              t,
                                                              t + deltaT,
                                                              std::min(0.1f, deltaT));

                            egoS.push_back(state[0]);
                            egoV.push_back(state[1]);
                            obsS.push_back(state[2]);
                            obsV.push_back(state[3]);
                            deltaS.push_back(state[2] - state[0]);
                            deltaV.push_back(state[3] - state[1]);
                        }

                        // theoretical curve
                        auto curve  = lonModel.getEq();
                        auto desire = lonModel.getDesire();

                        const ImVec2 GRAPH_SIZE{800, 80};

                        ImGui::PlotLines("ego d", &egoS[0], egoS.size(), 0, nullptr, 0.0f, 200.0f, GRAPH_SIZE);
                        ImGui::PlotLines("ego v", &egoV[0], egoV.size(), 0, nullptr, 0.0f, 20.0f, GRAPH_SIZE);
                        ImGui::PlotLines("obs d", &obsS[0], obsS.size(), 0, nullptr, 0.0f, 200.0f, GRAPH_SIZE);
                        ImGui::PlotLines("obs v", &obsV[0], obsV.size(), 0, nullptr, 0.0f, 20.0f, GRAPH_SIZE);
                        // ImGui::PlotLines("ds = obs - ego", &deltaS[0], deltaS.size(), 0, nullptr, 0.0f, 20.0f, GRAPH_SIZE);
                        // ImGui::PlotLines("dv = obs - ego", &deltaV[0], deltaV.size(), 0, nullptr, -20.0f, 20.0f, GRAPH_SIZE);
                        ImGui::PlotLines("eq", &curve[0], curve.size(), 0, nullptr, 0.0f, 200.0f, GRAPH_SIZE);
                        ImGui::PlotLines("desire", &desire[0], desire.size(), 0, nullptr, 0.0f, 200.0f, GRAPH_SIZE);

                        ImGui::EndTabItem();
                    }

                    if (ImGui::BeginTabItem("bezier"))
                    {

                        m_renderer.setPixelPerMeter(50.f);

                        ImGui::PushItemWidth(100.f);

                        ImGui::InputDouble("p0_x", &p0.x, 0.0, 0.0, "%.1f");
                        ImGui::SameLine();
                        ImGui::InputDouble("p0_y", &p0.y, 0.0, 0.0, "%.1f");

                        ImGui::InputDouble("p1_x", &p1.x, 0.0, 0.0, "%.1f");
                        ImGui::SameLine();
                        ImGui::InputDouble("p1_y", &p1.y, 0.0, 0.0, "%.1f");

                        ImGui::InputDouble("p2_x", &p2.x, 0.0, 0.0, "%.1f");
                        ImGui::SameLine();
                        ImGui::InputDouble("p2_y", &p2.y, 0.0, 0.0, "%.1f");

                        ImGui::InputDouble("p3_x", &p3.x, 0.0, 0.0, "%.1f");
                        ImGui::SameLine();
                        ImGui::InputDouble("p3_y", &p3.y, 0.0, 0.0, "%.1f");

                        if (ImGui::SliderAngle("start hdg", &start_hdg, -180.f, 180.f))
                        {
                            p1 = bezier::Vec2(p0.x + start_len * std::cos(start_hdg),
                                              p0.y + start_len * std::sin(start_hdg));
                        }
                        ImGui::SameLine();
                        if (ImGui::SliderFloat("start len", &start_len, -0.1f, 20.f))
                        {
                            p1 = bezier::Vec2(p0.x + start_len * std::cos(start_hdg),
                                              p0.y + start_len * std::sin(start_hdg));
                        }
                        if (ImGui::SliderAngle("end hdg", &end_hdg, -180.f, 180.f))
                        {
                            p2 = bezier::Vec2(p3.x + end_len * std::cos(end_hdg),
                                              p3.y + end_len * std::sin(end_hdg));
                        }
                        ImGui::SameLine();
                        if (ImGui::SliderFloat("end len", &end_len, -20.0f, -0.1f))
                        {
                            p2 = bezier::Vec2(p3.x + end_len * std::cos(end_hdg),
                                              p3.y + end_len * std::sin(end_hdg));
                        }

                        ImGui::SliderFloat("start k", &start_k, -1.0f, 1.0f);
                        ImGui::SliderFloat("end k", &end_k, -1.0f, 1.0f);


                        ImGui::PopItemWidth();

                        aidrive::Rect2f pt_dim{1.0f, 1.0f}; // vehicle dimension

                        auto convert = [](const bezier::Point& p) {
                            return aidrive::Vector3f({static_cast<float32_t>(p.x),
                                                      static_cast<float32_t>(p.y),
                                                      0.f});
                        };

                        m_renderer.drawRect(convert(p0), pt_dim);
                        m_renderer.drawRect(convert(p1), pt_dim);
                        m_renderer.drawRect(convert(p2), pt_dim);
                        m_renderer.drawRect(convert(p3), pt_dim);


                        // if (ImGui::SmallButton("compute"))
                        // {
                            // Create a cubic bezier with 4 points.
                            bezier::Bezier<3> cubicBezier({p0, p1, p2, p3});

                            bezierCurve.clear();
                            for (size_t i = 0u; i <= 100; ++i)
                            {
                                bezierCurve.push_back(
                                    convert(cubicBezier.valueAt(static_cast<float64_t>(i) / 100.)));
                            }

                            bezierBbox.clear();
                            bezier::TightBoundingBox bbox = cubicBezier.tbb();
                            {
                                bezierBbox.push_back(convert(bbox[0]));
                                bezierBbox.push_back(convert(bbox[1]));
                                bezierBbox.push_back(convert(bbox[2]));
                                bezierBbox.push_back(convert(bbox[3]));
                                bezierBbox.push_back(convert(bbox[0]));
                            }

                            k0 = cubicBezier.curvature(0.0);
                            k1 = cubicBezier.curvature(1.0);
                            ke = cubicBezier.maxCurvatureNumeric();
                        // }

                        if (ImGui::SmallButton("fit"))
                        {
                            bezier::Bezier<3> cubicBezier = fitBezier(p0.x, p0.y, start_hdg, start_k, p3.x, p3.y, end_hdg, end_k);
                            p0 = cubicBezier[0];
                            p1 = cubicBezier[1];
                            p2 = cubicBezier[2];
                            p3 = cubicBezier[3];

                            bezierCurve.clear();
                            for (size_t i = 0u; i <= 100; ++i)
                            {
                                bezierCurve.push_back(
                                    convert(cubicBezier.valueAt(static_cast<float64_t>(i) / 100.)));
                            }

                            bezierBbox.clear();
                            
                            bezier::TightBoundingBox bbox = cubicBezier.tbb();
                            {
                                bezierBbox.push_back(convert(bbox[0]));
                                bezierBbox.push_back(convert(bbox[1]));
                                bezierBbox.push_back(convert(bbox[2]));
                                bezierBbox.push_back(convert(bbox[3]));
                                bezierBbox.push_back(convert(bbox[0]));
                            }

                            k0 = cubicBezier.curvature(0.0);
                            k1 = cubicBezier.curvature(1.0);
                            ke = cubicBezier.maxCurvatureNumeric();
                        }


                        ImGui::Text("start curvature: %f", k0);
                        ImGui::Text("end curvature: %f", k1);
                        ImGui::Text("max curvature (numeric): %f", ke);

                        m_renderer.drawPolyline(bezierCurve,
                                                aidrive::Vector3f{0.0f, 0.0f, 0.0f},
                                                aidrive::render::COLOR_GREEN,
                                                true);
                        // m_renderer.drawPolyline(bezierBbox,
                        //                         aidrive::Vector3f{0.0f, 0.0f, 0.0f},
                        //                         aidrive::render::COLOR_BLACK,
                        //                         false);
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