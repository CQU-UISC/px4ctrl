#include <GLFW/glfw3.h>
#include <csignal>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <memory>
#include <spdlog/logger.h>
#include <spdlog/spdlog.h>
#include <thread>
#include <vector>
#include <iostream>

#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"

#include "px4ctrl_gcs.h"
#include "px4ctrl_gcs_client.h"

#define GL_SILENCE_DEPRECATION
#include <GLFW/glfw3.h> // Will drag system OpenGL headers

#ifdef __ROS_IMPL__
#include "px4ctrl_gcs_ros.h"
#include "ros/init.h"
#endif

#ifdef __ZMQ_IMPL__
#include <zmq.hpp>
#include <zmq_addon.hpp>
#include "px4ctrl_gcs_zmq.h"
#endif

namespace px4ctrl{
namespace gcs{

    #ifdef __ZMQ_IMPL__
    using json=nlohmann::json;
        ZmqProxy::ZmqProxy(std::string base_dir):ctx(2),
        xsub_socket(ctx,zmq::socket_type::xsub),
        xpub_socket(ctx,zmq::socket_type::xpub){
            std::ifstream config_file(base_dir + "/px4ctrl_zmq.json");
            json json_file;
            if (!config_file.is_open()) {
                spdlog::error("Failed to open config file");
                exit(1);
            }
            config_file >> json_file;
            spdlog::info("Config file loaded");
            config_file.close();

            if (!(json_file.contains("xpub_endpoint_bind") && 
                json_file.contains("xsub_endpoint_bind")
                )) {
                spdlog::error("Config file missing key");
                exit(1);
            }
            xpub_endpoint = json_file["xpub_endpoint_bind"].template get<std::string>();
            xsub_endpoint = json_file["xsub_endpoint_bind"].template get<std::string>();

            // Init XSUB socket
            try
            {
                xsub_socket.bind(xsub_endpoint);
                xpub_socket.bind(xpub_endpoint);
            }
            catch (zmq::error_t e)
            {
                spdlog::error("unable bind xsub/xpub,check your config file, err:{}",e.what());
                exit(1);
            }
            proxy_thread = std::thread(&ZmqProxy::run,this);
        }

        ZmqProxy::~ZmqProxy(){
            ctx.shutdown();
            ctx.close();
            proxy_thread.join();
        }

        void ZmqProxy::run(){
            zmq::proxy(xsub_socket, xpub_socket);
            return;
        }
    #endif


    GcsClient::GcsClient(std::shared_ptr<GcsCom> gcs_com):gcs(gcs_com){
    }

    void GcsClient::render_window(){
        handle_message();

        const std::vector<command::DRONE_COMMAND> cmd_list = {
            command::ENTER_OFFBOARD,
            command::EXIT_OFFBOARD,
            command::ARM,
            command::DISARM,
            command::TAKEOFF,
            command::LAND,
            command::ALLOW_CMD_CTRL,
            command::FORCE_HOVER
        };

        for(auto & el :cb_queue){
            while(el.second.size()>10){
                spdlog::info("drone:{}, cb_queue size: {}>10, pop",el.first,el.second.size());
                el.second.pop();
            }
            if(!el.second.empty()){
                drone_list[el.first].first = el.second.front();
                drone_list[el.first].second = clock::now();
                el.second.pop();
            }
        }

        ImGuiIO& io = ImGui::GetIO(); (void)io;
        ImGui::Begin("px4ctrl_gcs_client", NULL , ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize|ImGuiWindowFlags_NoTitleBar|ImGuiWindowFlags_NoCollapse|ImGuiWindowFlags_NoBackground);                          // Create a window called "Hello, world!" and append into it.
        for(const auto& elem:drone_list){
            const Drone&  el = elem.second.first;
            const clock::time_point& arrive_time = elem.second.second;
            ImGui::Text("drone_id: %d",el.drone_id);
            ImGui::Text("px4ctrl_status: %s %s %s",Px4CtrlStateName[el.px4ctrl_status[0]],Px4CtrlStateName[el.px4ctrl_status[1]],Px4CtrlStateName[el.px4ctrl_status[2]]);
            ImGui::Text("pos: %.2f %.2f %.2f",el.pos[0],el.pos[1],el.pos[2]);
            ImGui::Text("vel: %.2f %.2f %.2f",el.vel[0],el.vel[1],el.vel[2]);
            ImGui::Text("acc: %.2f %.2f %.2f",el.acc[0],el.acc[1],el.acc[2]);
            ImGui::Text("omega: %.2f %.2f %.2f",el.omega[0],el.omega[1],el.omega[2]);
            ImGui::Text("quat: %.2f %.2f %.2f %.2f",el.quat[0],el.quat[1],el.quat[2],el.quat[3]);
            ImGui::Text("battery: %.2f",el.battery);
            ImGui::Text("Last update: %.2f s",std::chrono::duration_cast<std::chrono::duration<double>>(clock::now()-arrive_time).count());
            for(const auto &cmd : cmd_list){
                if(ImGui::Button(command::DRONE_COMMAND_NAME[cmd])){
                    gcs_queue.push(
                        Gcs{id++,0,0,el.drone_id,(uint8_t)cmd,0}
                    );
                }
            }
        }
        if(drone_list.empty()){
            ImGui::Text("No drone");
        }
        ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / io.Framerate, io.Framerate);
        ImGui::End();
    }

    void GcsClient::handle_message(){
        if(clock::now()-last_recv_time>std::chrono::milliseconds(100)){
            //recv
            Drone drone_message;
            while(gcs->receive(drone_message)){
                cb_queue[drone_message.drone_id].push(drone_message);
                spdlog::info("recv drone:{}",drone_message.drone_id);
            }
            //send
            while(!gcs_queue.empty()){
                gcs->send(gcs_queue.front());
                gcs_queue.pop();
            }
            spdlog::info("send gcs heartbeat");
            gcs->send(Gcs{id++,0,0,ALL_DRONES,command::EMPTY,0});//Heartbeat
            last_recv_time = clock::now();
        }
    }
}
}

static void glfw_error_callback(int error, const char* description)
{
    spdlog::error("GLFW Error {}: {}\n", error, description);
}


void sigintHandler( int sig ) {
    spdlog::info( "[px4ctrl_gcs] exit..." );
    #ifdef __ROS_IMPL__
    ros::shutdown();
    #endif
    exit(0);
}

int main(int argc, char* argv[]){
    // 检查参数数量
    if (argc != 3) {
        std::cerr << "Usage: " << argv[0] << " -c <config_dir>" << std::endl;
        return 1;
    }

    std::string flag = argv[1];
    std::string config_dir;

    // 检查参数是否为 -c
    if (flag == "-c") {
        config_dir = argv[2];
        std::cout << "Config directory: " << config_dir << std::endl;
    } else {
        std::cerr << "Invalid argument: " << flag << std::endl;
        std::cerr << "Usage: " << argv[0] << " -c <config_dir>" << std::endl;
        return 1;
    }

    signal( SIGINT, sigintHandler );
    //set up
    glfwSetErrorCallback(glfw_error_callback);
    if (!glfwInit())
        return 1;

    // GL 3.3 + GLSL 330
    const char* glsl_version = "#version 330";
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    #ifdef __APPLE__
    // GL 3.2 + GLSL 150
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);  // 3.2+ only
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);            // Required on Mac
    #endif
    // glfwWindowHint(GLFW_DECORATED, 0);
    // glfwWindowHint(GLFW_TRANSPARENT_FRAMEBUFFER, GLFW_TRUE);
    //glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);  // 3.2+ only
    //glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);            // 3.0+ only

    // Create window with graphics context
    GLFWwindow* window = glfwCreateWindow(400, 600, "px4ctrl GCS client", nullptr, nullptr);
    if (window == nullptr)
        return 1;
    glfwMakeContextCurrent(window);
    glfwSwapInterval(0); 

    // Setup Dear ImGui context
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO(); (void)io;
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;     // Enable Keyboard Controls
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad;      // Enable Gamepad Controls
    // Setup Dear ImGui style
    ImGui::StyleColorsDark();
    //ImGui::StyleColorsLight();

    // Setup Platform/Renderer backends
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init(glsl_version);

   
    #ifdef __ROS_IMPL__
    //use ros impl
    //init ros node
    ros::init(argc, argv, "px4ctrl_gcs_client");
    ros::NodeHandle nh;
    std::shared_ptr<px4ctrl::gcs::GcsCom> gcs_com = std::make_shared<px4ctrl::gcs::rosimpl::GcsRosCom>(nh);
    
    #endif

    #ifdef __ZMQ_IMPL__
    px4ctrl::gcs::ZmqProxy proxy(config_dir);
    std::shared_ptr<px4ctrl::gcs::GcsCom> gcs_com = std::make_shared<px4ctrl::gcs::zmqimpl::GcsZmqCom>(config_dir);
    #endif

    //gcs client impl
    px4ctrl::gcs::GcsClient gcs_client(gcs_com);

    //clear_color = Imgui background color
    ImVec4 clear_color = ImGui::GetStyleColorVec4(ImGuiCol_WindowBg);
    //main loop
    while (!glfwWindowShouldClose(window))
    {
        // Poll and handle events (inputs, window resize, etc.)
        // You can read the io.WantCaptureMouse, io.WantCaptureKeyboard flags to tell if dear imgui wants to use your inputs.
        // - When io.WantCaptureMouse is true, do not dispatch mouse input data to your main application, or clear/overwrite your copy of the mouse data.
        // - When io.WantCaptureKeyboard is true, do not dispatch keyboard input data to your main application, or clear/overwrite your copy of the keyboard data.
        // Generally you may always pass all inputs to dear imgui, and hide them from your application based on those two flags.
        glfwPollEvents();

        // Start the Dear ImGui frame
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();
        int width, height;
        glfwGetWindowSize(window, &width, &height);
        ImGui::SetNextWindowSize(ImVec2(width, height)); // ensures ImGui fits the GLFW window
        ImGui::SetNextWindowPos(ImVec2(0, 0));

        //render window
        gcs_client.render_window(); 
       
        // Rendering
        ImGui::Render();
        int display_w, display_h;
        glfwGetFramebufferSize(window, &display_w, &display_h);
        glViewport(0, 0, display_w, display_h);
        glClearColor(clear_color.x * clear_color.w, clear_color.y * clear_color.w, clear_color.z * clear_color.w, clear_color.w);
        glClear(GL_COLOR_BUFFER_BIT);
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        glfwSwapBuffers(window);
    }
    // Cleanup
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();

    glfwDestroyWindow(window);
    glfwTerminate();
    return  0;
}