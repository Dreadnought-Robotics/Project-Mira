#include <iostream>
#include <thread>
#include <map>
#include <signal.h>
#include <sys/wait.h>
#include <unistd.h>
#include "ros/ros.h"
#include "std_msgs/Bool.h"

// Behavior Tree Node Types
class BehaviorTreeNode {
public:
    virtual ~BehaviorTreeNode() {}
    virtual void execute() = 0;
    virtual void terminate() = 0;
    virtual pid_t getPID() const { return -1; }
};

class ActionNode : public BehaviorTreeNode {
private:
    std::string nodeName;
    std::string launchFile;
    std::string launchPackage;
    pid_t pid;

public:
    ActionNode(const std::string& name, const std::string& pkg, const std::string& file) : nodeName(name), launchPackage(pkg), launchFile(file), pid(-1) {
        std::cout << "ActionNode created: " << nodeName << std::endl;
    }

    void execute() override {
        if (pid == -1) {
            pid = fork();
            if (pid == 0) {
                execlp("roslaunch", "roslaunch", launchPackage.c_str(), launchFile.c_str(), nullptr);
                _exit(1);  // If exec fails, exit child immediately
            }
            std::cout << "PID set for " << nodeName << ": " << pid << std::endl;
        }
    }

    void terminate() override {
        if (pid > 0) {
            std::cout << "Terminating " << nodeName << " node with PID: " << pid << std::endl;
            kill(pid, SIGINT);
            waitpid(pid, nullptr, 0);
            pid = -1;
        }
    }

    pid_t getPID() const override {
        return pid;
    }
};

// Global variables for ROS callbacks
bool aruco_detected = false;
bool emergency = false;

// ROS callback functions
void arucoCallback(const std_msgs::Bool::ConstPtr& msg) {
    aruco_detected = msg->data;
}

void emergencyCallback(const std_msgs::Bool::ConstPtr& msg) {
    emergency = msg->data;
}

// Global variable for child PIDs
std::vector<pid_t> child_pids;

// Function to kill all child processes
void killChildren() {
    for (pid_t pid : child_pids) {
        kill(pid, SIGKILL);
        std::cout << "Child process: " << pid << " killed" << std::endl;
    }
}

// Signal handler function
void handleSignal(int signal) {
    if (signal == SIGINT) {
        std::cout << "Exit code, killing all child processes" << std::endl;
        killChildren();
        std::cout << "Parent process exiting..." << std::endl;
        exit(0);
    }
}

// Main function
int main(int argc, char **argv) {
    // Initialize ROS
    ros::init(argc, argv, "behavior_tree_control");
    ros::NodeHandle nh;

    // ROS Subscribers
    ros::Subscriber sub2 = nh.subscribe("emergency_topic", 10, emergencyCallback);

    // Create the behavior tree nodes
    ActionNode* runEmergencyDependency = new ActionNode("runEmergencyDependency", "mira_pkg_handler", "runEmergencyDependency.launch");

    std::map<std::string, BehaviorTreeNode*> launchNodeMap;

    // Store launch nodes in a map
    launchNodeMap["runEmergencyDependency"] = runEmergencyDependency;
    runEmergencyDependency->execute(); 

    // std::thread execution_thread([&]() {
    //     //while (ros::ok()) {
    //         if (emergency) {
    //             disarm->execute();
    //             runEmergencyDependency->terminate();
    //         } else if (aruco_detected) {
    //             runEmergencyDependency->execute();
    //             runDocking->execute();
    //             runController->execute();
    //         } //else {
    //             //disarm->execute();
    //             //rov->execute();
    //             //break;
    //         //}
    //         // Sleep for a while to prevent high CPU usage
    //         std::this_thread::sleep_for(std::chrono::milliseconds(100));
    //     //}
    // });

//     // Input handling thread
//     std::thread input_thread([&]() {
//     int input = 0;
//     while (ros::ok()) {
//         std::cout << "Enter command (1-4 to terminate specific node, 0 to terminate runEmergencyDependency, -1 to terminate all and exit): ";
//         std::cin >> input;
//         switch (input) {
//             case 1: {
//                 auto it = launchNodeMap.find("rov");
//                 if (it != launchNodeMap.end()) {
//                     BehaviorTreeNode* node = it->second;
//                     if (node->getPID() != -1) {
//                         std::cout << "Terminating rov node with PID: " << node->getPID() << std::endl;
//                         node->terminate();
//                     }
//                 }
//                 break;
//             }
//             case 2: {
//                 auto it = launchNodeMap.find("runDocking");
//                 if (it != launchNodeMap.end()) {
//                     BehaviorTreeNode* node = it->second;
//                     if (node->getPID() != -1) {
//                         std::cout << "Terminating runDocking node with PID: " << node->getPID() << std::endl;
//                         node->terminate();
//                     }
//                 }
//                 break;
//             }
//             case 3: {
//                 auto it = launchNodeMap.find("runController");
//                 if (it != launchNodeMap.end()) {
//                     BehaviorTreeNode* node = it->second;
//                     if (node->getPID() != -1) {
//                         std::cout << "Terminating runController node with PID: " << node->getPID() << std::endl;
//                         node->terminate();
//                     }
//                 }
//                 break;
//             }
//             case 4: {
//                 auto it = launchNodeMap.find("disarm");
//                 if (it != launchNodeMap.end()) {
//                     BehaviorTreeNode* node = it->second;
//                     if (node->getPID() != -1) {
//                         std::cout << "Terminating disarm node with PID: " << node->getPID() << std::endl;
//                         node->terminate();
//                     }
//                 }
//                 break;
//             }
//             case 0: {
//                 auto it = launchNodeMap.find("runEmergencyDependency");
//                 if (it != launchNodeMap.end()) {
//                     BehaviorTreeNode* node = it->second;
//                     if (node->getPID() != -1) {
//                         std::cout << "Terminating runEmergencyDependency node with PID: " << node->getPID() << std::endl;
//                         node->terminate();
//                     }
//                 }
//                 break;
//             }
//             case -1: { // Terminate all and exit
//                 for (auto& entry : launchNodeMap) {
//                     entry.second->terminate();
//                 }
//                 ros::shutdown();
//                 return;
//             }
//             default: 
//                 std::cout << "Invalid command!" << std::endl;
//                 break;
//         }
//     }
// });


    // // Signal handling
    // signal(SIGINT, handleSignal);

    // // Spin until ROS is shutdown
    // ros::spin();

    // // Cleanup
    // execution_thread.join();
    // input_thread.join();

    // //delete runPyMavlink;
    // //delete runImagePipeline;
    // delete runEmergencyDependency;
    return 0;
}






