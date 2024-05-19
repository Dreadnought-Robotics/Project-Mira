#include <iostream>
#include <thread>
#include <map>
#include <signal.h>
#include <sys/wait.h>
#include <unistd.h>
#include "ros/ros.h"
#include "std_msgs/Bool.h"

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

bool aruco_detected = false;
bool emergency = false;

void arucoCallback(const std_msgs::Bool::ConstPtr& msg) {
    aruco_detected = msg->data;
}

void emergencyCallback(const std_msgs::Bool::ConstPtr& msg) {
    emergency = msg->data;
}

void killChildren(const std::map<std::string, BehaviorTreeNode*>& launchNodeMap) {
    for (const auto& pair : launchNodeMap) {
        pair.second->terminate();
    }
}

void handleSignal(int signal) {
    if (signal == SIGINT) {
        std::cout << "Exit code, killing all child processes" << std::endl;
        ros::shutdown();
        exit(0);
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "behavior_tree_control");
    ros::NodeHandle nh;

    ros::Subscriber sub1 = nh.subscribe("aruco_topic", 10, arucoCallback);
    ros::Subscriber sub2 = nh.subscribe("emergency_topic", 10, emergencyCallback);

    ActionNode runEmergencyDependency("runEmergencyDependency", "mira_pkg_handler", "runEmergencyDependency.launch");
    ActionNode runPerception("runPerception", "mira_perception", "perception.launch");
    ActionNode runDocking("runDocking", "mira_docking", "dock.launch");
    ActionNode runTeleop("runTeleop", "mira_rov", "teleop.launch");
    ActionNode runController("runController", "mira_controller", "controller.launch");

    std::map<std::string, BehaviorTreeNode*> launchNodeMap = {
        {"runEmergencyDependency", &runEmergencyDependency},
        {"runPerception", &runPerception},
        {"runDocking", &runDocking},
        {"runTeleop", &runTeleop},
        {"runController", &runController}
    };

    runEmergencyDependency.execute(); 
    runPerception.execute();
    runDocking.execute();
    runTeleop.execute();
    runController.execute();

    signal(SIGINT, handleSignal);

    while (ros::ok()) {
        int input = 0;
        std::cout << "Enter command (1-5 to terminate specific node, 0 to terminate runEmergencyDependency, -1 to terminate all and exit): ";
        std::cin >> input;
        switch (input) {
            case 1: {
                if (runPerception.getPID() != -1) {
                    std::cout << "Terminating runPerception node with PID: " << runPerception.getPID() << std::endl;
                    runPerception.terminate();
                }
                break;
            }
            case 2: {
                if (runDocking.getPID() != -1) {
                    std::cout << "Terminating runDocking node with PID: " << runDocking.getPID() << std::endl;
                    runDocking.terminate();
                }
                break;
            }
            case 3: {
                if (runTeleop.getPID() != -1) {
                    std::cout << "Terminating runTeleop node with PID: " << runTeleop.getPID() << std::endl;
                    runTeleop.terminate();
                }
                break;
            }
            case 4: {
                if (runController.getPID() != -1) {
                    std::cout << "Terminating runController node with PID: " << runController.getPID() << std::endl;
                    runController.terminate();
                }
                break;
            }
            case 0: {
                if (runEmergencyDependency.getPID() != -1) {
                    std::cout << "Terminating runEmergencyDependency node with PID: " << runEmergencyDependency.getPID() << std::endl;
                    runEmergencyDependency.terminate();
                }
                break;
            }
            case -1: { // Terminate all and exit
                killChildren(launchNodeMap);
                return 0;
            }
            default: 
                std::cout << "Invalid command!" << std::endl;
                break;
        }
    }

    return 0;
}
