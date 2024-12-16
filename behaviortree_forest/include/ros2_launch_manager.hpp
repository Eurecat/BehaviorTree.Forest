#pragma once

#include <unistd.h>
#include <signal.h>
#include <sys/types.h>
#include <sys/wait.h> 

#include <cstdint>
#include <memory>
#include <thread>
#include <mutex>
#include <vector>
#include <algorithm>
#include <numeric>
#include <cstring>

class ROS2LaunchManager 
{
public:
    ROS2LaunchManager()
    {
        std::atomic_init(&m_running, true);
        m_thread = std::thread(&ROS2LaunchManager::wait, this);
    }

    ~ROS2LaunchManager() 
    {
        if (m_running.load()) {
            m_running.store(false);

            if (m_thread.joinable()) {
                m_thread.join();
            }
        }
    }

    pid_t extract_bt_node_pid_from_python_pid(pid_t python_pid) 
    {
        pid_t bt_node_pid = 0;
        std::string cmd = "ps --ppid " + std::to_string(python_pid) + " -o pid,comm";
        FILE* fp = popen(cmd.c_str(), "r");
        //std::cout << "FROM Python PID: " << std::to_string(python_pid) << " Extracting BT_Node PID with cmd: " << cmd << std::endl;
        if (fp == nullptr) {
            std::cout << "Failed to run command: " << cmd << std::endl;
        }
        else
        {
            char buffer[128];
            std::string ps_output;
            while (fgets(buffer, sizeof(buffer), fp)) {
                ps_output += buffer;  // Append each line of output
            }
            fclose(fp);
            //std::cout << "RAW OUTPUT: " << ps_output << std::endl;

            // Split ps_output into lines
            size_t pos = 0;
            while ((pos = ps_output.find('\n')) != std::string::npos) {
                std::string line = ps_output.substr(0, pos);
                if (line.find("behaviortree") != std::string::npos) {
                    // Extract PID from the line that contains 'behaviortree_node'
                    size_t startPos = line.find_first_not_of(" \t"); // Find the first non-space character
                    if (startPos != std::string::npos) {
                        line = line.substr(startPos); // Trim leading spaces
                    }
                    std::string pid_str = line.substr(0, line.find(' ')); // The PID is the first element in the line
                    sscanf(pid_str.c_str(), "%d", &bt_node_pid);  // Parse the PID
                    //std::cout << "Node process PID extracted: " << bt_node_pid << std::endl;

                    // Option2: Extract the number using stoi
                    /*try {
                        int number = std::stoi(line); // Convert the number from the string
                        std::cout << "Extracted number: " << number << std::endl;
                    } catch (const std::invalid_argument& e) {
                        std::cout << "No valid number found." << std::endl;
                    } catch (const std::out_of_range& e) {
                        std::cout << "Number is out of range." << std::endl;
                    }*/
                    break;
                }
                ps_output.erase(0, pos + 1);
            }
        }
        return bt_node_pid;
    }

    template<typename... Args>
    pid_t start(Args... args) 
    {
        std::vector<std::string> args_vector = { args... };

        if (args_vector.size() > 0) {
            pid_t pid = ::fork();
           // int r = prctl(PR_SET_PDEATHSIG, SIGTERM);
            if (pid == 0) {
                //std::cout << "BT_SERVER: PID == 0" << std::endl;
                ::setsid();
                
                ::signal(SIGINT, SIG_IGN);
                
                /*::fclose(stdout);
                ::fclose(stdin);
                ::fclose(stderr);*/

                ::execlp("ros2", "ros2", "run", args..., nullptr);
            }
            else {
                //std::cout << "BT_SERVER: PID != 0" << std::endl;
                std::scoped_lock<std::mutex> scoped_lock(m_mutex);

                std::string args_string = std::accumulate(std::next(std::begin(args_vector)), std::end(args_vector), args_vector[0], [](std::string lhs, std::string rhs) -> std::string { return lhs + " " + rhs; });

                std::cout << "BT_SERVER: Starting \"ros2" << args_string << "\" with PID" << pid << std::endl;
                m_pids.push_back(pid);
            }

            return pid;
        }
        else {
            throw std::runtime_error("ROS2LaunchManager::start - No arguments provided");
        }
    }

    void stop(pid_t const &pid, int32_t const &signal) 
    {
        std::scoped_lock<std::mutex> scoped_lock(m_mutex);

        auto pid_it = std::find(std::begin(m_pids), std::end(m_pids), pid);

        if (pid_it != m_pids.end()) {
            ::kill(pid, signal);
            std::cout << "BT_SERVER: Stopping process with PID " << pid << "and signal " << signal << std::endl;
        }
        else {
            throw std::runtime_error("ROS2LaunchManager::stop - PID " + std::to_string(pid) + " not found");
        }
    }

private:
    void wait()
    {
        while (m_running.load()) {
            std::scoped_lock<std::mutex> scoped_lock(m_mutex);

            for (auto pid_it = std::begin(m_pids); pid_it != std::end(m_pids); ++pid_it) {
                pid_t const pid = *pid_it;

                int32_t status;

                if (::waitpid(pid, &status, WUNTRACED | WCONTINUED | WNOHANG) == pid) {
                    if (WIFEXITED(status)) {
                      //  RCLCPP_INFO(nh->get_logger(),"PID %d exited with status %d", pid, WEXITSTATUS(status));

                        pid_it = m_pids.erase(pid_it);

                        if (pid_it == std::end(m_pids)) {
                            break;
                        }
                    } 
                    else if (WIFSIGNALED(status)) {
                     //   RCLCPP_INFO(nh->get_logger(),"PID %d killed with signal %d", pid, WTERMSIG(status));

                        pid_it = m_pids.erase(pid_it);

                        if (pid_it == std::end(m_pids)) {
                            break;
                        }
                    } 
                    else if (WIFSTOPPED(status)) {
                      //  RCLCPP_INFO(nh->get_logger(),"PID %d stopped with signal %d", pid, WSTOPSIG(status));
                    } 
                    else if (WIFCONTINUED(status)) {
                      //  RCLCPP_INFO(nh->get_logger(),"PID %d continued"   , pid);
                    }
                }
            }
        }

        std::scoped_lock<std::mutex> scoped_lock(m_mutex);

        for (pid_t const &pid : m_pids) {
            ::kill(pid, SIGINT);

            int32_t status;

            ::waitpid(pid, &status, 0);
        }
    }

    std::vector<pid_t> m_pids;
    std::atomic<bool> m_running;
    std::thread m_thread;
    std::mutex m_mutex;
};