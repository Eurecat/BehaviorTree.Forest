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

#include "rclcpp/rclcpp.hpp"

class ROS2LaunchManager
{
public:
    ROS2LaunchManager(const std::shared_ptr<rclcpp::Node> node) :
        node_ptr_(node)
    {
        std::atomic_init(&m_running, true);
        // m_thread = std::thread(&ROS2LaunchManager::wait, this); // leave it for debugging purpose, otherwise it serves no real purpose
    }

    ~ROS2LaunchManager()
    {
        if (m_running.load())
        {
            m_running.store(false);

            if (m_thread.joinable())
            {
                m_thread.join();
            }
        }
    }

    pid_t extract_bt_node_pid_from_python_pid(pid_t python_pid)
    {
        pid_t bt_node_pid = 0;
        std::string cmd = "ps --ppid " + std::to_string(python_pid) + " -o pid,comm";
        FILE *fp = popen(cmd.c_str(), "r");
        RCLCPP_DEBUG(node_ptr_->get_logger(), "[ROS2LaunchManager] From Python PID: %d Extracting BT_Node PID with cmd: %s", python_pid, cmd.c_str());
        if (fp == nullptr)
        {
            RCLCPP_ERROR(node_ptr_->get_logger(), "[ROS2LaunchManager] Failed to run command: %s", cmd.c_str());
        }
        else
        {
            char buffer[128];
            std::string ps_output;
            while (fgets(buffer, sizeof(buffer), fp))
            {
                ps_output += buffer; // Append each line of output
            }
            fclose(fp);

            // Split ps_output into lines
            size_t pos = 0;
            while ((pos = ps_output.find('\n')) != std::string::npos)
            {
                std::string line = ps_output.substr(0, pos);
                if (line.find("behaviortree") != std::string::npos)
                {
                    // Extract PID from the line that contains 'behaviortree_node'
                    size_t startPos = line.find_first_not_of(" \t"); // Find the first non-space character
                    if (startPos != std::string::npos)
                    {
                        line = line.substr(startPos); // Trim leading spaces
                    }
                    std::string pid_str = line.substr(0, line.find(' ')); // The PID is the first element in the line
                    sscanf(pid_str.c_str(), "%d", &bt_node_pid);          // Parse the PID

                    // Option2: Extract the number using stoi
                    /*try {
                        int number = std::stoi(line); // Convert the number from the string
                    } catch (const std::invalid_argument& e) {
                    } catch (const std::out_of_range& e) {
                    }*/
                    break;
                }
                ps_output.erase(0, pos + 1);
            }
        }
        return bt_node_pid;
    }

    template <typename... Args>
    pid_t start(Args... args)
    {
        {
            std::vector<std::string> args_vector = {args...};

            if (args_vector.size() > 0)
            {
                pid_t pid = ::fork();
                // int r = prctl(PR_SET_PDEATHSIG, SIGTERM);
                if (pid == 0)
                {
                    ::setsid();

                    ::signal(SIGINT, SIG_IGN);

                    /*::fclose(stdout);
                    ::fclose(stdin);
                    ::fclose(stderr);*/

                    ::execlp("ros2", "ros2", "run", args..., nullptr);
                }
                else
                {
                    std::scoped_lock<std::mutex> scoped_lock(m_mutex);

                    std::string args_string = std::accumulate(std::next(std::begin(args_vector)), std::end(args_vector), args_vector[0], [](std::string lhs, std::string rhs) -> std::string
                                                              { return lhs + " " + rhs; });
                    
                    RCLCPP_INFO(node_ptr_->get_logger(), "[ROS2LaunchManager] Starting \"ros2 %s\" with PID: %d", args_string.c_str(), pid);

                    m_pids.push_back(pid);
                }

                return pid;
            }
            else
            {
                throw std::runtime_error("ROS2LaunchManager::start - No arguments provided");
            }
        }

        update_pids_status();
    }

    void stop(pid_t const &pid, int32_t const &signal)
    {
        {
            std::scoped_lock<std::mutex> scoped_lock(m_mutex);

            auto pid_it = std::find(std::begin(m_pids), std::end(m_pids), pid);

            if (pid_it != m_pids.end())
            {
                ::kill(pid, signal);
                
                RCLCPP_ERROR(node_ptr_->get_logger(), "[ROS2LaunchManager] Stopping process with PID: %d and signal %d", pid, signal);
            }
            else
            {
                throw std::runtime_error("ROS2LaunchManager::stop - PID " + std::to_string(pid) + " not found");
            }
        }

        update_pids_status();
    }

private:
    void update_pids_status()
    {
        std::scoped_lock<std::mutex> scoped_lock(m_mutex);

        for (auto pid_it = std::begin(m_pids); pid_it != std::end(m_pids); ++pid_it)
        {
            pid_t const pid = *pid_it;

            int32_t status;
            if (::waitpid(pid, &status, WUNTRACED | WCONTINUED | WNOHANG) == pid)
            {
                if (WIFEXITED(status))
                {
                    //  RCLCPP_INFO(nh->get_logger(),"PID %d exited with status %d", pid, WEXITSTATUS(status));

                    pid_it = m_pids.erase(pid_it);

                    if (pid_it == std::end(m_pids))
                    {
                        break;
                    }
                }
                else if (WIFSIGNALED(status))
                {
                    //   RCLCPP_INFO(nh->get_logger(),"PID %d killed with signal %d", pid, WTERMSIG(status));

                    pid_it = m_pids.erase(pid_it);

                    if (pid_it == std::end(m_pids))
                    {
                        break;
                    }
                }
                else if (WIFSTOPPED(status))
                {
                    //  RCLCPP_INFO(nh->get_logger(),"PID %d stopped with signal %d", pid, WSTOPSIG(status));
                }
                else if (WIFCONTINUED(status))
                {
                    //  RCLCPP_INFO(nh->get_logger(),"PID %d continued"   , pid);
                }
            }
        }
    }

    /*
        Wait thread for debugging purpose, otherwise it serves no function. That is why for the moment we don't spawn it in the constructor

        Checks regularly active pids and update its array.
        Make sure to kill everybody when destructor of the class is called
    */
    void wait()
    {
        while (m_running.load())
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(750));
            update_pids_status();
        }

        std::scoped_lock<std::mutex> scoped_lock(m_mutex);

        for (pid_t const &pid : m_pids)
        {
            ::kill(pid, SIGINT);

            int32_t status;

            ::waitpid(pid, &status, 0);
        }
    }

    std::vector<pid_t> m_pids;
    std::atomic<bool> m_running;
    std::thread m_thread;
    std::mutex m_mutex;

    std::shared_ptr<rclcpp::Node> node_ptr_;
};