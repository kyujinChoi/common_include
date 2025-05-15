#ifndef _DIAGNOSTIC_HPP
#define _DIAGNOSTIC_HPP

#include "rclcpp/rclcpp.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "rcl_interfaces/msg/log.hpp"
#include "parameter/parameter.h"
#include <regex>


#pragma once

using namespace std::chrono_literals;

enum
{
    DEBUG = 0,  // Debug is for pedantic information, which is useful when debugging issues.
    INFO = 1,   // Info is the standard informational level and is used to report expected
    WARN = 2,   // Warning is for information that may potentially cause issues or possibly unexpected
    ERROR = 3,  // Error is for information that this node cannot resolve.
    FATAL = 4,  // Information about a impending node shutdown.
};


class StatusReporter
{
    typedef struct DiagInfo
    {
        diagnostic_msgs::msg::DiagnosticStatus diag_status;
        std::mutex mtx;
    } diag_info_t;
    typedef struct TopicPeriodInfo
    {
        int warn_cnt;
        int warn_max_cnt;
        double warn_timeout;  // in seconds
        double error_timeout; // in seconds
        rclcpp::Time received_time;
        uint8_t check_status;
        std::mutex mtx;

        double frame_time_diff;
        bool initialFlag;

    } topic_period_info_t;


    enum
    {
        OK = 0,
        WARN = 1,
        ERROR = 2,
        STALE = 3
    };
    
public:
    StatusReporter(rclcpp::Node *node ) : node_(node), clock_(std::make_shared<rclcpp::Clock>(RCL_ROS_TIME))
    {
        

        std::string node_name = node->get_fully_qualified_name(); 
        
        std::regex pattern("/P\\d+");  
        std::string check_topic = std::regex_replace(node_name, pattern, "");

        param.insertParam("node_name", node_name);
        param.insertParam("check_topic", "check"+check_topic);
        initPubs();

        
    }
    ~StatusReporter() {}

    void pubLogMessage(uint8_t level, std::string msg)
    {
        log_msg.name = param.getParamStr("node_name");
        log_msg.level = level;
        log_msg.msg = msg;
        log_msg.stamp = node_->get_clock()->now();
        pubLogMsg->publish(log_msg);

        return;
    }

    void createTopicPeriodInfo(std::string check_key,std::string hw_id, double warn_timeout, int warn_max_cnt, double error_timeout)
    {   

        if (hasDiagKey(check_key))
        {
            std::cout << "Already exist in diag_key: " << check_key << std::endl;
            return;
        }

        std::shared_ptr<topic_period_info_t> topic_period_info = std::make_shared<topic_period_info_t>();
        topic_period_info->warn_cnt = 0;
        topic_period_info->warn_max_cnt = warn_max_cnt;
        topic_period_info->warn_timeout = warn_timeout;
        topic_period_info->error_timeout = error_timeout;
        topic_period_info->received_time = rclcpp::Time(0,0,RCL_ROS_TIME);
        topic_period_info->check_status = OK;
        topic_period_info->initialFlag = false;

        topic_period_info_map[check_key] = topic_period_info;
        

        
        createDiagInfo(check_key);
        setDiagHwId(check_key, hw_id);
        return;
    }

    void updateTopicPeriod(std::string check_key)
    {

        rclcpp::Time cur_update_time = clock_->now();

        if (topic_period_info_map[check_key]->received_time == rclcpp::Time(0, 0, RCL_ROS_TIME)) {
            topic_period_info_map[check_key]->received_time = cur_update_time;
            return;  
        }

        {
            std::lock_guard<std::mutex> lock(topic_period_info_map[check_key]->mtx);
            topic_period_info_map[check_key]->frame_time_diff = (cur_update_time - topic_period_info_map[check_key]->received_time).seconds();
            topic_period_info_map[check_key]->received_time = cur_update_time ;
            
        }

        if (topic_period_info_map[check_key]->initialFlag == false){
            pubLogMessage(::INFO,check_key+" topic monitoring start!");
            topic_period_info_map[check_key]->initialFlag = true;
        }
        // std::cout<<"frame_time_diff : "<<topic_period_info_map[check_key]->frame_time_diff<<std::endl;
        // std::cout << "FrameRate : " << 1 / topic_period_info_map[check_key]->frame_time_diff << " Hz" << std::endl;

        return;
    }

    void createDiagInfo(std::string diag_key)
    {
        if (hasDiagKey(diag_key))
            return;

        std::shared_ptr<diag_info_t> diag_info = std::make_shared<diag_info_t>();
        diag_info->diag_status.name = param.getParamStr("node_name");
        diag_info->diag_status.level = OK;
        diag_info->diag_status.message = "Initialized";

        diag_info_map[diag_key] = diag_info;

        return;
    }

    void setDiagName(std::string diag_key, std::string name) // checker이름
    {
        std::lock_guard<std::mutex> lock(diag_info_map[diag_key]->mtx);
        diag_info_map[diag_key]->diag_status.name = name;
        return;
    }

    void setDiagHwId(std::string diag_key, std::string hw_id) // 문제 주체 HW이름
    {
        std::lock_guard<std::mutex> lock(diag_info_map[diag_key]->mtx);
        diag_info_map[diag_key]->diag_status.hardware_id = hw_id;
        return;
    }

    void setDiagLevel(std::string diag_key, int level)
    {
        std::lock_guard<std::mutex> lock(diag_info_map[diag_key]->mtx);
        diag_info_map[diag_key]->diag_status.level = level;
        return;
    }

    void setDiagMsg(std::string diag_key, std::string msg)
    {
        std::lock_guard<std::mutex> lock(diag_info_map[diag_key]->mtx);
        diag_info_map[diag_key]->diag_status.message = msg;
        return;
    }

    bool setDiagKeyVal(std::string diag_key, std::string key, std::string value)
    {
        if (!hasDiagKey(diag_key))
            return false;

        std::lock_guard<std::mutex> lock(diag_info_map[diag_key]->mtx);

        diagnostic_msgs::msg::KeyValue key_val;
        key_val.key = key;
        key_val.value = value;

        auto &values = diag_info_map[diag_key]->diag_status.values;

        // 기존 key가 있는지 찾기
        auto it = std::find_if(values.begin(), values.end(),
                               [&key](const diagnostic_msgs::msg::KeyValue &kv)
                               {
                                   return kv.key == key;
                               });
        // 있으면 값만 업데이트
        if (it != values.end())
        {
            it->value = value;
        }
        else
        {
            // 없으면 새로 추가
            values.push_back(key_val);
        }
        return true;
    }

    bool deleteDiagKeyVal(std::string diag_key, std::string key)
    {
        if (!hasDiagKey(diag_key))
            return false;

        std::lock_guard<std::mutex> lock(diag_info_map[diag_key]->mtx);

        auto &values = diag_info_map[diag_key]->diag_status.values;

        // key가 있는지 찾기
        auto it = std::remove_if(values.begin(), values.end(),
                                 [&key](const diagnostic_msgs::msg::KeyValue &kv)
                                 {
                                     return kv.key == key;
                                 });
        // 있으면 삭제
        if (it != values.end())
        {
            values.erase(it, values.end());
        }
        return true;
    }

    void clearDiagKeyVal(std::string diag_key)
    {
        diag_info_map[diag_key]->diag_status.values.clear();
        return;
    }
    bool deleteDiag(std::string diag_key)
    {
        if (!hasDiagKey(diag_key))
            return false;
        clearDiagKeyVal(diag_key);
        auto it = diag_info_map.find(diag_key);
        if (it != diag_info_map.end())
            diag_info_map.erase(it);
        return true;
    }
    
private:
    uint8_t handleTopicPeriod(std::string check_key)
    {
        if (topic_period_info_map[check_key]->received_time.seconds() == rclcpp::Time(0).seconds())
            return STALE;

        rclcpp::Time cur_time = clock_->now();
        rclcpp::Time prev_time;

        double frame_time_diff;
        {
            std::lock_guard<std::mutex> lock(topic_period_info_map[check_key]->mtx);
            prev_time = topic_period_info_map[check_key]->received_time;
            frame_time_diff = topic_period_info_map[check_key]->frame_time_diff; 
        }

        double time_diff = (cur_time - prev_time).seconds();
    
        

        if ((time_diff >= topic_period_info_map[check_key]->error_timeout)||(frame_time_diff>= topic_period_info_map[check_key]->error_timeout))
        {
            // std::cout<<"time_diff : "<<time_diff<<std::endl;
            // std::cout<<"frame_time_diff in handletopic period : "<<frame_time_diff<<std::endl;
            topic_period_info_map[check_key]->check_status = ERROR;
            setDiagLevel(check_key, ERROR);
            setDiagMsg(check_key, "Error: No message received");
            clearDiagKeyVal(check_key);
            setDiagKeyVal(check_key, "topic", check_key);
            setDiagKeyVal(check_key, "elapsed_time", std::to_string(frame_time_diff));
            setDiagKeyVal(check_key, "Hz", std::to_string(1 / frame_time_diff));

            pubLogMessage(::ERROR, "ERROR!, Hz : " + std::to_string(1 / frame_time_diff));
            return ERROR;
        }
        else if (frame_time_diff >= topic_period_info_map[check_key]->warn_timeout &&
                    topic_period_info_map[check_key]->warn_cnt >= topic_period_info_map[check_key]->warn_max_cnt)
        {
            topic_period_info_map[check_key]->check_status = ERROR;
            topic_period_info_map[check_key]->warn_cnt++;
            std::string total_cnt = std::to_string(topic_period_info_map[check_key]->warn_cnt) + "/" + std::to_string(topic_period_info_map[check_key]->warn_max_cnt)+" "+std::to_string(frame_time_diff);

            setDiagLevel(check_key, ERROR);
            setDiagMsg(check_key, "Error: Waning count max");
            clearDiagKeyVal(check_key);
            setDiagKeyVal(check_key, "topic", check_key);
            setDiagKeyVal(check_key, "warn_cnt/warn_max_cnt and timediff", total_cnt);

            pubLogMessage(::ERROR, "ERROR!, Total warn cnt : " + total_cnt);
            return ERROR;
        }
        else if (frame_time_diff >= topic_period_info_map[check_key]->warn_timeout &&
                    topic_period_info_map[check_key]->warn_cnt < topic_period_info_map[check_key]->warn_max_cnt) 
        {
            topic_period_info_map[check_key]->check_status = WARN;
            topic_period_info_map[check_key]->warn_cnt++;
            setDiagLevel(check_key, WARN);
            setDiagMsg(check_key, "Warning: No message received");
            clearDiagKeyVal(check_key);
            setDiagKeyVal(check_key, "topic", check_key);
            setDiagKeyVal(check_key, "elapsed_time", std::to_string(frame_time_diff));
            setDiagKeyVal(check_key, "Hz", std::to_string(1 / frame_time_diff));

            pubLogMessage(::WARN, "WARN!, Hz : " + std::to_string(1 / frame_time_diff));
            return WARN;
        }
        else if (time_diff >= topic_period_info_map[check_key]->warn_timeout &&
            topic_period_info_map[check_key]->warn_cnt < topic_period_info_map[check_key]->warn_max_cnt)
        {
            topic_period_info_map[check_key]->check_status = WARN;
            setDiagLevel(check_key, WARN);
            setDiagMsg(check_key, "Warning: No message received");
            clearDiagKeyVal(check_key);
            setDiagKeyVal(check_key, "topic", check_key);
            setDiagKeyVal(check_key, "elapsed_time", std::to_string(time_diff));

            pubLogMessage(::WARN, "100ms callback WARN!, Hz : " + std::to_string(1 / time_diff));
            return WARN;
        }
        else
        {
            topic_period_info_map[check_key]->check_status = OK;
            topic_period_info_map[check_key]->warn_cnt = 0;
            setDiagLevel(check_key, OK);
            setDiagMsg(check_key, "");
            clearDiagKeyVal(check_key);
            setDiagKeyVal(check_key, "topic", check_key);
            setDiagKeyVal(check_key, "Hz", std::to_string(1 / frame_time_diff));
            return OK;
        }

        
    }

    void callback100msTimer()
    {
        uint8_t result = OK;
        std_msgs::msg::UInt8 result_status;
    
        for (const auto &[key, period_info] : topic_period_info_map)
        {
            uint8_t status = handleTopicPeriod(key);
            if (status == ERROR)
                result = ERROR;
            else if (status == WARN && result == OK)
                result = WARN;
        }
        
        if(result == ERROR)
        {
            result_status.data = result;
            pubCheck->publish(result_status);
            return;
        }

        for (const auto &[key, diag_info] : diag_info_map)
        {
            std::lock_guard<std::mutex> lock(diag_info->mtx);
            if (diag_info->diag_status.level == ERROR)
                result = ERROR;
            else if (diag_info->diag_status.level == WARN && result == OK)
                result = WARN;
        }
        result_status.data = result;
        pubCheck->publish(result_status);
        return;
    }

    void callback1sTimer()
    {   
        // std::cout<<"callback1sTimer"<<std::endl;

        diag_arr_msg.status.clear();
        diag_arr_msg.header.stamp = rclcpp::Clock().now();
        
        for (const auto &[key, diag_info] : diag_info_map)
        {
            std::lock_guard<std::mutex> lock(diag_info->mtx);
            diag_arr_msg.status.push_back(diag_info->diag_status);

        }
        pubDiagnostic->publish(diag_arr_msg);
        
        return;
    }

    void initPubs()
    {   
        pubLogMsg = node_->create_publisher<rcl_interfaces::msg::Log>("spatial_logmsg", 1);
        pubCheck = node_->create_publisher<std_msgs::msg::UInt8>(param.getParamStr("check_topic"), rclcpp::SensorDataQoS());
        pubDiagnostic = node_->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 10);
        timer_100ms = node_->create_wall_timer(100ms, std::bind(&StatusReporter::callback100msTimer, this));
        timer_1s = node_->create_wall_timer(1s, std::bind(&StatusReporter::callback1sTimer, this));

        return;
    }

    bool hasDiagKey(std::string diag_key)
    {
        return diag_info_map.find(diag_key) != diag_info_map.end();
    }

private:
    rclcpp::Node *node_;
    Parameter param;
    
    std::unordered_map<std::string, std::shared_ptr<diag_info_t>> diag_info_map;
    std::unordered_map<std::string, std::shared_ptr<topic_period_info_t>> topic_period_info_map;

    rclcpp::Clock::SharedPtr clock_;
    rclcpp::TimerBase::SharedPtr timer_100ms, timer_1s;
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr pubDiagnostic;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr pubCheck;
    rclcpp::Publisher<rcl_interfaces::msg::Log>::SharedPtr pubLogMsg;
    diagnostic_msgs::msg::DiagnosticArray diag_arr_msg;
    rcl_interfaces::msg::Log log_msg;
};

#endif