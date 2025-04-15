#ifndef _DIAGNOSTIC_HPP
#define _DIAGNOSTIC_HPP

#include "rclcpp/rclcpp.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "parameter/parameter.h"

#pragma once

using namespace std::chrono_literals;

class DiagnosticHandler
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
    } topic_period_info_t;
    enum
    {
        OK = 0,
        WARN = 1,
        ERROR = 2,
        STALE = 3
    };
public:
    DiagnosticHandler(rclcpp::Node *node, std::string pub_topic) : node_(node), clock_(std::make_shared<rclcpp::Clock>(RCL_ROS_TIME))
    {
        param.insertParam("node_name", node->get_fully_qualified_name());
        param.insertParam("pub_topic", pub_topic);
        initPubs();
    }
    ~DiagnosticHandler() {}

    void createTopicPeriodInfo(std::string check_key, double warn_timeout, int warn_max_cnt, double error_timeout)
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
        topic_period_info->received_time = rclcpp::Time(0);
        topic_period_info->check_status = OK;
        topic_period_info_map[check_key] = topic_period_info;

        createDiagInfo(check_key);

        return;
    }

    void updateTopicPeriod(std::string check_key)
    {
        std::lock_guard<std::mutex> lock(topic_period_info_map[check_key]->mtx);
        topic_period_info_map[check_key]->received_time = clock_->now();
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

private:
    uint8_t handleTopicPeriod(std::string check_key)
    {
        if (topic_period_info_map[check_key]->received_time.seconds() == rclcpp::Time(0).seconds())
            return STALE;

        rclcpp::Time cur_time = clock_->now();
        topic_period_info_map[check_key]->mtx.lock();
        rclcpp::Time prev_time = topic_period_info_map[check_key]->received_time;
        topic_period_info_map[check_key]->mtx.unlock();

        double time_diff = (cur_time - prev_time).seconds();

        if (time_diff >= topic_period_info_map[check_key]->error_timeout)
        {
            topic_period_info_map[check_key]->check_status = ERROR;
            setDiagLevel(check_key, ERROR);
            setDiagMsg(check_key, "Error: No message received");
            clearDiagKeyVal(check_key);
            setDiagKeyVal(check_key, "topic", check_key);
            setDiagKeyVal(check_key, "elapsed_time", std::to_string(time_diff));
            return ERROR;
        }
        else if (time_diff >= topic_period_info_map[check_key]->warn_timeout &&
                 topic_period_info_map[check_key]->warn_cnt >= topic_period_info_map[check_key]->warn_max_cnt)
        {
            topic_period_info_map[check_key]->check_status = ERROR;
            topic_period_info_map[check_key]->warn_cnt++;
            std::string total_cnt = std::to_string(topic_period_info_map[check_key]->warn_cnt) + "/" + std::to_string(topic_period_info_map[check_key]->warn_max_cnt);

            setDiagLevel(check_key, ERROR);
            setDiagMsg(check_key, "Error: Waning count max");
            clearDiagKeyVal(check_key);
            setDiagKeyVal(check_key, "topic", check_key);
            setDiagKeyVal(check_key, "warn_cnt/warn_max_cnt", total_cnt);
            return ERROR;
        }
        else if (time_diff >= topic_period_info_map[check_key]->warn_timeout &&
                 topic_period_info_map[check_key]->warn_cnt < topic_period_info_map[check_key]->warn_max_cnt)
        {
            topic_period_info_map[check_key]->check_status = WARN;
            topic_period_info_map[check_key]->warn_cnt++;
            setDiagLevel(check_key, WARN);
            setDiagMsg(check_key, "Warning: No message received");
            clearDiagKeyVal(check_key);
            setDiagKeyVal(check_key, "topic", check_key);
            setDiagKeyVal(check_key, "elapsed_time", std::to_string(time_diff));
            
            return WARN;
        }
        else
        {
            topic_period_info_map[check_key]->check_status = OK;
            topic_period_info_map[check_key]->warn_cnt = 0;
            setDiagLevel(check_key, OK);
            setDiagMsg(check_key, "");
            clearDiagKeyVal(check_key);
        }

        return OK;
    }

    void callback100msTimer()
    {
        uint8_t result = OK;
        std_msgs::msg::UInt8 check_status;

        for (const auto &[key, diag_info] : topic_period_info_map)
        {
            uint8_t status = handleTopicPeriod(key);
            if (status == ERROR)
                result = ERROR;
            else if (status == WARN && result == OK)
                result = WARN;
        }
        check_status.data = result;
        pubCheck->publish(check_status);
        return;
    }

    void callback1sTimer()
    {
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
        pubCheck = node_->create_publisher<std_msgs::msg::UInt8>(param.getParamStr("pub_topic"), rclcpp::SensorDataQoS());
        pubDiagnostic = node_->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 10);
        timer_100ms = node_->create_wall_timer(100ms, std::bind(&DiagnosticHandler::callback100msTimer, this));
        timer_1s = node_->create_wall_timer(1s, std::bind(&DiagnosticHandler::callback1sTimer, this));

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
    diagnostic_msgs::msg::DiagnosticArray diag_arr_msg;
};

#endif