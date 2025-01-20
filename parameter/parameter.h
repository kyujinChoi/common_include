#ifndef _PARAMETER_H_
#define _PARAMETER_H_
// #pragma once

#include <unordered_map>
#include <iostream>
#include <boost/variant.hpp>
#include <string>
#include <vector>
#include <math.h>
#include "coconut/coconut.h"

#define PI 3.141592
typedef unsigned char uint8;
typedef unsigned short uint16;
typedef unsigned int uint32;
typedef signed int int32;

class Parameter
{
public:
    Parameter(){};
    ~Parameter(){};

    bool find(std::string key)
    {
        if (params.find(key) == params.end())
            return false;
        return true;
    }
    
    bool modifyParam(std::string key, boost::variant<int, std::string, double> value)
    {
        if (params.find(key) == params.end())
            return false;
        params[key] = value;
        return true;
    }
    bool insertParam(std::string key, boost::variant<int, std::string, double> value)
    {
        if (params.find(key) == params.end())
        {
            params[key] = value;
            return true;
        }
        else
        {
            params[key] = value;
            return true;
        }
        return false;
    }
    int getParamInt(std::string key)
    {
        auto iter = params.find(key);
        if (iter != params.end())
        {
            switch(iter->second.which())
            {
                case 0 :
                    return boost::get<int>(iter->second);
                case 1 :
                    return std::stoi(boost::get<std::string>(iter->second));
                case 2 :
                    return (int)round(boost::get<double>(iter->second));
            }
        }
        return 0;
    }
    std::string getParamStr(std::string key)
    {
        auto iter = params.find(key);
        if (iter != params.end())
        {
            switch(iter->second.which())
            {
                case 0 :
                    return std::to_string(boost::get<int>(iter->second));
                case 1 :
                    return boost::get<std::string>(iter->second);
                case 2 :
                    return std::to_string(boost::get<double>(iter->second));
            }
        }
        return "";
    }
    double getParamDouble(std::string key)
    {
        auto iter = params.find(key);
        if (iter != params.end())
        {
            switch(iter->second.which())
            {
                case 0 :
                    return double(boost::get<int>(iter->second));
                case 1 :
                    return std::stod(boost::get<std::string>(iter->second));
                case 2 :
                    return boost::get<double>(iter->second);
            }
        }
        return 0;
    }
    boost::variant<int, std::string, double> getParamVar(std::string key)
    {
        auto iter = params.find(key);
        if (iter != params.end())
        {
            return iter->second;
        }
        return boost::variant<int, std::string>{};
    }
    std::unordered_map<std::string, boost::variant<int, std::string, double>> getAllParams()
    {
        return params;
    }
    void printAllParams()
    {
        std::cout << "---------------------------------\n";
        std::vector<std::pair<std::string, boost::variant<int, std::string, double>>> print_params(params.begin(), params.end());
        std::sort(print_params.begin(), print_params.end());
        for (auto iter = print_params.begin(); iter != print_params.end(); ++iter)
        {
            std::cout << "[" << iter->first << "] : " << iter->second << std::endl;
        }
        std::cout << "---------------------------------\n";
        return;
    }
    std::vector<std::string> getAllKeys()
    {
        std::vector<std::string> result;
        for (auto iter = params.begin(); iter != params.end(); ++iter)
        {
            result.push_back(iter->first);
        }
        return result;
    }
    int size()
    {
        return params.size();
    }
    void clear()
    {
        return params.clear();
    }

protected:
    std::unordered_map<std::string, boost::variant<int, std::string, double>> params;
};
#endif