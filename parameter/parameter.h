#ifndef _PARAMETER_H_
#define _PARAMETER_H_

#include <unordered_map>
#include <iostream>
#include <boost/variant.hpp>
#include <string>
#include <vector>
#include <cmath>       // for std::round
#include <algorithm>   // for std::sort
#include <sstream>     // for joining vector<double> into string

typedef unsigned char  uint8;
typedef unsigned short uint16;
typedef unsigned int   uint32;
typedef signed int     int32;

class Parameter
{
public:
    Parameter() {};
    ~Parameter() {};

    // Check if a key exists
    bool find(const std::string &key) const
    {
        return (params.find(key) != params.end());
    }

    // Modify an existing parameter; returns false if key not found
    template <typename T>
    bool modifyParam(const std::string &key, const T &value)
    {
        auto it = params.find(key);
        if (it == params.end())
            return false;
        it->second = value;
        return true;
    }

    // Insert (or overwrite) a parameter; always returns true
    template <typename T>
    bool insertParam(const std::string &key, const T &value)
    {
        if constexpr (std::is_convertible<T, std::string>::value)
            params[key] = std::string(value);  // string 계열로 변환
        else
            params[key] = value;

        return true;
    }

    // Get as int (supports int, string→stoi, double→round, bool→(int), vector<double>→round(first element) if non‐empty)
    int getParamInt(const std::string &key) const
    {
        auto it = params.find(key);
        if (it != params.end())
        {
            const auto &var = it->second;
            switch (var.which())
            {
                case 0: // int
                    return boost::get<int>(var);
                case 1: // std::string
                    try { return std::stoi(boost::get<std::string>(var)); }
                    catch (...) { return 0; }
                case 2: // double
                    return static_cast<int>(std::round(boost::get<double>(var)));
                case 3: // bool
                    return static_cast<int>(boost::get<bool>(var));
                case 4: // std::vector<double>
                {
                    const auto &vec = boost::get<std::vector<double>>(var);
                    if (!vec.empty())
                        return static_cast<int>(std::round(vec.front()));
                    return 0;
                }
                default:
                    return 0;
            }
        }
        return 0;
    }

    // Get as std::string (supports int→to_string, string, double→to_string, bool→"true"/"false", vector<double>→"[v0,v1,...]")
    std::string getParamStr(const std::string &key) const
    {
        auto it = params.find(key);
        if (it != params.end())
        {
            const auto &var = it->second;
            switch (var.which())
            {
                case 0: // int
                    return std::to_string(boost::get<int>(var));
                case 1: // std::string
                    return boost::get<std::string>(var);
                case 2: // double
                    return std::to_string(boost::get<double>(var));
                case 3: // bool
                    return (boost::get<bool>(var) ? "true" : "false");
                case 4: // std::vector<double>
                {
                    const auto &vec = boost::get<std::vector<double>>(var);
                    std::ostringstream oss;
                    oss << "[";
                    for (size_t i = 0; i < vec.size(); ++i)
                    {
                        oss << vec[i];
                        if (i + 1 < vec.size()) 
                            oss << ",";
                    }
                    oss << "]";
                    return oss.str();
                }
                default:
                    return "";
            }
        }
        return "";
    }

    // Get as double (supports int→double, string→stod, double, bool→(double), vector<double>→first element or 0.0)
    double getParamDouble(const std::string &key) const
    {
        auto it = params.find(key);
        if (it != params.end())
        {
            const auto &var = it->second;
            switch (var.which())
            {
                case 0: // int
                    return static_cast<double>(boost::get<int>(var));
                case 1: // std::string
                    try { return std::stod(boost::get<std::string>(var)); }
                    catch (...) { return 0.0; }
                case 2: // double
                    return boost::get<double>(var);
                case 3: // bool
                    return static_cast<double>(boost::get<bool>(var));
                case 4: // std::vector<double>
                {
                    const auto &vec = boost::get<std::vector<double>>(var);
                    if (!vec.empty())
                        return vec.front();
                    return 0.0;
                }
                default:
                    return 0.0;
            }
        }
        return 0.0;
    }

    // Get as bool (supports int→(bool), string→stoi→(bool), double→round→(bool), bool, vector<double>→(bool of first element))
    bool getParamBool(const std::string &key) const
    {
        auto it = params.find(key);
        if (it != params.end())
        {
            const auto &var = it->second;
            switch (var.which())
            {
                case 0: // int
                    return (boost::get<int>(var) != 0);
                case 1: // std::string
                    try { return (std::stoi(boost::get<std::string>(var)) != 0); }
                    catch (...) { return false; }
                case 2: // double
                    return (static_cast<int>(std::round(boost::get<double>(var))) != 0);
                case 3: // bool
                    return boost::get<bool>(var);
                case 4: // std::vector<double>
                {
                    const auto &vec = boost::get<std::vector<double>>(var);
                    if (!vec.empty())
                        return (static_cast<int>(std::round(vec.front())) != 0);
                    return false;
                }
                default:
                    return false;
            }
        }
        return false;
    }

    // New: Get as std::vector<double>. If key not found or wrong type, returns empty vector.
    std::vector<double> getParamVec(const std::string &key) const
    {
        auto it = params.find(key);
        if (it != params.end() && it->second.which() == 4)
        {
            return boost::get<std::vector<double>>(it->second);
        }
        return {};
    }

    // Return the raw variant (so caller can std::get<...>() themselves if needed)
    boost::variant<int, std::string, double, bool, std::vector<double>> getParamVar(const std::string &key) const
    {
        auto it = params.find(key);
        if (it != params.end())
            return it->second;

        // Return an empty‐initialized variant (holds integer 0 by default).
        return boost::variant<int, std::string, double, bool, std::vector<double>>{};
    }

    // Return a copy of the entire map
    std::unordered_map<std::string, boost::variant<int, std::string, double, bool, std::vector<double>>> getAllParams() const
    {
        return params;
    }

    // Print all params in sorted‐by‐key order
    void printAllParams()
    {
        std::cout << "---------------------------------\n";
        // Copy into vector and sort by key
        std::vector<std::pair<std::string, boost::variant<int, std::string, double, bool, std::vector<double>>>> print_params(
            params.begin(), params.end());
        std::sort(print_params.begin(), print_params.end(),
                  [](const auto &a, const auto &b){ return a.first < b.first; });

        for (const auto &p : print_params)
        {
            std::cout << "[" << p.first << "] : ";
            const auto &var = p.second;
            switch (var.which())
            {
                case 0: // int
                    std::cout << boost::get<int>(var);
                    break;
                case 1: // std::string
                    std::cout << boost::get<std::string>(var);
                    break;
                case 2: // double
                    std::cout << boost::get<double>(var);
                    break;
                case 3: // bool
                    std::cout << (boost::get<bool>(var) ? "true" : "false");
                    break;
                case 4: // std::vector<double>
                {
                    const auto &vec = boost::get<std::vector<double>>(var);
                    std::cout << "[";
                    for (size_t i = 0; i < vec.size(); ++i)
                    {
                        std::cout << vec[i];
                        if (i + 1 < vec.size()) std::cout << ",";
                    }
                    std::cout << "]";
                    break;
                }
                default:
                    std::cout << "(unknown)";
            }
            std::cout << "\n";
        }
        std::cout << "---------------------------------\n" << std::endl;
    }

    // Return a list of all keys
    std::vector<std::string> getAllKeys() const
    {
        std::vector<std::string> result;
        result.reserve(params.size());
        for (const auto &p : params)
            result.push_back(p.first);
        return result;
    }

    // Number of stored params
    int size() const
    {
        return static_cast<int>(params.size());
    }

    // Clear all parameters
    void clear()
    {
        params.clear();
    }

protected:
    // Now includes: int, std::string, double, bool, std::vector<double>
    std::unordered_map<
        std::string,
        boost::variant<int, std::string, double, bool, std::vector<double>>
    > params;
};

#endif // _PARAMETER_H_
