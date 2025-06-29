#ifndef UTIL_STRING_H_
#define UTIL_STRING_H_

#include <string>
#include <cstring>
#include <vector>
#include <sstream>
#include <iomanip>
#include <algorithm>
inline std::string to_string_with_precision(double value, int precision = 3)
{
    std::ostringstream out;
    out << std::fixed << std::setprecision(precision) << value;
    return out.str();
}
inline std::vector<std::string> splitWithEmpty(std::string str, char Delimiter)
{
    std::vector<std::string> result; // istringstream에 str을 담는다.
    std::string buffer;          // 구분자를 기준으로 절삭된 문자열이 담겨지는 버퍼

    for (size_t i = 0; i < str.size(); ++i)
    {
        if (str[i] == Delimiter)
        {
            result.push_back(buffer);
            buffer.clear();
        }
        else
        {
            buffer += str[i];
        }
    }

    // 마지막 항목 추가 (마지막에 Delimiter가 있어도 ""를 넣어야 함)
    result.push_back(buffer);

    return result;
}
inline std::vector<std::string> split(std::string str, char Delimiter)
{
    std::istringstream iss(str); // istringstream에 str을 담는다.
    std::string buffer;          // 구분자를 기준으로 절삭된 문자열이 담겨지는 버퍼

    std::vector<std::string> result;

    // istringstream은 istream을 상속받으므로 getline을 사용할 수 있다.
    while (getline(iss, buffer, Delimiter))
    {
        buffer.erase(std::remove(buffer.begin(), buffer.end(), ' '), buffer.end());
        if (!buffer.empty())
            result.push_back(buffer);
    }

    return result;
}
inline std::string replaceAll(std::string str, std::string from_delimiter, std::string to_delimiter)
{
    std::string result = str;
    size_t pos = result.find(from_delimiter);
    while (pos != std::string::npos)
    {
        result.replace(pos, from_delimiter.length(), to_delimiter);
        pos = result.find(from_delimiter, pos + to_delimiter.length());
    }
    return result;
}
inline bool isContain(std::string source_str, std::string sub_str)
{
    std::string::size_type n = source_str.find(sub_str);
    if (n == std::string::npos)
        return false;
    else
        return true;
}
#endif