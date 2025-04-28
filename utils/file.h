#ifndef UTIL_FILE_H_
#define UTIL_FILE_H_

#include <sys/stat.h>
#include <sys/types.h>
#include <dirent.h>
#include <vector>
#include <string>
#include <cstring>
#include <iostream>
#include <sys/vfs.h>
#include <fstream>
#include <sstream>
#include <filesystem>

inline void mkdirRecur(const std::string& path)
{
    if (!std::filesystem::exists(path)) 
        std::filesystem::create_directories(path);
    
    return;
}
inline void eraseDir(std::string path)
{
    DIR* dir = opendir(path.c_str());
    if (dir == NULL) {
        std::cerr << "Failed to open directory." << std::endl;
        return;
    }

    dirent* entry;
    while ((entry = readdir(dir)) != NULL) {
        if (strcmp(entry->d_name, ".") == 0 || strcmp(entry->d_name, "..") == 0)
            continue;

        std::string subPath = std::string(path) + "/" + entry->d_name;
        if (entry->d_type == DT_DIR) {
            eraseDir(subPath);
        } else {
            if (remove(subPath.c_str()) != 0) {
                std::cerr << "Failed to remove file: " << subPath << std::endl;
            }
        }
    }

    closedir(dir);
    if (rmdir(path.c_str()) != 0) {
        std::cerr << "Failed to remove directory: " << path << std::endl;
    }
    return;
}
inline std::vector<std::string> listDir(const std::string path)
{
    std::vector<std::string> fileName;

    struct dirent *entry;
    DIR *dir = opendir(path.c_str());

    if (dir == NULL)
    {
        return fileName;
    }
    while ((entry = readdir(dir)) != NULL)
    {
        if (strcmp(entry->d_name, ".") == 0 || strcmp(entry->d_name, "..") == 0)
            continue;
        fileName.push_back(entry->d_name);
    }
    closedir(dir);

    return fileName;
}

inline std::string getFileName(const std::string path, const std::string extension)
{
    std::string result = "";

    struct dirent *entry;
    DIR *dir = opendir(path.c_str());

    if (dir == NULL)
    {
        return result;
    }
    while ((entry = readdir(dir)) != NULL)
    {
        if (strcmp(entry->d_name, ".") == 0 || strcmp(entry->d_name, "..") == 0)
            continue;
        result = entry->d_name;
        if (result.compare(result.size() - extension.size(), extension.size(), extension) == 0)
            break;
    }
    closedir(dir);

    return path+result;
}


inline void readMarkerCsv(const std::string path, std::vector<std::vector<double>> &leftMarker, std::vector<std::vector<double>> &rightMarker)
{
    std::ifstream file(path);
    std::string line;
    if (!file.is_open())
    {
        std::cerr << "File could not be opened.\n";
        return;
    }

    // 각 줄을 읽어들임
    while (getline(file, line))
    {
        std::istringstream iss(line);
        std::string token, label;
        double x, y, z;

        // 공백으로 구분된 데이터를 파싱
        iss >> label >> x >> y >> z;

        if (label == "L")
        {
            leftMarker.push_back({x, y, z});
        }
        else if (label == "R")
        {
            rightMarker.push_back({x, y, z});
        }
    }
}
#endif