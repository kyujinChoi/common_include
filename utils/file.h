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



inline void mkdirRecur(std::string path)
{
    mkdir(path.c_str(), 0777);
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
#endif