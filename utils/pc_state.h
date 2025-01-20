#ifndef _PC_STATE_H_
#define _PC_STATE_H_
// #pragma once

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <sstream>
#include <mutex>

struct CpuUsage
{
    long user;       // 사용자 모드 시간
    long nice;       // 우선 순위가 낮은 프로세스의 사용자 모드 시간
    long system;     // 커널 모드 시간
    long idle;       // 유휴 시간
    long iowait;     // I/O 대기 시간
    long irq;        // 하드웨어 인터럽트 시간
    long softirq;    // 소프트웨어 인터럽트 시간
    long steal;      // 가상 머신이 사용할 수 없는 CPU 시간
    long guest;      // 게스트 운영 체제의 사용자 모드 시간
    long guest_nice; // 게스트 운영 체제의 우선 순위가 낮은 사용자 모드 시간

    long total() const
    {
        return user + nice + system + idle + iowait + irq + softirq + steal + guest + guest_nice;
    }

    long active() const
    {
        return user + nice + system + irq + softirq + steal + guest + guest_nice;
    }
};

struct NetUsage
{
    unsigned long long receive_bytes;
    unsigned long long transmit_bytes;
};

class PCState
{
public:
    PCState()
    {
        cpu_mon_thread = std::thread(&PCState::monitorCpuUsage, this);
        net_mon_thread = std::thread(&PCState::monitorNetUsage, this);
    };
    ~PCState() {};
    void joinMonitorThread()
    {
        if (cpu_mon_thread.joinable())
            cpu_mon_thread.join();
        if (net_mon_thread.joinable())
            net_mon_thread.join();
        return;
    }
    std::vector<float> getCpuUsage()
    {
        // CPU 사용량 출력
        while(!is_initialized)
            usleep(1000);
        cpu_mtx.lock();
        for (size_t i = 1; i < prev_cpuUsages.size(); ++i)
        { // 첫 번째 "cpu"는 전체 CPU 사용량
            const auto &initial = prev_cpuUsages[i];
            const auto &final = cur_cpuUsages[i];

            long totalInitial = initial.total();
            long totalFinal = final.total();
            long activeInitial = initial.active();
            long activeFinal = final.active();

            // 시간 경과에 따른 사용량 계산
            long totalDiff = totalFinal - totalInitial;
            long activeDiff = activeFinal - activeInitial;

            if (totalDiff > 0)
            {
                float usagePercent = (static_cast<float>(activeDiff) / totalDiff) * 100;
                cpu_usage[i-1] = usagePercent;
            }
        }
        cpu_mtx.unlock();
        return cpu_usage;
    }
    std::unordered_map<std::string, std::pair<float, float>> getNetUsageMB()
    {
        while(!is_initialized)
            usleep(1000);
        net_mtx.lock();
        for (const auto &[iface, initial] : prev_netUsages)
        {
            if (cur_netUsages.find(iface) != cur_netUsages.end())
            {
                const auto &final = cur_netUsages[iface];

                unsigned long long recv_diff = final.receive_bytes - initial.receive_bytes;
                unsigned long long trans_diff = final.transmit_bytes - initial.transmit_bytes;

                // 초당 바이트 수 계산
                double recv_Bps = static_cast<double>(recv_diff) / 1.0;   // B/s
                double trans_Bps = static_cast<double>(trans_diff) / 1.0; // B/s

                net_usage[iface] = std::make_pair(recv_Bps / (1024.0 * 1024.0), trans_Bps / (1024.0 * 1024.0));
                // std::cout << "인터페이스: " << iface << "\n";
                // std::cout << "수신: " << recv_Bps << " B/s, 전송: " << trans_Bps << " B/s\n\n";
            }
        }
        net_mtx.unlock();
        return net_usage;
    }
    std::pair<int, int> getMemoryUsageMB()
    {
        while(!is_initialized)
            usleep(1000);
        std::ifstream file("/proc/meminfo");
        std::string currentKey;
        long memTotal = 0, memFree = 0, buffers = 0, cached = 0;

        while (file >> currentKey)
        {
            if (currentKey == "MemTotal:")
            {
                file >> memTotal;
                memTotal *= 1024; // 바이트로 변환
            }
            else if (currentKey == "MemFree:")
            {
                file >> memFree;
                memFree *= 1024;
            }
            else if (currentKey == "Buffers:")
            {
                file >> buffers;
                buffers *= 1024;
            }
            else if (currentKey == "Cached:")
            {
                file >> cached;
                cached *= 1024;
            }
            file.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        }

        // 사용 중인 메모리 계산
        long usedMemory = (memTotal / 1000000) - (memFree / 1000000) - (buffers / 1000000) - (cached / 1000000);

        return std::make_pair((memTotal / 1000000), usedMemory);
    }
    inline std::pair<double, double> getDriveUsageGB(std::string path)
    {
        double dGB = -1;
        double dGBUsed = -1;
        struct statfs sb;
        if ((statfs(path.c_str(), &sb)) == 0)
        {
            //        printf("optimal transfer blk size is %d\n",sb.f_bsize);
            //        printf("total data blocks are %d\n",sb.f_blocks);
            //        printf("free blocks in fs are %d\n",sb.f_bfree);
            //        printf("total file nodes in fs are %d\n",sb.f_files);
            //        printf("free file nodes in fs are %d\n",sb.f_ffree);

            dGB = (double)(sb.f_blocks * sb.f_bsize) / 1073741824;
            // double dGBRatio = (double)((double)(sb.f_blocks - sb.f_bfree) / sb.f_blocks) * 100.0;
            dGBUsed = (double)((sb.f_blocks - sb.f_bfree) * sb.f_bsize) / 1073741824;
            // dGBRatio = dGBUsed / dGB * 100.0;
        }
        return std::make_pair(dGB, dGBUsed);
    }
private:
    void monitorCpuUsage()
    {
        bool is_first = true;
        while (1)
        {
            cpu_mtx.lock();
            if (is_first)
            {
                readCpuUsage(prev_cpuUsages);
                cpu_usage.resize(prev_cpuUsages.size() - 1);
            }
            else
            {
                prev_cpuUsages = cur_cpuUsages;
                readCpuUsage(cur_cpuUsages);
            }
            cpu_mtx.unlock();
            if (is_first)
            {
                is_first = false;
                continue;
            }
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
        return;
    }
    void readCpuUsage(std::vector<CpuUsage> &m_cpu_vec)
    {
        std::ifstream file("/proc/stat");
        std::string line;
        m_cpu_vec.clear();
        while (std::getline(file, line))
        {
            if (line.substr(0, 3) == "cpu")
            { // "cpu"로 시작하는 줄을 찾습니다.
                std::istringstream ss(line);
                std::string cpuLabel;
                CpuUsage usage;

                ss >> cpuLabel; // "cpu" 또는 "cpu0", "cpu1" 등 읽기
                ss >> usage.user >> usage.nice >> usage.system >> usage.idle >> usage.iowait >> usage.irq >> usage.softirq >> usage.steal >> usage.guest >> usage.guest_nice;

                m_cpu_vec.push_back(usage);
            }
        }
        return;
    }
    void monitorNetUsage()
    {
        bool is_first = true;
        while (1)
        {
            net_mtx.lock();
            if (is_first)
            {
                readNetUsage(prev_netUsages);
            }
            else
            {
                prev_netUsages = cur_netUsages;
                readNetUsage(cur_netUsages);
                is_initialized = true;
            }
            net_mtx.unlock();
            if (is_first)
            {
                is_first = false;
                continue;
            }
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
        return;
    }
    void readNetUsage(std::unordered_map<std::string, NetUsage> &m_net_vec)
    {
        std::ifstream file("/proc/net/dev");
        std::string line;

        // 첫 두 줄은 헤더이므로 건너뜁니다
        std::getline(file, line);
        std::getline(file, line);
        m_net_vec.clear();
        while (std::getline(file, line))
        {
            std::istringstream iss(line);
            std::string iface;
            NetUsage data;

            // 인터페이스 이름 추출
            std::getline(iss, iface, ':');
            // 공백 제거
            iface.erase(0, iface.find_first_not_of(" \t"));

            // 수신 바이트, 패킷 등 추출
            iss >> data.receive_bytes;
            // 패킷, 에러 등은 건너뜀
            unsigned long dummy;
            for (int i = 0; i < 7; ++i)
            {
                iss >> dummy;
            }

            // 전송 바이트 추출
            iss >> data.transmit_bytes;
            // 나머지 패킷, 에러 등은 건너뜀

            m_net_vec[iface] = data;
        }
        return;
    }

public:
private:
    std::unordered_map<std::string, NetUsage> prev_netUsages, cur_netUsages;
    std::vector<CpuUsage> prev_cpuUsages, cur_cpuUsages;
    std::vector<float> cpu_usage;
    std::mutex cpu_mtx, net_mtx;
    std::thread cpu_mon_thread, net_mon_thread;
    std::unordered_map<std::string, std::pair<float, float>> net_usage;
    bool is_initialized = false;
};
#endif
