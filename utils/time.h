#ifndef UTIL_TIME_H_
#define UTIL_TIME_H_

#include <iostream>
#include <iomanip>
#include <sys/time.h>
#include <sys/timerfd.h>
inline int init_timerfd(int msec)
{
    int fd = -1;
    struct itimerspec timeout;

    if ((fd = timerfd_create(CLOCK_REALTIME, 0)) < 0)
    {
        printf("init_timer error");
        exit(1);
    }

    timeout.it_interval.tv_sec = 0;
    timeout.it_interval.tv_nsec = msec * 1000 * 1000;
    timeout.it_value.tv_sec = 0;
    timeout.it_value.tv_nsec = msec * 1000 * 1000;
    if (timerfd_settime(fd, 0, &timeout, NULL) != 0)
    {
        printf("timerfd_settime error\n");
        exit(1);
    }

    return fd;
}
inline unsigned long long wait_timerfd(int fd)
{
  unsigned long long missed;
  int result = read(fd, &missed, sizeof(missed));
  return missed;
}
inline time_t TimeFromYMD(int year, int month, int day) {
  struct tm tm = {0};
  tm.tm_year = year - 1900;
  tm.tm_mon = month - 1;
  tm.tm_mday = day;
  return mktime(&tm);
}
inline int getGPSWeek(int year, int month, int day) {
  // See update below
  double diff = difftime(TimeFromYMD(year, month, day), TimeFromYMD(1980, 1, 6));  // See update
  return (int) (diff / (60L*60*24*7));
}

inline int getWeekDay(int y, int m, int d) noexcept
{  
    if(m <= 0 || m > 12)
        return -1;
    static int t[] = { 0, 3, 2, 5, 0, 3, 5, 1, 4, 6, 2, 4 };  
    y -= m < 3;  
    return ( y + y / 4 - y / 100 + y / 400 + t[m - 1] + d) % 7;  
}
inline int getWeekSeconds(int y, int m, int d, int H, int M, int S)
{
  return getWeekDay(y,m,d) * 86400LL + H * 3600 + M * 60 + S;
}
inline std::string getYYMMDDHHMMSS()
{
	std::stringstream fn;
	const time_t t = time(NULL);
	struct tm *now_ptm = localtime(&t);
	// std::cout<<"now_t.tv_sec : "<< now_t.tv_sec << std::endl;

	fn << std::setw(2) << std::setfill('0') << (now_ptm->tm_year + 1900) % 100;
	fn << std::setw(2) << std::setfill('0') << now_ptm->tm_mon + 1;
	fn << std::setw(2) << std::setfill('0') << now_ptm->tm_mday;
  fn << "_";
	fn << std::setw(2) << std::setfill('0') << now_ptm->tm_hour;
	fn << std::setw(2) << std::setfill('0') << now_ptm->tm_min;
	fn << std::setw(2) << std::setfill('0') << now_ptm->tm_sec;

  return fn.str();
}
inline void printCurTime()
{
  struct timeval time_now {};
  gettimeofday(&time_now, nullptr);
  std::stringstream time_ss;
  time_ss << std::setw(10) << std::setfill('0') << time_now.tv_sec;
  time_ss << ".";
  time_ss << std::setw(6) << std::setfill('0') << time_now.tv_usec;
	std::cout<<"["<< time_ss.str() << "]" << std::endl;
}
inline void printTimeDiff(double threshold_sec)
{
  static double prev_usec = 0.0;

  struct timeval time_now
  {
  };
  gettimeofday(&time_now, nullptr);
  // std::stringstream time_ss;
  double usec = time_now.tv_sec % 100 * 1000 * 1000;
  usec += time_now.tv_usec;
  usec = usec / 1000.0 / 1000.0;
  if (prev_usec == 0.0)
  {
    prev_usec = usec;
    return;
  }
  double diff = usec - prev_usec;
  if (diff >= threshold_sec)
  {
    // time_ss << std::setw(8) << std::setfill('0') << diff;
    // std::cout<<"["<< time_ss.str() << "]" << std::endl;
    std::cout << diff << std::endl;
  }
  prev_usec = usec;
}
static double __STOP_WATCH_START = 0;
inline void stopWatchStart()
{
  std::cout << "\n####Stop Watch START####" << std::endl;
  struct timeval time_now
  {
  };
  gettimeofday(&time_now, nullptr);
  // std::stringstream time_ss;
  double usec = time_now.tv_sec % 100 * 1000 * 1000;
  usec += time_now.tv_usec;
  usec = usec / 1000.0 / 1000.0;
  __STOP_WATCH_START = usec;
  return;
}
inline void stopWatchLap(std::string msg)
{
 
  struct timeval time_now
  {
  };
  gettimeofday(&time_now, nullptr);
  // std::stringstream time_ss;
  double usec = time_now.tv_sec % 100 * 1000 * 1000;
  usec += time_now.tv_usec;
  usec = usec / 1000.0 / 1000.0;
  double diff = usec - __STOP_WATCH_START;
    std::cout << msg << " " << diff << std::endl;
  return;
}
#endif