#ifndef COMMON_TIME_HPP_
#define COMMON_TIME_HPP_
#include <ros/ros.h>
#include <chrono>
#include <ostream>
#include <ratio>
#include <time.h>
#include <cerrno>
#include <cstring>
#include <string>
#include <glog/logging.h>

constexpr int64_t kUtsEpochOffsetFromUnixEpochInSeconds = (719162ll * 24ll * 60ll * 60ll);
struct UniversalTimeScaleClock {
    using rep = int64_t;
    using period = std::ratio<1, 1000000>;
    using duration = std::chrono::duration<rep, period>;
    using time_point = std::chrono::time_point<UniversalTimeScaleClock>;
    static constexpr bool is_steady = true;
};
using Duration = UniversalTimeScaleClock::duration;
using Time = UniversalTimeScaleClock::time_point;
Duration  FromSeconds(double seconds);
Duration  FromMilliseconds(int64_t milliseconds);
double    ToSeconds(Duration duration);
double    ToSeconds(std::chrono::steady_clock::duration duration);
double    ToSeconds(const Time& time);
Time      FromUniversal(int64_t ticks);
int64_t   ToUniversal(Time time);
double    GetThreadCpuTimeSeconds();
Time      GetNow();
Time      Get10msNow();
ros::Time ToRos(Time time);
Time      FromRos(const ::ros::Time& time);
ros::Time MonotonicToRos(const struct timeval time);

std::ostream& operator<<(std::ostream& os, Time time);
bool          operator>(const Time& lhs, const Time& rhs);
bool          operator<(const Time& lhs, const Time& rhs);
bool          operator!=(const Time& lhs, const Time& rhs);
bool          operator==(const Time& lhs, const Time& rhs);
bool          operator<=(const Time& lhs, const Time& rhs);
bool          operator>=(const Time& lhs, const Time& rhs);
Time          operator-(const Time& lhs, const double & duration);
Time          operator+(const Time& lhs, const double & duration);
#endif
