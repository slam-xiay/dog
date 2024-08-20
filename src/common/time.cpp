#include "time.hpp"



Duration FromSeconds(const double seconds) {
  return std::chrono::duration_cast<Duration>(std::chrono::duration<double>(seconds));
}

double ToSeconds(const Duration duration) {
  return std::chrono::duration_cast<std::chrono::duration<double>>(duration).count();
}

double ToSeconds(const std::chrono::steady_clock::duration duration) {
  return std::chrono::duration_cast<std::chrono::duration<double>>(duration).count();
}

double ToSeconds(const Time& time) { return double(ToUniversal(time)) / 1e6; }

Time GetNow() {
  int64_t time =
      std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch())
          .count();
  return FromUniversal(time);
}

Time Get10msNow() {
  int64_t time =
      50000 + std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch())
                  .count();
  return FromUniversal(time);
}

Time FromUniversal(const int64_t ticks) { return Time(Duration(ticks)); }

int64_t ToUniversal(const Time time) { return time.time_since_epoch().count(); }

std::ostream& operator<<(std::ostream& os, const Time time) {
  os << std::to_string(ToSeconds(time));
  return os;
}

Duration FromMilliseconds(const int64_t milliseconds) {
  return std::chrono::duration_cast<Duration>(std::chrono::milliseconds(milliseconds));
}

double GetThreadCpuTimeSeconds() {
  struct timespec thread_cpu_time;
  CHECK(clock_gettime(CLOCK_THREAD_CPUTIME_ID, &thread_cpu_time) == 0) << std::strerror(errno);
  return thread_cpu_time.tv_sec + 1e-9 * thread_cpu_time.tv_nsec;
}

ros::Time ToRos(Time time) {
  int64_t uts_timestamp = 1000 * ToUniversal(time);
  // int64_t   ns_since_unix_epoch = (uts_timestamp - kUtsEpochOffsetFromUnixEpochInSeconds * 10000000ll) * 100ll;
  ros::Time ros_time;
  ros_time.fromNSec(uts_timestamp);
  return ros_time;
}

Time FromRos(const ros::Time& time) {
  return FromUniversal(time.sec * 1000000ll + time.nsec / 1000);
  // return FromUniversal((time.sec + kUtsEpochOffsetFromUnixEpochInSeconds) * 10000000ll + (time.nsec + 50) / 100);
}

ros::Time TimesampToRos(int64_t timesamp) {
  std::string suanz = std::to_string(timesamp);
  std::string sec_string = suanz.substr(0, 10);
  std::string nsec_string = suanz.substr(10, 9);
  while (nsec_string.length() < 9) {
    nsec_string += "0";
  }
  return ros::Time(std::stoi(sec_string), std::stoi(nsec_string));
}

ros::Time MonotonicToRos(const struct timeval time) {
  struct timespec monotonic = {0, 0};
  struct timespec realtime = {0, 0};
  struct timespec tfedtime = {0, 0};
  TIMEVAL_TO_TIMESPEC(&time, &tfedtime);
  clock_gettime(CLOCK_MONOTONIC, &monotonic);
  clock_gettime(CLOCK_REALTIME, &realtime);
  return ros::Time(tfedtime.tv_sec + realtime.tv_sec - monotonic.tv_sec,
                   tfedtime.tv_nsec + realtime.tv_nsec - monotonic.tv_nsec);
}

bool operator>(const Time& lhs, const Time& rhs) { return (ToUniversal(lhs) - ToUniversal(rhs) > 1000); }

bool operator<(const Time& lhs, const Time& rhs) { return (ToUniversal(lhs) - ToUniversal(rhs) < 1000); }

bool operator!=(const Time& lhs, const Time& rhs) { return (fabs(ToUniversal(lhs) - ToUniversal(rhs)) > 1000); }

bool operator==(const Time& lhs, const Time& rhs) { return (fabs(ToUniversal(lhs) - ToUniversal(rhs)) < 1000); }

bool operator<=(const Time& lhs, const Time& rhs) { return ((ToUniversal(rhs) - ToUniversal(lhs)) > 1000); }

bool operator>=(const Time& lhs, const Time& rhs) { return ((ToUniversal(lhs) - ToUniversal(rhs)) > 1000); }

Time operator-(const Time& lhs, const double & duration){
  return (lhs - FromSeconds(duration));
}

Time operator+(const Time& lhs, const double & duration){
  return (lhs + FromSeconds(duration));
}

