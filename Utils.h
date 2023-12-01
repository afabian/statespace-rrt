#ifndef RRT_UTILS_H
#define RRT_UTILS_H

#include <sys/stat.h>
#include <string>

int mkpath(const char* file_path, mode_t mode);

inline int min(int a, int b) { return (a < b) ? a : b; }
inline float min(float a, float b) { return (a < b) ? a : b; }
inline double min(double a, double b) { return (a < b) ? a : b; }

inline int max(int a, int b) { return (a > b) ? a : b; }
inline float max(float a, float b) { return (a > b) ? a : b; }
inline double max(double a, double b) { return (a > b) ? a : b; }

inline float minmax(float input, float lower_limit, float upper_limit) { return (input < lower_limit) ? lower_limit : ((input > upper_limit) ? upper_limit : input); }
inline double minmax(double input, double lower_limit, double upper_limit) { return (input < lower_limit) ? lower_limit : ((input > upper_limit) ? upper_limit : input); }

inline float sign(float x) { return (x < 0.0f) ? -1.0f : 1.0f; }
inline double sign(double x) { return (x < 0.0f) ? -1.0 : 1.0; }
inline int sign(int x) { return (x < 0) ? -1 : 1; }

#endif //RRT_UTILS_H
