

/* 常数和基本函数 */
#define PI                          3.1415926f
#define TWO_PI                      6.2831853f
#define ONE_OVER_SQRT3              0.5773503f
#define TWO_OVER_SQRT3              1.1547005f
#define SQRT3                       1.7320508f
#define SQRT3_OVER_2                0.8660254f
#define RPM_TO_RAD_PER_S_RATIO      0.1047198f
#define RAD_PER_S_TO_RPM_RATIO      9.549296f

#define CLAMP(x, min, max) ((x) > (max) ? (max) : ((x) < (min) ? (min) : (x)))

#define FABS(x)     ((x) > 0 ? (x) : -(x))
#define MIN(a,b)    ((a)<(b)?(a):(b))
#define MAX(a,b)    ((a)>(b)?(a):(b))
#define FMOD(x, y)  ((x) - (int)((x) / (y)) * (y))
