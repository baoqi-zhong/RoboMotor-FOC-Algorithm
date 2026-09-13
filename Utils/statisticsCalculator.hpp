#include "main.h"

#include "stdint.h"

namespace Utils
{

class StatisticsCalculator
{
public:
    StatisticsCalculator(float dropRate = 0);
    void reset();
    void setDropRate(float dropRate);
    void addData(float data);
    float getMean();
    float getVariance();

private:
    /* data */
    float n;
    float correctedSumOfSquares;
    float mean;
    float variance;
    uint32_t dropPeriod;
    uint32_t dropCounter;

};

} // namespace Utils