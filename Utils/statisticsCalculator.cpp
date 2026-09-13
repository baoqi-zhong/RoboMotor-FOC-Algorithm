#include "statisticsCalculator.hpp"

namespace Utils
{
StatisticsCalculator::StatisticsCalculator(float dropRate)
{
    this->reset();
    this->setDropRate(dropRate);
}

void StatisticsCalculator::reset()
{
    this->n = 0;
    this->correctedSumOfSquares = 0;
    this->mean = 0;
    this->variance = 0;
    this->dropPeriod = 0;
    this->dropCounter = 0;
}

void StatisticsCalculator::setDropRate(float dropRate)
{
    if(dropRate == 0)
    {
        this->dropPeriod = 0;
    }
    else
    {
        this->dropPeriod = 1.0f / (1.0f - dropRate);    
    }

}

void StatisticsCalculator::addData(float data)
{
    if(this->dropPeriod)
        this->dropCounter++;
    
    if(this->dropPeriod == 0 || this->dropCounter > this->dropPeriod)
    {
        float oldMean = this->mean;
        this->mean += (data - oldMean) / (this->n + 1);
        this->correctedSumOfSquares += (data - oldMean) * (data - this->mean);
        this->n++;    

        this->dropCounter = 0;
        getVariance();
    }
}

float StatisticsCalculator::getMean()
{
    return this->mean;
}

float StatisticsCalculator::getVariance()
{
    if(this->n == 0)
        return 0;

    this->variance = this->correctedSumOfSquares / this->n;
    return this->variance;
}
} // namespace Utils