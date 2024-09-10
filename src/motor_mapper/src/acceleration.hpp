
#ifndef ACCELERATION_HPP
#define ACCELERATION_HPP

#include <ros/ros.h>

#include <chrono>
#include <cmath>
#include <cstdint>

namespace acceleration
{

inline std::int64_t millis()
{
    return std::chrono::duration_cast<std::chrono::milliseconds>(
               std::chrono::system_clock::now().time_since_epoch())
        .count();
}

class Accelerator
{
  public:
    Accelerator() {};
    virtual ~Accelerator() {};

    virtual float getNextValue(const float forcing)
    {
        throw std::runtime_error(
            "Accelerator base class cannot be directly used.");
    };

    virtual void fillMemory(const float lastValue)
    {
        throw std::runtime_error(
            "Accelerator base class cannot be directly used.");
    };
};

template<std::size_t MEMORY_LENGTH>
class SCurve : public Accelerator
{
  protected:
    std::array<float, MEMORY_LENGTH> filter;
    std::array<float, MEMORY_LENGTH> memory;
    std::size_t                      lastMemoryIndex;

  public:
    SCurve()
    {
        memory.fill(0);
        lastMemoryIndex = 0;

        // Build a filter to convolve with the input
        float stepSize  = 1.0f / MEMORY_LENGTH;
        float filterSum = 0;

        for (std::size_t idx = 0; idx < MEMORY_LENGTH; idx++)
        {
            if (idx < MEMORY_LENGTH / 2)
            {
                filter[idx] = idx * stepSize;
            }
            else
            {
                filter[idx] = 1 - (idx * stepSize);
            }
            filterSum += filter[idx];
        }
        for (std::size_t idx = 0; idx < MEMORY_LENGTH; idx++)
        {
            filter[idx] /= filterSum;
        }
    }

    float getNextValue(const float forcing)
    {
        static float       sum;
        static std::size_t idx;
        static std::size_t mem_idx;

        // Append the memory with the new value
        memory[lastMemoryIndex++] = forcing;
        lastMemoryIndex
            %= MEMORY_LENGTH; // If we overflow, rotate back to the beginning

        sum = 0;
        // Convolve the filter coefficients with the memory
        for (idx = 0; idx < MEMORY_LENGTH; idx++)
        {
            mem_idx  = (idx + lastMemoryIndex) % MEMORY_LENGTH;
            sum     += filter[idx] * memory[mem_idx];
        }

        return sum;
    }

    void fillMemory(const float lastValue) { memory.fill(lastValue); }
};

class Holder : public Accelerator
{
  protected:
    Accelerator* accelerator;
    float        heldValue;

  public:
    Holder(const Accelerator* a)
        : accelerator(a)
    {
        heldValue = 0;
    }

    virtual float getNextValue(const float forcing)
    {
        if (forcing != 0)
        {
            heldValue = accelerator->getNextValue(forcing);
            return heldValue;
        }

        // Keep the child accelerator up to date
        accelerator->getNextValue(forcing);
        return heldValue;
    };

    virtual void fillMemory(const float lastValue)
    {
        heldValue = lastValue;
        return accelerator->fillMemory(lastValue);
    };
};

class DerivativeLimiter : public Accelerator
{
  protected:
    const float  limitPerMillisecond;
    float        previousValue;
    std::int64_t previousTime;

  public:
    DerivativeLimiter(const float tau_seconds)
        : limitPerMillisecond(1.0 / (1000.0 * tau_seconds))
    {
        previousValue = 0;
        previousTime  = 0;
    }

    virtual float getNextValue(const float forcing)
    {
        if (previousTime == 0)
        {
            previousTime  = millis();
            previousValue = forcing;
            return forcing;
        }
        std::int64_t sampleTime = millis();
        std::int64_t timeDelta  = sampleTime - previousTime;
        previousTime            = sampleTime;

        // This is inverted to make the math below work without division
        float delta = forcing - previousValue;

        float rate = delta / timeDelta;

        if (rate > limitPerMillisecond)
        {
            previousValue += timeDelta * limitPerMillisecond;
        }
        else if (rate < -limitPerMillisecond)
        {
            previousValue -= timeDelta * limitPerMillisecond;
        }
        else
        {
            previousValue = forcing;
        }
        return previousValue;
    };

    virtual void fillMemory(const float lastValue)
    {
        previousValue = lastValue;
        previousTime  = 0;
    };
};

// class NthOrderLimiter : public Accelerator {
//  protected:
//   const std::vector<float> limits;
//   std::vector<float> memory;
//   size_t order;

//  public:
//   NthOrderLimiter(std::vector<float> lim) : limits(lim) {
//     order = lim.size();
//     memory.resize(order);
//   }
//   virtual float getNextValue(float forcing) {
// }
//   virtual void fillMemory(float last_value) {
//   };
// };

}; // namespace acceleration
#endif // End of include guard for ACCELERATION_HPP
