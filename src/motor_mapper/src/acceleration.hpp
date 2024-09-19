#pragma once

#include <ros/ros.h>

#include <array>
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
    std::array<float, MEMORY_LENGTH> p_Filter;
    std::array<float, MEMORY_LENGTH> p_Memory;
    std::size_t                      p_LastMemoryIndex;

  public:
    SCurve()
    {
        p_Memory.fill(0);
        p_LastMemoryIndex = 0;

        // Build a filter to convolve with the input
        float stepSize  = 1.0f / MEMORY_LENGTH;
        float filterSum = 0.0f;

        for (std::size_t idx = 0; idx < MEMORY_LENGTH; idx++)
        {
            if (idx < MEMORY_LENGTH / 2)
            {
                p_Filter[idx] = idx * stepSize;
            }
            else
            {
                p_Filter[idx] = 1 - (idx * stepSize);
            }
            filterSum += p_Filter[idx];
        }
        for (std::size_t idx = 0; idx < MEMORY_LENGTH; idx++)
        {
            p_Filter[idx] /= filterSum;
        }
    }

    float getNextValue(const float forcing)
    {
        static float       checksum;
        static std::size_t memoryIndex;

        // Append the memory with the new value
        p_Memory[p_LastMemoryIndex++] = forcing;
        p_LastMemoryIndex
            %= MEMORY_LENGTH; // If we overflow, rotate back to the beginning

        checksum = 0;
        // Convolve the filter coefficients with the memory
        for (std::size_t idx = 0; idx < MEMORY_LENGTH; idx++)
        {
            memoryIndex  = (idx + p_LastMemoryIndex) % MEMORY_LENGTH;
            checksum    += p_Filter[idx] * p_Memory[memoryIndex];
        }

        return checksum;
    }

    void fillMemory(const float lastValue) { p_Memory.fill(lastValue); }
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
