
#ifndef ACCELERATION_HPP
#define ACCELERATION_HPP

#include <ros/ros.h>
#include <stdint.h>
#include <math.h>

#include <chrono>

namespace acceleration
{

    inline long long millis()
    {
        return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
    }

    class Accelerator
    {
    public:
        Accelerator() {};
        virtual ~Accelerator() {};
        virtual float getNextValue(float forcing)
        {
            throw std::runtime_error("Accelerator base class cannot be directly used.");
        };
        virtual void fillMemory(float last_value)
        {
            throw std::runtime_error("Accelerator base class cannot be directly used.");
        };
    };

    template <size_t len>
    class SCurve : public Accelerator
    {
    protected:
        std::array<float, len> filter, memory;
        size_t last_mem_idx;

    public:
        SCurve()
        {
            this->memory.fill(0);
            this->last_mem_idx = 0;

            // Build a filter to convolve with the input
            float stepsize = 1.0f / len;
            float filter_sum = 0;

            for (size_t i = 0; i < len; i++)
            {
                if (i < len / 2)
                    this->filter[i] = i * stepsize;
                else
                    this->filter[i] = 1 - (i * stepsize);
                filter_sum += filter[i];
            }
            for (size_t i = 0; i < len; i++)
            {
                this->filter[i] /= filter_sum;
            }
        }
        float getNextValue(float forcing)
        {
            static float sum;
            static size_t i, mem_idx;

            // Append the memory with the new value
            this->memory[this->last_mem_idx++] = forcing;
            this->last_mem_idx %= len; // If we overflow, rotate back to the beginning

            sum = 0;
            // Convolve the filter coefficients with the memory
            for (i = 0; i < len; i++)
            {
                mem_idx = (i + last_mem_idx) % len;
                sum += filter[i] * memory[mem_idx];
            }

            return sum;
        }
        void fillMemory(float last_value)
        {
            this->memory.fill(last_value);
        }
    };

    class Holder : public Accelerator
    {
    protected:
        Accelerator *accel;
        float held;

    public:
        Holder(Accelerator *a) : accel(a)
        {
            held = 0;
        }
        virtual float getNextValue(float forcing)
        {
            if (forcing != 0)
            {
                held = accel->getNextValue(forcing);
                return held;
            }
            // Keep the child accelerator up to date
            accel->getNextValue(forcing);
            return held;
        };
        virtual void fillMemory(float last_value)
        {
            held = last_value;
            return accel->fillMemory(last_value);
        };
    };

    class DerivativeLimiter : public Accelerator
    {
    protected:
        const float limit_per_ms;
        float prev_val;
        long long prevtime;

    public:
        DerivativeLimiter(float tau_seconds) : limit_per_ms(1.0 / (1000.0 * tau_seconds))
        {
            prev_val = 0;
            prevtime = 0;
        }
        virtual float getNextValue(float forcing)
        {
            if (prevtime == 0)
            {
                prevtime = millis();
                prev_val = forcing;
                return forcing;
            }
            long long sample_time = millis();
            long long dt = sample_time - prevtime;
            prevtime = sample_time;

            // This is inverted to make the math below work without division
            float delta = forcing - prev_val;

            float rate = delta / dt;

            if (rate > this->limit_per_ms)
                prev_val += dt * this->limit_per_ms;
            else if (rate < -this->limit_per_ms)
                prev_val -= dt * this->limit_per_ms;
            else
                prev_val = forcing;

            return prev_val;
        };
        virtual void fillMemory(float last_value)
        {
            prev_val = last_value;
            prevtime = 0;
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
