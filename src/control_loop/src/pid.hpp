#ifndef PID_HPP
#define PID_HPP

#include <stdio.h>

#include <algorithm>
#include <functional>
#include <numeric>
#include <vector>

class pid
{
  private:
    const double _max;
    const double _min;
    const double _Kp;
    const double _Kd;
    const double _Ki;
    double       _pre_error;
    double       _integral;

  public:
    pid(double Kp, double Ki, double Kd, double max = 1, double min = -1)
        : _max(max),
          _min(min),
          _Kp(Kp),
          _Kd(Kd),
          _Ki(Ki),
          _pre_error(0),
          _integral(0)
    {
    }

    /**
     * @brief Run the PID control loop.
     *
     * @param
     */
    double tick(double current, double desired, double dt, bool print = false)
    {
        // Calculate error
        const double error = desired - current;

        // Proportional term
        const double Pout = _Kp * error;

        // Integral term
        _integral         += error * dt;
        const double Iout  = _Ki * _integral;
        // Also restrict integral to prevent growth after it saturates
        _integral          = std::max(_integral, _min);
        _integral          = std::min(_integral, _max);

        // Derivative term
        const double derivative = (error - _pre_error) / dt;
        const double Dout       = _Kd * derivative;

        // Calculate total output
        double output = Pout + Iout + Dout;

        if (print)
        {
            ROS_INFO(
                "e = %10.3f: P(%6.3f): %6.3f, I(%6.3f): %6.3f, D(%6.3f): %6.3f",
                error,
                _Kp,
                Pout,
                _Ki,
                Iout,
                _Kd,
                Dout);
        }

        // Restrict to max/min
        output = std::max(output, _min);
        output = std::min(output, _max);

        // Save error to previous error
        _pre_error = error;

        return output;
    }
};

// // class IIRFilter
// // {
// // private:
// //     const std::vector<double> a, b;
// //     std::vector<double> x_mem, y_mem;
// // public:
// //     IIRFilter(std::vector<double> b, std::vector<double> a = {})
// //     : a(a), b(b) {
// //         x_mem.resize(b.size(), 0);
// //         y_mem.resize(a.size(), 0);
// //     }
// //     double operator()(double x) {
// //         x_mem.insert(x_mem.begin(), x);
// //         x_mem.pop_back();
// //         double y = 0;
// //         for (int i = 0; i < b.size(); i++) {
// //             y += b[i]*x_mem[i];
// //         }
// //         for (int i = 0; i < a.size(); i++) {
// //             y -= a[i]*y_mem[i];
// //         }
// //         y_mem.insert(y_mem.begin(), y);
// //         y_mem.pop_back();
// //         return y;
// //     }
// // };

// class FiniteResponseIntegrator {
//  private:
//   std::vector<double> x_mem;
//   double prev_time;

//  public:
//   FiniteResponseIntegrator(size_t N) {
//     x_mem.resize(N, 0);
//     prev_time = 0;
//   }
//   double operator()(double x, double time_s) {
//     if (prev_time == 0) {
//       prev_time = time_s;
//       return 0;
//     }
//     double dt = time_s - prev_time;
//     prev_time = time_s;
//     x_mem.insert(x_mem.begin(), x * dt);
//     x_mem.pop_back();
//     return std::accumulate(x_mem.begin(), x_mem.end(), 0.0);
//   }
// };

// class NthOrderIntegrator {
//  private:
//   std::vector<FiniteResponseIntegrator> integrators;

//  public:
//   NthOrderIntegrator(size_t order, size_t N)
//   : integrators(order, {N}){}
//   double operator()(double x, double time_s) {
//     double y = x;
//     for (int i = 0; i < integrators.size(); i++) {
//       y = integrators[i](y, time_s);
//     }
//     return y;
//   }
// };

// void testIntegrators() {
//   printf("FiniteResponseIntegrator<3>:\n");
//   FiniteResponseIntegrator integrator(3);
//   double t = 0;
//   for (int i = 0; i < 10; i++) {
//     t += 0.1;
//     printf("%f\n", integrator(1, t));
//   }

//   printf("NthOrderIntegrator<2, 3>:\n");

//   NthOrderIntegrator integrator2(2, 3);
//   t = 0;
//   for (int i = 0; i < 10; i++) {
//     t += 0.1;
//     printf("%f\n", integrator2(1, t));
//   }
// }

#endif // end of PID_HPP
