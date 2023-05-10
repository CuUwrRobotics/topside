#ifndef MOTOR_MAPPER_MAIN
#define MOTOR_MAPPER_MAIN

/**
 * @brief A class implementing a filter which removes the DC component of a signal.
 *
 * The DC Blocker will remove the DC component of a signal. One class instance
 * must exist per signal.
 *
 * Example:
 *
 * ```cpp
 * DcBlockingFilter dc_filter(0.999);
 * double signal[N];
 *
 * // ... fill signal with data
 *
 * // Filter the signal
 * double filtered[N];
 * for (int i = 0; i < N; i++) {
 *   filtered[i] = dc_filter.filter(signal[i]);
 * }
 * ```
 *
 * The filter can be represented by the following difference equation:
 *
 * y[n] = x[n] - x[n-1] + r * y[n-1]
 *
 * where:
 * - x[n] is the input signal
 * - y[n] is the output signal
 * - r is the coefficient of the filter, defined in (0, 1)
 *
 * # Filter Coefficient
 *
 * A higher value of r will result in a less aggressive filter. More agressive
 * filters will remove DC with a smaller time constant, but will also attenuate
 * more at low frequencies. Common values for r range from 0.90 to 0.999.
 *
 * # Time Constant
 *
 * The time constant of the filter, in number of samples, is given by:
 *
 * tau = 1 / (1 - r)
 *
 * For example:
 * | r     | tau (samples) |
 * |-------|---------------|
 * | 0.999 | 1000          |
 * | 0.99  | 100           |
 * | 0.9   | 10            |
 * | 0.5   | 2             |
 * | 0.1   | 1.111         |
 *
 * # More Info
 *
 * For more information, see:
 *
 * - Practical info, tests, and implementation: https://ccrma.stanford.edu/~jos/fp/DC_Blocker.html
 * - Theory: http://dspguru.com/dsp/tricks/fixed-point-dc-blocking-filter-with-noise-shaping/
 */
class DcBlockingFilter
{
private:
  double r;
  double prev_x, prev_y;

public:
  /**
   * @brief Construct a new DC Blocking Filter object.
   *
   * @param r_val The Coefficient of the filter. See class description for more info.
   */
  DcBlockingFilter(double r_val)
  {
    this->r = r_val;
    prev_x = 0;
    prev_y = 0;
  }
  /**
   * @brief Filter a single sample.
   *
   * @param x The input sample.
   * @return double The filtered sample.
   */
  double filter(double x)
  {
    double y = x - prev_x + r * prev_y;
    prev_x = x;
    prev_y = y;
    return y;
  }
};

#endif /* end of include guard: MOTOR_MAPPER_MAIN */
