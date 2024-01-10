package frc.robot.math;

import edu.wpi.first.util.CircularBuffer;

/**
 * A moving average filter that accurately calculates averages, even before 'taps' number of samples are provided.
 */
public class MovingAverageFilter {
  private final int taps;
  private CircularBuffer<Double> m_inputs = new CircularBuffer<Double>(0);

  public MovingAverageFilter(int taps) {
    this.taps = taps;
  }

  public void reset() {
    m_inputs.resize(0);
  }

  public double calculate(double value) {
    if (m_inputs.size() < taps) {
      m_inputs.resize(m_inputs.size() + 1);
    } 
    m_inputs.addFirst(value);
    double returnValue = 0;
    for (int i = 0; i < m_inputs.size(); i++) {
      returnValue += m_inputs.get(i);
    }
    return returnValue / m_inputs.size();
  }

}
