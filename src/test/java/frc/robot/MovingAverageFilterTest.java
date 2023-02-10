package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

public class MovingAverageFilterTest {

  @Test
  public void testAverage() {
    var filter = new MovingAverageFilter(5);

    assertEquals(4.0, filter.calculate(4.0));
    assertEquals(3.0, filter.calculate(2.0));
    assertEquals(3.0, filter.calculate(3.0));
    assertEquals(5.0, filter.calculate(11.0));
    assertEquals(5.0, filter.calculate(5.0));
    assertEquals(5.0, filter.calculate(4.0));

  }

  @Test
  public void testReset() {
    var filter = new MovingAverageFilter(5);

    for (double i : new double[] {14.0, 14.0, 14.0}) {
      filter.calculate(i);
    }
    assertEquals(14.0, filter.calculate(14.0));

    filter.reset();
    assertEquals(4.0, filter.calculate(4.0));
    assertEquals(3.0, filter.calculate(2.0));
    assertEquals(3.0, filter.calculate(3.0));
    assertEquals(5.0, filter.calculate(11.0));
    assertEquals(5.0, filter.calculate(5.0));
    assertEquals(5.0, filter.calculate(4.0));

  }
  
}
