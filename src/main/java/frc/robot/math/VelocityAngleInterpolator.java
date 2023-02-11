package frc.robot.math;

import java.util.List;

import edu.wpi.first.util.InterpolatingTreeMap;

/**
 * Table that will interpolate velocity, angle, and height based on distance.
 */
public class VelocityAngleInterpolator {

  private final InterpolatingTreeMap<Double, Double> distanceVelocityMap = new InterpolatingTreeMap<>();
  private final InterpolatingTreeMap<Double, Double> distanceAngleMap = new InterpolatingTreeMap<>();
  private final InterpolatingTreeMap<Double, Double> distanceHeightMap = new InterpolatingTreeMap<>();

  /**
   * Constructor that takes a list of settings that will be loaded to the table.
   * @param settingsList list of settings
   */
  public VelocityAngleInterpolator(List<ConeShooterSettings> settingsList) {
    for(ConeShooterSettings settings : settingsList) {
      distanceVelocityMap.put(settings.distance, settings.velocity);
      distanceAngleMap.put(settings.distance, settings.angle);
      distanceHeightMap.put(settings.distance, settings.height);
    }
  }

  /**
   * Calculates the shooter velocity, angle, and height by interpolating based on the distance.
   * @param distance distance to the target
   * @return shooter settings
   */
  public ConeShooterSettings calculate(double distance) {
    return ConeShooterSettings.shooterSettings(
        distance, distanceVelocityMap.get(distance), distanceAngleMap.get(distance), distanceHeightMap.get(distance));
  }

  /**
   * Shooter settings
   */
  public static class ConeShooterSettings {
    public final double distance;
    public final double velocity;
    public final double angle;
    public final double height;

    /**
     * Constructor
     * @param distance distance from target
     * @param velocity velocity of the shooter
     * @param angle angle of the wrist
     * @param height height of the elevator
     */
    public ConeShooterSettings(double distance, double velocity, double angle, double height) {
      this.distance = distance;
      this.velocity = velocity;
      this.angle = angle;
      this.height = height;
    }

    /**
     * Static factory method to make it more concise to create an instance
     * @param distance distance from target
     * @param velocity velocity of the shooter
     * @param wristAngle angle of the wrist
     * @param height height of the elevator
     * @return new instace with the provided values
     */
    public static ConeShooterSettings shooterSettings(double distance, double velocity, double wristAngle, double height) {
      return new ConeShooterSettings(distance, velocity, wristAngle, height);
    }
  }

}
