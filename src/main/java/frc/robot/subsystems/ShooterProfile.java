package frc.robot.subsystems;

import static frc.robot.Constants.ConeShootingConstants.LOW_TABLE;
import static frc.robot.Constants.ConeShootingConstants.MIDDLE_TABLE;
import static frc.robot.Constants.ConeShootingConstants.TOP_TABLE;

import edu.wpi.first.math.util.Units;
import frc.robot.math.VelocityAngleInterpolator;

/**
 * Shooter profiles enum with shootings for scoring
 */
public enum ShooterProfile {

  /** Score a cone on a top node */
  SCORE_CONE_TOP(TOP_TABLE, 1.35, Units.degreesToRadians(1.0)),

  /** Score a cone on a middle node */
  SCORE_CONE_MIDDLE(MIDDLE_TABLE, 1.0, Units.degreesToRadians(1.5)),

  /** Score a cone in a low node */
  SCORE_CONE_LOW(LOW_TABLE, 1.1, Units.degreesToRadians(1.0));

  /** Lookup table for shooter velocity, angle, and height */
  public final VelocityAngleInterpolator lookupTable;

  /** Preferred shooting distance */
  public final double shootingDistance;

  public final double aimTolerance;

  /**
   * Create a shooter profile
   * @param lookupTable lookup table with shooter settings
   * @param shootingDistance distance to shoot from
   */
  private ShooterProfile(VelocityAngleInterpolator lookupTable, double shootingDistance, double aimTolerance) {
    this.lookupTable = lookupTable;
    this.shootingDistance = shootingDistance;
    this.aimTolerance = aimTolerance;
  }

}