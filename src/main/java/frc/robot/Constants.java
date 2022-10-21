// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.math.util.Units.inchesToMeters;
import static java.lang.Math.toRadians;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  /**
   * The left-to-right distance between the drivetrain wheels
   *
   * Should be measured from center to center.
   */
  public static final double DRIVETRAIN_TRACKWIDTH_METERS = inchesToMeters(20.5);
  /**
   * The front-to-back distance between the drivetrain wheels.
   *
   * Should be measured from center to center.
   */
  public static final double DRIVETRAIN_WHEELBASE_METERS = inchesToMeters(20.5);

  public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 0;
  public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 10;
  public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 20;
  public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -toRadians(74.8828125);

  public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 1;
  public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 11;
  public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 21;
  public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -toRadians(104.765625);

  public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 2;
  public static final int BACK_LEFT_MODULE_STEER_MOTOR = 12;
  public static final int BACK_LEFT_MODULE_STEER_ENCODER = 22;
  public static final double BACK_LEFT_MODULE_STEER_OFFSET = -toRadians(162.158203125);

  public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 3;
  public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 13;
  public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 23;
  public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -toRadians(74.70703125);

  public static class VisionConstants {

    /**
     * Physical location of the camera on the robot, relative to the center of the robot.
     */
    public static final Transform2d CAMERA_TO_ROBOT = 
        new Transform2d(new Translation2d(-0.3425, 0.0), new Rotation2d(0.0));
    public static final Transform3d CAMERA_TO_ROBOT_3D =
        new Transform3d(new Translation3d(-0.3425, 0.0, -0.233), new Rotation3d());
  }
}
