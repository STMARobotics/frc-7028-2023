// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.math.util.Units.degreesToRadians;
import static edu.wpi.first.math.util.Units.inchesToMeters;
import static frc.robot.math.VelocityAngleInterpolator.ConeShooterSettings.shooterSettings;
import static java.lang.Math.PI;
import static java.lang.Math.toRadians;

import java.util.List;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.math.VelocityAngleInterpolator;
import frc.robot.swerve.ModuleConfiguration;

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
  
  public static final class DrivetrainConstants {

    public static final boolean ADD_TO_DASHBOARD = true;

    /**
     * The left-to-right distance between the drivetrain wheels
     *
     * Should be measured from center to center.
     */
    public static final double DRIVETRAIN_TRACKWIDTH_METERS = inchesToMeters(18.75);
    /**
     * The front-to-back distance between the drivetrain wheels.
     *
     * Should be measured from center to center.
     */
    public static final double DRIVETRAIN_WHEELBASE_METERS = inchesToMeters(18.75);

    public static final String CANIVORE_BUS_NAME = "swerve";

    public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 3;
    public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 13;
    public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 23;
    public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -toRadians(286.962890625);

    public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 2;
    public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 12;
    public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 22;
    public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -toRadians(41.220703125);

    public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 1;
    public static final int BACK_LEFT_MODULE_STEER_MOTOR = 11;
    public static final int BACK_LEFT_MODULE_STEER_ENCODER = 21;
    public static final double BACK_LEFT_MODULE_STEER_OFFSET = -toRadians(68.90625);

    public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 0;
    public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 10;
    public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 20;
    public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -toRadians(66.4453125);
    
    public static final int PIGEON_ID = 30;

    /**
     * The maximum velocity of the robot in meters per second.
     * <p>
     * This is a measure of how fast the robot should be able to drive in a straight line.
     */
    public static final double MAX_VELOCITY_METERS_PER_SECOND = 6380.0 / 60.0 *
        ModuleConfiguration.MK4I_L2.getDriveReduction() *
        ModuleConfiguration.MK4I_L2.getWheelDiameter() * PI;

     /**
     * The maximum angular velocity of the robot in radians per second.
     * <p>
     * This is a measure of how fast the robot can rotate in place.
     */
    // Here we calculate the theoretical maximum angular velocity. You can also replace this with a measured amount.
    public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = 
        (DrivetrainConstants.MAX_VELOCITY_METERS_PER_SECOND /
        Math.hypot(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0));

    public static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(
        // Front left
        new Translation2d(DRIVETRAIN_WHEELBASE_METERS / 2.0, DRIVETRAIN_TRACKWIDTH_METERS / 2.0),
        // Front right
        new Translation2d(DRIVETRAIN_WHEELBASE_METERS / 2.0, -DRIVETRAIN_TRACKWIDTH_METERS / 2.0),
        // Back left
        new Translation2d(-DRIVETRAIN_WHEELBASE_METERS / 2.0, DRIVETRAIN_TRACKWIDTH_METERS / 2.0),
        // Back right
        new Translation2d(-DRIVETRAIN_WHEELBASE_METERS / 2.0, -DRIVETRAIN_TRACKWIDTH_METERS / 2.0)
    );

    /** Voltage needed to overcome the motor’s static friction. kS */
    public static final double DRIVE_kS = 0.18;
    /** Voltage needed to hold (or "cruise") at a given constant velocity. kV */
    public static final double DRIVE_kV = 2.300;
    /** Voltage needed to induce a given acceleration in the motor shaft. kA */
    public static final double DRIVE_kA = 0.52878;

    public static final double STEER_kP = 0.2;
    public static final double STEER_kI = 0.001;
    public static final double STEER_kD = 0.0;

    public static final double DRIVE_kP = 0.04;
    public static final double DRIVE_kI = 0.0;
    public static final double DRIVE_kD = 0.0;

  }

  public static final class TeleopDriveConstants {

    public static final double XBOX_CONTROLLER_DEADBAND = 0.1;
    public static final double JOYSTICK_DEADBAND = 0.1;

    public static final double X_RATE_LIMIT = 6.0;
    public static final double Y_RATE_LIMIT = 6.0;
    public static final double ROTATION_RATE_LIMIT = 5.0 * PI;

    public static final double HEADING_MAX_VELOCITY = PI * 4;
    public static final double HEADING_MAX_ACCELERATION = PI * 16;
    
    public static final double HEADING_kP = 2.0;
    public static final double HEADING_kI = 0.0;
    public static final double HEADING_kD = 0.0;

    public static final double HEADING_TOLERANCE = degreesToRadians(1.5);

  }

  public static class VisionConstants {

    /** Physical location of the apriltag camera on the robot, relative to the center of the robot. */
    public static final Transform3d APRILTAG_CAMERA_TO_ROBOT =
        new Transform3d(new Translation3d(-0.06, 0.250, -0.2127), new Rotation3d(0.0, degreesToRadians(15.0), 0.0));

    /** Physical location of the shooter camera on the robot, relative to the center of the robot. */
    public static final Transform3d LOW_LIMELIGHT_TO_ROBOT = new Transform3d(
        new Translation3d(-0.083, 0.254, -0.537),
        new Rotation3d(0.0, degreesToRadians(-9.8), degreesToRadians(-1.0)));

    public static final String LOW_LIMELIGHT_NAME = "limelight";
    
    /** Physical location of the high camera on the robot, relative to the center of the robot. */
    public static final Transform3d HIGH_LIMELIGHT_TO_ROBOT = new Transform3d(
        new Translation3d(-0.11, -0.015, -0.895),
        new Rotation3d(degreesToRadians(-90.0), degreesToRadians(34.6), 0.0));

    public static final String HIGH_LIMELIGHT_NAME = "limelight-high";
    
    public static final double FIELD_LENGTH_METERS = 16.54175;
    public static final double FIELD_WIDTH_METERS = 8.0137;
  }

  public static class AutoConstants {
    public static final TrapezoidProfile.Constraints THETA_CONSTRAINTS = new TrapezoidProfile.Constraints(PI, 2 / PI);
    public static final double THETA_kP = 3.1;
    public static final double THETA_kI = 0.0;
    public static final double THETA_kD = 0.0;

    public static final double X_kP = 5.0;
    public static final double X_kI = 0.0;
    public static final double X_kD = 0.0;

    public static final double Y_kP = 5.0;
    public static final double Y_kI = 0.0;
    public static final double Y_kD = 0.0;
  }
  
  public static class ElevatorConstants {
    public static final int ELEVATOR_LEADER_ID = 40;
    public static final int ELEVATOR_FOLLOWER_ID = 41;
    public static final int ANALOG_SENSOR_CHANNEL = 0;
    public static final double ELEVATOR_PARK_HEIGHT = .06;

  }

  public static class WristConstants {
    public static final int WRIST_LEADER_ID = 3;
    public static final double WRIST_PARK_HEIGHT = 1.4;
  }

  public static class ShooterConstants {
    public static final int SHOOTER_LEADER_ID = 5;
    public static final int SHOOTER_FOLLOWER_ID = 6;
  }

  public static class ConeShootingConstants {
    public static final double SHOOT_TIME = 0.5;

    public static final VelocityAngleInterpolator TOP_TABLE = new VelocityAngleInterpolator(List.of(
      shooterSettings(1.25, 28.0, 0.8, 0.95),
      shooterSettings(1.45, 29.5, 0.8, 0.95)
    ));

    public static final VelocityAngleInterpolator MIDDLE_TABLE = new VelocityAngleInterpolator(List.of(
      shooterSettings(0.8, 25.5, 0.83, 0.45),
      shooterSettings(1.3, 27.25, 0.83, 0.45)
    ));

    public static final VelocityAngleInterpolator LOW_TABLE = new VelocityAngleInterpolator(List.of(
      shooterSettings(1.0, 15.0, 0.0, 0.1)
    ));

  }

}
