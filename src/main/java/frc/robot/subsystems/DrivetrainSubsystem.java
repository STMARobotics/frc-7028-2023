// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.AutoConstants.THETA_CONSTRAINTS;
import static frc.robot.Constants.DrivetrainConstants.BACK_LEFT_MODULE_DRIVE_MOTOR;
import static frc.robot.Constants.DrivetrainConstants.BACK_LEFT_MODULE_STEER_ENCODER;
import static frc.robot.Constants.DrivetrainConstants.BACK_LEFT_MODULE_STEER_MOTOR;
import static frc.robot.Constants.DrivetrainConstants.BACK_LEFT_MODULE_STEER_OFFSET;
import static frc.robot.Constants.DrivetrainConstants.BACK_RIGHT_MODULE_DRIVE_MOTOR;
import static frc.robot.Constants.DrivetrainConstants.BACK_RIGHT_MODULE_STEER_ENCODER;
import static frc.robot.Constants.DrivetrainConstants.BACK_RIGHT_MODULE_STEER_MOTOR;
import static frc.robot.Constants.DrivetrainConstants.BACK_RIGHT_MODULE_STEER_OFFSET;
import static frc.robot.Constants.DrivetrainConstants.FRONT_LEFT_MODULE_DRIVE_MOTOR;
import static frc.robot.Constants.DrivetrainConstants.FRONT_LEFT_MODULE_STEER_ENCODER;
import static frc.robot.Constants.DrivetrainConstants.FRONT_LEFT_MODULE_STEER_MOTOR;
import static frc.robot.Constants.DrivetrainConstants.FRONT_LEFT_MODULE_STEER_OFFSET;
import static frc.robot.Constants.DrivetrainConstants.FRONT_RIGHT_MODULE_DRIVE_MOTOR;
import static frc.robot.Constants.DrivetrainConstants.FRONT_RIGHT_MODULE_STEER_ENCODER;
import static frc.robot.Constants.DrivetrainConstants.FRONT_RIGHT_MODULE_STEER_MOTOR;
import static frc.robot.Constants.DrivetrainConstants.FRONT_RIGHT_MODULE_STEER_OFFSET;
import static frc.robot.Constants.DrivetrainConstants.PIGEON_ID;

import java.util.Arrays;
import java.util.function.Supplier;
import java.util.stream.IntStream;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.swerve.ModuleConfiguration;
import frc.robot.swerve.SwerveModule;
import frc.robot.swerve.SwerveSpeedController;
import frc.robot.swerve.SwerveSteerController;

public class DrivetrainSubsystem extends SubsystemBase {

  private final WPI_Pigeon2 pigeon = new WPI_Pigeon2(PIGEON_ID);
  // private final AHRS navx = new AHRS(SPI.Port.kMXP, (byte) 200); // NavX connected over MXP
  private final SwerveModule[] swerveModules;

  private ChassisSpeeds desiredChassisSpeeds;

  public DrivetrainSubsystem() {
    ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");
    pigeon.configMountPoseRoll(90);
    pigeon.configMountPoseYaw(-90);

    ShuffleboardLayout frontLeftLayout = null;
    ShuffleboardLayout frontRightLayout = null;
    ShuffleboardLayout backLeftLayout = null;
    ShuffleboardLayout backRightLayout = null;

    if (DrivetrainConstants.ADD_TO_DASHBOARD) {
      frontLeftLayout = tab.getLayout("Front Left Module", BuiltInLayouts.kList)
          .withSize(2, 4)
          .withPosition(0, 0);

      frontRightLayout = tab.getLayout("Front Right Module", BuiltInLayouts.kList)
          .withSize(2, 4)
          .withPosition(2, 0);
      
      backLeftLayout =tab.getLayout("Back Left Module", BuiltInLayouts.kList)
          .withSize(2, 4)
          .withPosition(4, 0);
      
      backRightLayout = tab.getLayout("Back Right Module", BuiltInLayouts.kList)
          .withSize(2, 4)
          .withPosition(6, 0);
    }

    swerveModules = new SwerveModule[] {
        createSwerveModule(
            frontLeftLayout,
            ModuleConfiguration.MK4_L1,
            FRONT_LEFT_MODULE_DRIVE_MOTOR,
            FRONT_LEFT_MODULE_STEER_MOTOR,
            FRONT_LEFT_MODULE_STEER_ENCODER,
            FRONT_LEFT_MODULE_STEER_OFFSET
        ),
        createSwerveModule(
            frontRightLayout,
            ModuleConfiguration.MK4_L1,
            FRONT_RIGHT_MODULE_DRIVE_MOTOR,
            FRONT_RIGHT_MODULE_STEER_MOTOR,
            FRONT_RIGHT_MODULE_STEER_ENCODER,
            FRONT_RIGHT_MODULE_STEER_OFFSET
        ),
        createSwerveModule(
            backLeftLayout,
            ModuleConfiguration.MK4_L1,
            BACK_LEFT_MODULE_DRIVE_MOTOR,
            BACK_LEFT_MODULE_STEER_MOTOR,
            BACK_LEFT_MODULE_STEER_ENCODER,
            BACK_LEFT_MODULE_STEER_OFFSET
        ),
        createSwerveModule(
            backRightLayout,
            ModuleConfiguration.MK4_L1,
            BACK_RIGHT_MODULE_DRIVE_MOTOR,
            BACK_RIGHT_MODULE_STEER_MOTOR,
            BACK_RIGHT_MODULE_STEER_ENCODER,
            BACK_RIGHT_MODULE_STEER_OFFSET
        )};

    // Put the motors in brake mode when enabled, coast mode when disabled
    new Trigger(RobotState::isEnabled).onTrue(new StartEndCommand(() -> {
      for (SwerveModule swerveModule : swerveModules) {
        swerveModule.setNeutralMode(NeutralMode.Brake);
      }
    }, () -> {
      for (SwerveModule swerveModule : swerveModules) {
        swerveModule.setNeutralMode(NeutralMode.Coast);
      }
    }));
  }

  /**
   * Creates a server module instance
   * @param container shuffleboard layout, or null
   * @param moduleConfiguration module configuration
   * @param driveMotorPort drive motor CAN ID
   * @param steerMotorPort steer motor CAN ID
   * @param steerEncoderPort steer encoder CAN ID
   * @param steerOffset offset for steer encoder
   * @return new swerve module instance
   */
  private static SwerveModule createSwerveModule(
      ShuffleboardLayout container,
      ModuleConfiguration moduleConfiguration,
      int driveMotorPort,
      int steerMotorPort,
      int steerEncoderPort,
      double steerOffset) {

    return new SwerveModule(
        new SwerveSpeedController(driveMotorPort, moduleConfiguration, container), 
        new SwerveSteerController(steerMotorPort, steerEncoderPort, steerOffset, container, moduleConfiguration));
  }

  public Rotation2d getGyroscopeRotation() {
    return pigeon.getRotation2d();

    // We have to invert the angle of the NavX so that rotating the robot counter-clockwise makes the angle increase.
    // return Rotation2d.fromDegrees(360.0 - navx.getYaw());
  }

  public void drive(ChassisSpeeds chassisSpeeds) {
    desiredChassisSpeeds = chassisSpeeds;
  }

  public void stop() {
    desiredChassisSpeeds = new ChassisSpeeds();
  }

  @Override
  public void periodic() {
    // Set the swerve module states
    if (desiredChassisSpeeds != null) {
      var currentStates = getModuleStates();
      var desiredStates = DrivetrainConstants.KINEMATICS.toSwerveModuleStates(desiredChassisSpeeds);

      if(desiredChassisSpeeds.vxMetersPerSecond == 0.0 && desiredChassisSpeeds.vyMetersPerSecond == 0.0
          && desiredChassisSpeeds.omegaRadiansPerSecond == 0.0) {
        // Keep the wheels at their current angle when stopped, don't snap back to straight
        IntStream.range(0, currentStates.length).forEach(i -> desiredStates[i].angle = currentStates[i].angle);
      }

      SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DrivetrainConstants.MAX_VELOCITY_METERS_PER_SECOND);
      setModuleStates(desiredStates);
    }
    desiredChassisSpeeds = null;
  }

  /**
   * Gets the current drivetrain state (velocity, and angle), as reported by the modules themselves.
   * @return current drivetrain state. Array orders are frontLeft, frontRight, backLeft, backRight
   */
  public SwerveModuleState[] getModuleStates() {
    return Arrays.stream(swerveModules).map(module -> module.getState()).toArray(SwerveModuleState[]::new);
  }

  /**
   * Gets the current drivetrain position, as reported by the modules themselves.
   * @return current drivetrain state. Array orders are frontLeft, frontRight, backLeft, backRight
   */
  public SwerveModulePosition[] getModulePositions() {
    return Arrays.stream(swerveModules).map(module -> module.getPosition()).toArray(SwerveModulePosition[]::new);
  }

  /**
   * Sets the states of the modules.
   * @param states array of states. Must be ordered frontLeft, frontRight, backLeft, backRight
   */
  private void setModuleStates(SwerveModuleState[] states) {
    IntStream.range(0, swerveModules.length).forEach(i -> swerveModules[i].setDesiredState(states[i]));
  }

  /**
   * Creates a command to follow a Trajectory on the drivetrain.
   * @param trajectory trajectory to follow
   * @return command that will run the trajectory
   */
  public Command createCommandForTrajectory(Trajectory trajectory, Supplier<Pose2d> poseSupplier) {
    var thetaController = new ProfiledPIDController(
        AutoConstants.THETA_kP, AutoConstants.THETA_kI, AutoConstants.THETA_kD, THETA_CONSTRAINTS);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand =
        new SwerveControllerCommand(
          trajectory,
          poseSupplier,
          DrivetrainConstants.KINEMATICS,
          new PIDController(AutoConstants.X_kP, AutoConstants.X_kI, AutoConstants.X_kD),
          new PIDController(AutoConstants.Y_kP, AutoConstants.Y_kI, AutoConstants.Y_kD),
          thetaController,
          this::setModuleStates,
          this);
            
      return swerveControllerCommand;
  }
}
