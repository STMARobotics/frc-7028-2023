// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.DrivetrainConstants.BACK_LEFT_MODULE_DRIVE_MOTOR;
import static frc.robot.Constants.DrivetrainConstants.BACK_LEFT_MODULE_STEER_ENCODER;
import static frc.robot.Constants.DrivetrainConstants.BACK_LEFT_MODULE_STEER_MOTOR;
import static frc.robot.Constants.DrivetrainConstants.BACK_LEFT_MODULE_STEER_OFFSET;
import static frc.robot.Constants.DrivetrainConstants.BACK_RIGHT_MODULE_DRIVE_MOTOR;
import static frc.robot.Constants.DrivetrainConstants.BACK_RIGHT_MODULE_STEER_ENCODER;
import static frc.robot.Constants.DrivetrainConstants.BACK_RIGHT_MODULE_STEER_MOTOR;
import static frc.robot.Constants.DrivetrainConstants.BACK_RIGHT_MODULE_STEER_OFFSET;
import static frc.robot.Constants.DrivetrainConstants.DRIVETRAIN_TRACKWIDTH_METERS;
import static frc.robot.Constants.DrivetrainConstants.DRIVETRAIN_WHEELBASE_METERS;
import static frc.robot.Constants.DrivetrainConstants.FRONT_LEFT_MODULE_DRIVE_MOTOR;
import static frc.robot.Constants.DrivetrainConstants.FRONT_LEFT_MODULE_STEER_ENCODER;
import static frc.robot.Constants.DrivetrainConstants.FRONT_LEFT_MODULE_STEER_MOTOR;
import static frc.robot.Constants.DrivetrainConstants.FRONT_LEFT_MODULE_STEER_OFFSET;
import static frc.robot.Constants.DrivetrainConstants.FRONT_RIGHT_MODULE_DRIVE_MOTOR;
import static frc.robot.Constants.DrivetrainConstants.FRONT_RIGHT_MODULE_STEER_ENCODER;
import static frc.robot.Constants.DrivetrainConstants.FRONT_RIGHT_MODULE_STEER_MOTOR;
import static frc.robot.Constants.DrivetrainConstants.FRONT_RIGHT_MODULE_STEER_OFFSET;
import static frc.robot.Constants.DrivetrainConstants.PIGEON_ID;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
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
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.swerve.DrivetrainState;
import frc.robot.swerve.Falcon500SwerveModule;
import frc.robot.swerve.Mk4SwerveModuleFactory;
import frc.robot.swerve.ModuleConfiguration;

public class DrivetrainSubsystem extends SubsystemBase {
  /**
   * The maximum voltage that will be delivered to the drive motors.
   * <p>
   * This can be reduced to cap the robot's maximum speed. Typically, this is useful during initial testing of the robot.
   */
  public static final double MAX_VOLTAGE = 12.0;
  //  The formula for calculating the theoretical maximum velocity is:
  //   <Motor free speed RPM> / 60 * <Drive reduction> * <Wheel diameter meters> * pi

  /**
   * The maximum velocity of the robot in meters per second.
   * <p>
   * This is a measure of how fast the robot should be able to drive in a straight line.
   */
  public static final double MAX_VELOCITY_METERS_PER_SECOND = 6380.0 / 60.0 *
      ModuleConfiguration.MK4_L1.getDriveReduction() *
      ModuleConfiguration.MK4_L1.getWheelDiameter() * Math.PI;
  
  /**
   * The maximum angular velocity of the robot in radians per second.
   * <p>
   * This is a measure of how fast the robot can rotate in place.
   */
  // Here we calculate the theoretical maximum angular velocity. You can also replace this with a measured amount.
  public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = 
          (MAX_VELOCITY_METERS_PER_SECOND /
          Math.hypot(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0));

  public static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(
          // Front left
          new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
          // Front right
          new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0),
          // Back left
          new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
          // Back right
          new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0)
  );

  // The important thing about how you configure your gyroscope is that rotating the robot counter-clockwise should
  // cause the angle reading to increase until it wraps back over to zero.
  private final WPI_Pigeon2 pigeon = new WPI_Pigeon2(PIGEON_ID);
  // private final AHRS navx = new AHRS(SPI.Port.kMXP, (byte) 200); // NavX connected over MXP

  // These are our modules. We initialize them in the constructor.
  private final Falcon500SwerveModule frontLeftModule;
  private final Falcon500SwerveModule frontRightModule;
  private final Falcon500SwerveModule backLeftModule;
  private final Falcon500SwerveModule backRightModule;

  private ChassisSpeeds chassisSpeeds;
  private DrivetrainState currentDrivetrainState;

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
    // We use Falcon 500s in L1 configuration.
    frontLeftModule = Mk4SwerveModuleFactory.createFalcon500(
        // This parameter is optional, but will allow you to see the current state of the module on the dashboard.
        frontLeftLayout,
        // This can either be L1, L2, L3, L4 depending on your gear configuration
        ModuleConfiguration.MK4_L1,
        // This is the ID of the drive motor
        FRONT_LEFT_MODULE_DRIVE_MOTOR,
        // This is the ID of the steer motor
        FRONT_LEFT_MODULE_STEER_MOTOR,
        // This is the ID of the steer encoder
        FRONT_LEFT_MODULE_STEER_ENCODER,
        // This is how much the steer encoder is offset from true zero (In our case, zero is facing straight forward)
        FRONT_LEFT_MODULE_STEER_OFFSET
    );

    // We will do the same for the other modules
    frontRightModule = Mk4SwerveModuleFactory.createFalcon500(
        frontRightLayout,
        ModuleConfiguration.MK4_L1,
        FRONT_RIGHT_MODULE_DRIVE_MOTOR,
        FRONT_RIGHT_MODULE_STEER_MOTOR,
        FRONT_RIGHT_MODULE_STEER_ENCODER,
        FRONT_RIGHT_MODULE_STEER_OFFSET
    );

    backLeftModule = Mk4SwerveModuleFactory.createFalcon500(
        backLeftLayout,
        ModuleConfiguration.MK4_L1,
        BACK_LEFT_MODULE_DRIVE_MOTOR,
        BACK_LEFT_MODULE_STEER_MOTOR,
        BACK_LEFT_MODULE_STEER_ENCODER,
        BACK_LEFT_MODULE_STEER_OFFSET
    );

    backRightModule = Mk4SwerveModuleFactory.createFalcon500(
        backRightLayout,
        ModuleConfiguration.MK4_L1,
        BACK_RIGHT_MODULE_DRIVE_MOTOR,
        BACK_RIGHT_MODULE_STEER_MOTOR,
        BACK_RIGHT_MODULE_STEER_ENCODER,
        BACK_RIGHT_MODULE_STEER_OFFSET
    );

    this.currentDrivetrainState = 
        new DrivetrainState(frontLeftModule, frontRightModule, backLeftModule, backRightModule);

    new Trigger(RobotState::isEnabled).onTrue(new StartEndCommand(() -> {
      frontLeftModule.setNeutralMode(NeutralMode.Brake);
      frontRightModule.setNeutralMode(NeutralMode.Brake);
      backLeftModule.setNeutralMode(NeutralMode.Brake);
      backRightModule.setNeutralMode(NeutralMode.Brake);
    }, () -> {
      frontLeftModule.setNeutralMode(NeutralMode.Coast);
      frontRightModule.setNeutralMode(NeutralMode.Coast);
      backLeftModule.setNeutralMode(NeutralMode.Coast);
      backRightModule.setNeutralMode(NeutralMode.Coast);
    }));
  }

  public Rotation2d getGyroscopeRotation() {
    return pigeon.getRotation2d();

    // We have to invert the angle of the NavX so that rotating the robot counter-clockwise makes the angle increase.
    // return Rotation2d.fromDegrees(360.0 - navx.getYaw());
  }

  public void drive(ChassisSpeeds chassisSpeeds) {
    this.chassisSpeeds = chassisSpeeds;
  }

  public void stop() {
    chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);
  }

  @Override
  public void periodic() {
    // Read the current states - only once per iteration
    currentDrivetrainState = new DrivetrainState(frontLeftModule, frontRightModule, backLeftModule, backRightModule);

    // Set the swerve module states
    if (chassisSpeeds != null) {
      var states = KINEMATICS.toSwerveModuleStates(chassisSpeeds);
      if(chassisSpeeds.vxMetersPerSecond == 0.0 && chassisSpeeds.vyMetersPerSecond == 0.0
          && chassisSpeeds.omegaRadiansPerSecond == 0.0) {
        // Keep the wheels at their current angle when stopped, don't snap back to straight
        var currentStates = currentDrivetrainState.getSwerveModuleStates();
        for(int i = 0; i < currentStates.length; i++) {
          states[i].angle = currentStates[i].angle;
        }
      }

      SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_VELOCITY_METERS_PER_SECOND);
      setModuleStates(states);
    }
    chassisSpeeds = null;
  }

  /**
   * Gets the current drivetrain state (position, velocity, and angle), as reported by the modules themselves.
   * @return current drivetrain state. Array orders are frontLeft, frontRight, backLeft, backRight
   */
  public DrivetrainState getDrivetrainState() {
    return currentDrivetrainState;
  }

  /**
   * Sets the states of the modules.
   * @param states array of states. Must be ordered frontLeft, frontRight, backLeft, backRight
   */
  private void setModuleStates(SwerveModuleState[] states) {
    setModuleState(frontLeftModule, states[0]);
    setModuleState(frontRightModule, states[1]);
    setModuleState(backLeftModule, states[2]);
    setModuleState(backRightModule, states[3]);
  }

  private static void setModuleState(Falcon500SwerveModule module, SwerveModuleState state) {
    module.set(state.speedMetersPerSecond, state.angle.getRadians());
  }

  public SwerveDriveKinematics getKinematics() {
    return KINEMATICS;
  }

  public void resetDriveEncoders() {
    frontLeftModule.resetDriveEncoders();
    frontRightModule.resetDriveEncoders();
    backLeftModule.resetDriveEncoders();
    backRightModule.resetDriveEncoders();
  }

  /**
   * Creates a command to follow a Trajectory on the drivetrain.
   * @param trajectory trajectory to follow
   * @return command that will run the trajectory
   */
  public Command createCommandForTrajectory(Trajectory trajectory, PoseEstimatorSubsystem poseEstimator) {

    TrapezoidProfile.Constraints kThetaControllerConstraints = 
        new TrapezoidProfile.Constraints(Math.PI, 2 / Math.PI);

    var thetaController = new ProfiledPIDController(5, 0, 0, kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand =
        new SwerveControllerCommand(
          trajectory,
          poseEstimator::getCurrentPose,
          KINEMATICS,
          new PIDController(16, 0, 0),
          new PIDController(16, 0, 0),
          thetaController,
          this::setModuleStates,
          this);
            
      return swerveControllerCommand;
  }
}
