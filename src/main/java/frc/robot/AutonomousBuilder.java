package frc.robot;

import static edu.wpi.first.wpilibj2.command.Commands.runOnce;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.HashMap;
import java.util.stream.Stream;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.PickupConstants;
import frc.robot.commands.AutoPickupCommand;
import frc.robot.commands.DefaultLEDCommand;
import frc.robot.commands.JustShootCommand;
import frc.robot.commands.LEDCustomCommand;
import frc.robot.commands.ShootConeCommand;
import frc.robot.commands.autonomous.BalanceCommand;
import frc.robot.commands.autonomous.DriveToPoseCommand;
import frc.robot.commands.autonomous.TransitCommand;
import frc.robot.limelight.LimelightProfile;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import frc.robot.subsystems.ShooterProfile;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.WristSubsystem;

public class AutonomousBuilder {
  private final DrivetrainSubsystem drivetrainSubsystem;
  private final ElevatorSubsystem elevatorSubsystem;
  private final LEDSubsystem ledSubsystem;
  private final ShooterSubsystem shooterSubsystem;
  private final WristSubsystem wristSubsystem;
  private final LimelightSubsystem highLimelightSubsystem;
  private final LimelightSubsystem lowLimelightSubsystem;
  private final PoseEstimatorSubsystem poseEstimator;

  private final SendableChooser<Command> autoChooser = new SendableChooser<>();
  private final SwerveAutoBuilder swerveAutoBuilder;

  public AutonomousBuilder(DrivetrainSubsystem drivetrainSubsystem, ElevatorSubsystem elevatorSubsystem,
      LEDSubsystem ledSubsystem, ShooterSubsystem shooterSubsystem, WristSubsystem wristSubsystem,
      LimelightSubsystem lowLimelightSubsystem, LimelightSubsystem highLimelightSubsystem,
      PoseEstimatorSubsystem poseEstimator) {
    this.drivetrainSubsystem = drivetrainSubsystem;
    this.elevatorSubsystem = elevatorSubsystem;
    this.ledSubsystem = ledSubsystem;
    this.shooterSubsystem = shooterSubsystem;
    this.wristSubsystem = wristSubsystem;
    this.highLimelightSubsystem = highLimelightSubsystem;
    this.lowLimelightSubsystem = lowLimelightSubsystem;
    this.poseEstimator = poseEstimator;

    var eventMap = buildEventMap();
    swerveAutoBuilder = new AutoBuilder(
        poseEstimator::getCurrentPose,
        poseEstimator::setCurrentPose,
        DrivetrainConstants.KINEMATICS,
        new PIDConstants(AutoConstants.X_kP, AutoConstants.X_kI, AutoConstants.X_kD),
        new PIDConstants(AutoConstants.PATH_THETA_kP, AutoConstants.PATH_THETA_kI, AutoConstants.PATH_THETA_kD),
        drivetrainSubsystem::setModuleStates,
        eventMap,
        true,
        new Command[] {
          new DefaultLEDCommand(ledSubsystem, shooterSubsystem::hasCone, shooterSubsystem::hasCube)
        },
        drivetrainSubsystem
    );

    autoChooser.setDefaultOption("None", Commands.none());
    // Add all the paths in the pathplanner directory
    try (Stream<Path> files = Files.list(Paths.get(Filesystem.getDeployDirectory().getAbsolutePath(), "pathplanner"))) {
      files.filter(file -> !Files.isDirectory(file))
          .map(Path::getFileName)
          .map(Path::toString)
          .filter(fileName -> fileName.endsWith(".path"))
          .filter(fileName -> fileName.startsWith("Auto - "))
          .sorted()
          .map(pathName -> pathName.substring(0, pathName.lastIndexOf(".")))
          .forEach(pathName -> autoChooser.addOption("PP: " + pathName.substring(6), buildAutoForPathGroup(pathName)));
    } catch (IOException e) {
      System.out.println("********* Failed to list PathPlanner paths. *********");
    }

  }

  /**
   * Adds the auto chooser
   * @param dashboard dashboard
   */
  public void addDashboardWidgets(ShuffleboardTab dashboard) {
    dashboard.add("Autonomous", autoChooser).withSize(2, 1).withPosition(0, 3);
  }

  /**
   * Gets the currently selected auto command
   * @return auto command
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  private Command buildAutoForPathGroup(String pathGroupName) {
    var twoConePath = PathPlanner.loadPathGroup(pathGroupName, PathPlanner.getConstraintsFromPath(pathGroupName));
    if (twoConePath == null) {
      return Commands.print("********* Path failed to load. Not running auto: " + pathGroupName + " *********");
    }
    return swerveAutoBuilder.fullAuto(twoConePath);
  }

  private HashMap<String, Command> buildEventMap() {
    var eventMap = new HashMap<String, Command>();
    eventMap.put("ShootCubeTop", shootCubeTop().withTimeout(2.0));
    eventMap.put("ShootCubeMid", shootCubeMid().withTimeout(2.0));
    eventMap.put("ShootCubeBottom", shootCubeBottom().withTimeout(2.0));
    eventMap.put("ShootConeTop", shootConeTop().withTimeout(2.0));
    eventMap.put("ShootConeMid", shootConeMid().withTimeout(2.0));
    eventMap.put("ShootConeBottom", shootConeBottom().withTimeout(2.0));
    eventMap.put("Transit", transit());
    eventMap.put("PickupCone", keepingXBelow(pickupCone(), 7.6));
    eventMap.put("PickupCube", keepingXBelow(pickupCube(), 7.6));
    eventMap.put("ShootCubeFloor", shootCubeFloor().withTimeout(2.0));
    eventMap.put("LaunchCube", shootCubeFloor().withTimeout(2.0));
    eventMap.put("PrepareToLaunchCube", prepareToLaunchCube());
    eventMap.put("PrepareForConePickup", prepareForConePickup());
    eventMap.put("BalanceBackwards", balanceBackwards().withTimeout(3.0)); 
    eventMap.put("BalanceForwards", balanceForwards().withTimeout(3.0)); 
    return eventMap;
  }

  public Command shootCubeTop() {
    return new JustShootCommand(0.8, 0.5, 25.0, elevatorSubsystem, wristSubsystem, shooterSubsystem, ledSubsystem);
  }

  public Command shootCubeMid() {
    return new JustShootCommand(0.55, 0.4, 23.0, elevatorSubsystem, wristSubsystem, shooterSubsystem, ledSubsystem);
  }

  public Command shootCubeBottom() {
    return new JustShootCommand(0.06, 0.1, 15.0, elevatorSubsystem, wristSubsystem, shooterSubsystem, ledSubsystem);
  }

  /**
   * Shoots a cube quickly from the floor to the top node.
   */
  public Command shootCubeFloor() {
    return new JustShootCommand(0.01, 1.23, 100.0, elevatorSubsystem, wristSubsystem, shooterSubsystem, ledSubsystem);
  }

  /**
   * Launches a cube at max velocity
   */
  public Command launchCube() {
    return new JustShootCommand(0.0, 0.78, 130.0, elevatorSubsystem, wristSubsystem, shooterSubsystem, ledSubsystem);
  }

  /**
   * Moves wrist to position to launch a cube
   */
  public Command prepareToLaunchCube() {
    return runOnce(() -> wristSubsystem.moveToPosition(0.78), wristSubsystem);
  }

  public Command shootConeTop() {
    return new ShootConeCommand(ShooterProfile.SCORE_CONE_TOP, LimelightProfile.SCORE_CONE_TOP,
        drivetrainSubsystem, elevatorSubsystem, wristSubsystem, shooterSubsystem, lowLimelightSubsystem,
        ledSubsystem);
  }

  public Command shootConeMid() {
    return new ShootConeCommand(ShooterProfile.SCORE_CONE_MIDDLE, LimelightProfile.SCORE_CONE_MIDDLE,
        drivetrainSubsystem, elevatorSubsystem, wristSubsystem, shooterSubsystem, highLimelightSubsystem,
        ledSubsystem);
  }
  
  public Command shootConeBottom() {
    return new JustShootCommand(0.06, 0.1, 16.0, elevatorSubsystem, wristSubsystem, shooterSubsystem, ledSubsystem);
  }

  public Command transit() {
    return new TransitCommand(elevatorSubsystem, wristSubsystem, shooterSubsystem);
  }

  public Command pickupCone() {
    return new AutoPickupCommand(
          PickupConstants.CONE_ELEVATOR_HEIGHT, PickupConstants.CONE_WRIST_ANGLE, PickupConstants.CONE_INTAKE_DUTY_CYCLE,
          PickupConstants.CONE_FORWARD_SPEED, elevatorSubsystem, wristSubsystem, drivetrainSubsystem, shooterSubsystem,
          poseEstimator::getCurrentPose, highLimelightSubsystem, LimelightProfile.PICKUP_CONE_FLOOR,
          shooterSubsystem::hasCone, PickupConstants.CLASSNAME_CONE)
      .deadlineWith(
          new LEDCustomCommand(leds -> leds.alternate(LEDSubsystem.CONE_COLOR, Color.kRed, 1.0), ledSubsystem));
  }

  public Command pickupCube() {
    return new AutoPickupCommand(
          PickupConstants.CUBE_ELEVATOR_HEIGHT, PickupConstants.CUBE_WRIST_ANGLE, PickupConstants.CUBE_INTAKE_DUTY_CYCLE,
          PickupConstants.CUBE_FORWARD_SPEED, elevatorSubsystem, wristSubsystem, drivetrainSubsystem, shooterSubsystem,
          poseEstimator::getCurrentPose, highLimelightSubsystem, LimelightProfile.PICKUP_CUBE_FLOOR,
          shooterSubsystem::hasCube, PickupConstants.CLASSNAME_CUBE)
      .deadlineWith(
          new LEDCustomCommand(leds -> leds.alternate(LEDSubsystem.CUBE_COLOR, Color.kRed, 1.0), ledSubsystem));
  }

  public Command prepareForConePickup() {
    return runOnce(() -> {
      elevatorSubsystem.moveToPosition(PickupConstants.CONE_ELEVATOR_HEIGHT);
      wristSubsystem.moveToPosition(PickupConstants.CONE_WRIST_ANGLE);
    }, elevatorSubsystem, wristSubsystem);
  }

  public Command driveToPose(Pose2d pose, boolean useAllianceColor) {
    return new DriveToPoseCommand(drivetrainSubsystem, poseEstimator::getCurrentPose, pose, ledSubsystem, useAllianceColor);
  }

  public Command driveToPose(Pose2d pose, TrapezoidProfile.Constraints xyConstraints,
      TrapezoidProfile.Constraints omegaConstraints, boolean useAllianceColor) {
    
    return new DriveToPoseCommand(drivetrainSubsystem, poseEstimator::getCurrentPose, pose, xyConstraints,
        omegaConstraints, ledSubsystem, useAllianceColor);
  }

  public Command balanceBackwards() {
    return new BalanceCommand(drivetrainSubsystem, true);
  }

  public Command balanceForwards() {
    return new BalanceCommand(drivetrainSubsystem, false);
  }

  /**
   * Wraps the given command so that it will end if the robot's X coordinate becomes greater than the given value.
   * @param command command to wrap
   * @param x x value
   * @return wrapped command
   */
  public Command keepingXBelow(Command command, double x) {
    return command.until(() -> poseEstimator.getCurrentPose().getX() > x);
  }

  /**
   * Wraps the given command so that it will end if the robot's X coordinate becomes less than the given value.
   * @param command command to wrap
   * @param x x value
   * @return wrapped command
   */
  public Command keepingXAbove(Command command, double x) {
    return command.until(() -> poseEstimator.getCurrentPose().getX() < x);
  }

}
