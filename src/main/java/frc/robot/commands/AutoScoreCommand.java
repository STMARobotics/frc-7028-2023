package frc.robot.commands;

import static frc.robot.Constants.DrivetrainConstants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.AutonomousBuilder;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ScoreLocationSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.WristSubsystem;

/**
 * Command to look at the selected scoring location, drive to pose, then shoot.
 */
public class AutoScoreCommand extends CommandBase {

  private static final TrapezoidProfile.Constraints XY_CONSTRAINTS = new TrapezoidProfile.Constraints(
      2.0,
      2.0);
  private static final TrapezoidProfile.Constraints OMEGA_CONSTRAINTS = new TrapezoidProfile.Constraints(
      MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND * 0.4,
      MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND * 2.0);
  
  private final ScoreLocationSubsystem scoreLocationSubsystem;
  private final BooleanSupplier hasCone;
  private final AutonomousBuilder autoBuilder;

  public AutoScoreCommand(ScoreLocationSubsystem scoreLocationSubsystem, BooleanSupplier hasCone,
      AutonomousBuilder autoBuilder, 
      DrivetrainSubsystem drivetrainSubsystem,
      ElevatorSubsystem elevatorSubsystem,
      LEDSubsystem ledSubsystem,
      ShooterSubsystem shooterSubsystem,
      WristSubsystem wristSubsystem,
      LimelightSubsystem highLimelightSubsystem,
      LimelightSubsystem lowLimelightSubsystem
    ) {
    this.scoreLocationSubsystem = scoreLocationSubsystem;
    this.hasCone = hasCone;
    this.autoBuilder = autoBuilder;

    addRequirements(drivetrainSubsystem, elevatorSubsystem, ledSubsystem, shooterSubsystem, wristSubsystem,
        highLimelightSubsystem, lowLimelightSubsystem);
  }

  private final Pose2d[][] shootPoses = new Pose2d[][]{
    // Grid 0
    new Pose2d[]{
      new Pose2d(2.0, 4.74, Rotation2d.fromDegrees(180)),
      new Pose2d(1.8, 4.43, Rotation2d.fromDegrees(180)),
      new Pose2d(2.0, 3.88, Rotation2d.fromDegrees(180))
    },
    // Grid 1
    new Pose2d[]{
      new Pose2d(2.0, 3.30, Rotation2d.fromDegrees(180)),
      new Pose2d(1.8, 2.75, Rotation2d.fromDegrees(180)),
      new Pose2d(2.0, 2.19, Rotation2d.fromDegrees(180))
    },
    // Grid 2
    new Pose2d[]{
      new Pose2d(2.0, 1.62, Rotation2d.fromDegrees(180)),
      new Pose2d(1.8, 1.07, Rotation2d.fromDegrees(180)),
      new Pose2d(2.0, 0.80, Rotation2d.fromDegrees(-174))
    },
  };

  private Alliance alliance = Alliance.Invalid;
  private Command scoreSequence;

  public void setAlliance(Alliance alliance) {
    this.alliance = alliance;
  }

  @Override
  public void initialize() {
    Command shootCommand;
    var grid = scoreLocationSubsystem.getSelectedGrid();
    var column = scoreLocationSubsystem.getSelectedColumn();
    var row = scoreLocationSubsystem.getSelectedRow();

    if (row == 0) {
      // Hybrid nodes - select based on shooter
      if (hasCone.getAsBoolean()) {
        shootCommand = autoBuilder.shootConeBottom();
      } else {
        shootCommand = autoBuilder.shootCubeBottom();
      }
    } else if(column == 1) {
      if (row == 0) {
        shootCommand = autoBuilder.shootCubeBottom();
      } else if (row == 1) {
        shootCommand = autoBuilder.shootCubeMid();
      } else {
        shootCommand = autoBuilder.shootCubeTop();
      }
    } else {
      if (row == 0) {
        shootCommand = autoBuilder.shootConeBottom();
      } else if (row == 1) {
        shootCommand = autoBuilder.shootConeMid();
      } else {
        shootCommand = autoBuilder.shootConeTop();
      }
    }

    if (alliance == Alliance.Red) {
      // On red side, grid numbers are flipped 0 is 2 and vice versa
      grid = Math.abs(grid - 2);
      column = Math.abs(column - 2);
    }

    // Find the pose for the grid location
    scoreSequence = new SequentialCommandGroup(
      autoBuilder.driveToPose(shootPoses[grid][column], XY_CONSTRAINTS, OMEGA_CONSTRAINTS, true),
      shootCommand
    );

    scoreSequence.initialize();
  }

  @Override
  public void execute() {
    scoreSequence.execute();
  }

  @Override
  public boolean isFinished() {
    return scoreSequence.isFinished();
  }

  @Override
  public void end(boolean interrupted) {
    scoreSequence.end(interrupted);
  }

}
