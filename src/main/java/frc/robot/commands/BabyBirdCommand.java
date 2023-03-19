package frc.robot.commands;

import static edu.wpi.first.wpilibj2.command.Commands.run;
import static frc.robot.Constants.PickupConstants.BABY_BIRD_ELEVATOR_HEIGHT;
import static frc.robot.Constants.PickupConstants.BABY_BIRD_INTAKE_DUTY_CYCLE;
import static frc.robot.Constants.PickupConstants.BABY_BIRD_WRIST_ANGLE;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.autonomous.TransitCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.WristSubsystem;

/** Command to pick up a game piece from the the single station - and hold elevator and wrist until interrupted */
public class BabyBirdCommand extends CommandBase {

  private static final double SPEED_COEFFICIENT = 0.2;
  private static final double XY_SLEW_RATE = 2.0;
  private static final double THETA_SLEW_RATE = 3 * Math.PI;

  private static final double ELEVATOR_TOLERANCE = 0.0254;

  private final DoubleSupplier xSupplier;
  private final DoubleSupplier ySupplier;
  private final DoubleSupplier omegaSupplier;
  private final ElevatorSubsystem elevatorSubsystem;
  private final WristSubsystem wristSubsystem;
  private final DrivetrainSubsystem drivetrainSubsystem;
  private final ShooterSubsystem shooterSubsystem;
  private final Supplier<Pose2d> poseSupplier;

  private final SlewRateLimiter xRateLimiter = new SlewRateLimiter(XY_SLEW_RATE);
  private final SlewRateLimiter yRateLimiter = new SlewRateLimiter(XY_SLEW_RATE);
  private final SlewRateLimiter thetaRateLimiter = new SlewRateLimiter(THETA_SLEW_RATE);
  
  /**
   * Constructor
   * @param xSupplier x speed - this will be limited
   * @param ySupplier y speed - this will be limited
   * @param omegaSupplier rotation speed - this will be limited
   * @param elevatorSubsystem elevator
   * @param wristSubsystem wrist
   * @param drivetrainSubsystem drivetrain
   * @param shooterSubsystem shooter
   * @param poseSupplier pose supplier
   */
  public BabyBirdCommand(DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier omegaSupplier,
      ElevatorSubsystem elevatorSubsystem, WristSubsystem wristSubsystem, DrivetrainSubsystem drivetrainSubsystem,
      ShooterSubsystem shooterSubsystem, Supplier<Pose2d> poseSupplier) {

    this.xSupplier = xSupplier;
    this.ySupplier = ySupplier;
    this.omegaSupplier = omegaSupplier;
    this.elevatorSubsystem = elevatorSubsystem;
    this.wristSubsystem = wristSubsystem;
    this.drivetrainSubsystem = drivetrainSubsystem;
    this.shooterSubsystem = shooterSubsystem;
    this.poseSupplier = poseSupplier;

    addRequirements(elevatorSubsystem, wristSubsystem, drivetrainSubsystem, shooterSubsystem);
  }

  @Override
  public void initialize() {
    elevatorSubsystem.moveToPosition(BABY_BIRD_ELEVATOR_HEIGHT);
    wristSubsystem.moveToPosition(BABY_BIRD_WRIST_ANGLE);
    shooterSubsystem.shootDutyCycle(BABY_BIRD_INTAKE_DUTY_CYCLE);

    var robotAngle = poseSupplier.get().getRotation();
    var chassisSpeeds = drivetrainSubsystem.getChassisSpeeds();
    var fieldSpeeds = 
        new Translation2d(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond).rotateBy(robotAngle);
    var robotSpeeds = new ChassisSpeeds(fieldSpeeds.getX(), fieldSpeeds.getY(), chassisSpeeds.omegaRadiansPerSecond);
    
    // Reset the slew rate limiters, in case the robot is already moving
    xRateLimiter.reset(robotSpeeds.vxMetersPerSecond);
    yRateLimiter.reset(robotSpeeds.vyMetersPerSecond);
    thetaRateLimiter.reset(robotSpeeds.omegaRadiansPerSecond);
  }
  
  @Override
  public void execute() {
    elevatorSubsystem.moveToPosition(BABY_BIRD_ELEVATOR_HEIGHT);

    if (shooterSubsystem.hasCone() || shooterSubsystem.hasCube()) {
      shooterSubsystem.activeStop();
    } else {
      shooterSubsystem.shootDutyCycle(BABY_BIRD_INTAKE_DUTY_CYCLE);
    }

    // Don't let driver move until elevator is at height
    var driverCanDrive =
        Math.abs(elevatorSubsystem.getElevatorPosition() - BABY_BIRD_ELEVATOR_HEIGHT) <= ELEVATOR_TOLERANCE;
    var xSpeed = 0.0;
    var ySpeed = 0.0;
    var omegaSpeed = 0.0;
    if (driverCanDrive) {
      xSpeed = xSupplier.getAsDouble() * SPEED_COEFFICIENT;
      ySpeed = ySupplier.getAsDouble() * SPEED_COEFFICIENT;
      omegaSpeed = omegaSupplier.getAsDouble() * SPEED_COEFFICIENT;
    }
    xSpeed = xRateLimiter.calculate(xSpeed);
    ySpeed = yRateLimiter.calculate(ySpeed);
    omegaSpeed = thetaRateLimiter.calculate(omegaSpeed);

    drivetrainSubsystem.drive(
        ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, omegaSpeed, poseSupplier.get().getRotation()));
  }

  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.activeStop();
    wristSubsystem.parkWrist();
    drivetrainSubsystem.stop();
    var transitCommand = new TransitCommand(elevatorSubsystem, wristSubsystem, shooterSubsystem)
        .deadlineWith(run(drivetrainSubsystem::stop, drivetrainSubsystem));
    transitCommand.schedule();
  }

}
