package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.WristSubsystem;

public class TeleopConePickupCommand extends Command {

  private static final double ELEVATOR_TOLERANCE = 0.0254;
  private static final double WRIST_TOLERANCE = 0.075;

  private final double elevatorMeters;
  private final double wristRadians;
  private final double intakeDutyCycle;
  private final double forwardSpeed;

  private final DoubleSupplier ySupplier;
  private final DoubleSupplier xSupplier;
  private final DoubleSupplier rotationSupplier;

  private final ElevatorSubsystem elevatorSubsystem;
  private final WristSubsystem wristSubsystem;
  private final ShooterSubsystem shooterSubsystem;
  private final DrivetrainSubsystem drivetrainSubsystem;
  private final BooleanSupplier doneSuppler;

  public TeleopConePickupCommand(
      double elevatorMeters, double wristRadians, double intakeDutyCycle, double forwardSpeed, 
      ElevatorSubsystem elevatorSubsystem, WristSubsystem wristSubsystem, DrivetrainSubsystem drivetrainSubsystem, 
      ShooterSubsystem shooterSubsystem,  DoubleSupplier xSupplier, DoubleSupplier ySupplier,
      DoubleSupplier rotationSupplier, BooleanSupplier doneSuppler) {

    this.elevatorMeters = elevatorMeters;
    this.wristRadians = wristRadians;
    this.intakeDutyCycle = intakeDutyCycle;
    this.forwardSpeed = forwardSpeed;
    this.elevatorSubsystem = elevatorSubsystem;
    this.wristSubsystem = wristSubsystem;
    this.drivetrainSubsystem = drivetrainSubsystem;
    this.shooterSubsystem = shooterSubsystem;
    this.xSupplier = xSupplier;
    this.ySupplier = ySupplier;
    this.rotationSupplier = rotationSupplier;
    this.doneSuppler = doneSuppler;

    addRequirements(elevatorSubsystem, wristSubsystem, shooterSubsystem, drivetrainSubsystem);
  }

  @Override
  public void initialize() {
    wristSubsystem.moveToPosition(wristRadians);
  }
  
  @Override
  public void execute() {
    var strafeSpeed = ySupplier.getAsDouble();
    var rotationSpeed = rotationSupplier.getAsDouble();

    var xSpeed = xSupplier.getAsDouble();
    if (xSpeed > 0) {
      xSpeed = forwardSpeed;
    }
    drivetrainSubsystem.drive(new ChassisSpeeds(xSpeed, strafeSpeed, rotationSpeed));

    elevatorSubsystem.moveToPosition(elevatorMeters);

    var readyToIntake =
        Math.abs(elevatorSubsystem.getElevatorPosition() - elevatorMeters) < ELEVATOR_TOLERANCE
        && Math.abs(wristSubsystem.getWristPosition() - wristRadians) < WRIST_TOLERANCE;

    if (readyToIntake) {
      shooterSubsystem.shootDutyCycle(intakeDutyCycle);
    }
  }

  @Override
  public boolean isFinished() {
    return doneSuppler.getAsBoolean();
  }

  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.activeStop();
  }

}
