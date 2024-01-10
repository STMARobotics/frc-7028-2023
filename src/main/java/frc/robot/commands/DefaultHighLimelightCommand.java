package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.GamePiece;
import frc.robot.limelight.LimelightProfile;
import frc.robot.subsystems.LimelightSubsystem;

/**
 * Command for the high limelight to proactively switch pipelines.
 */
public class DefaultHighLimelightCommand extends Command {
  
  public final BooleanSupplier hasCone;
  public final BooleanSupplier hasCube;
  public final LimelightSubsystem limelightSubsystem;
  public final Supplier<GamePiece> gamePieceSupplier;

  public DefaultHighLimelightCommand(BooleanSupplier hasCone, BooleanSupplier hasCube,
      LimelightSubsystem limelightSubsystem, Supplier<GamePiece> gamePieceSupplier) {
    this.hasCone = hasCone;
    this.hasCube = hasCube;
    this.gamePieceSupplier = gamePieceSupplier;
    this.limelightSubsystem = limelightSubsystem;

    addRequirements(limelightSubsystem);
  }

  @Override
  public void execute() {
    if(hasCone.getAsBoolean()) {
      limelightSubsystem.setPipelineId(LimelightProfile.SCORE_CONE_MIDDLE.pipelineId);
    } else if (hasCube.getAsBoolean()) {
      limelightSubsystem.setPipelineId(LimelightProfile.PICKUP_CUBE_FLOOR.pipelineId);
    } else if (gamePieceSupplier.get() == GamePiece.CONE) {
      limelightSubsystem.setPipelineId(LimelightProfile.PICKUP_CONE_FLOOR.pipelineId);
    } else {
      limelightSubsystem.setPipelineId(LimelightProfile.PICKUP_CUBE_FLOOR.pipelineId);
    }
  }

}
