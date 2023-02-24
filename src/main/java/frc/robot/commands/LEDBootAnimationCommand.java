package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.LEDSubsystem.Mode;

/** Command to run the boot animation on the LED strips */
public class LEDBootAnimationCommand extends CommandBase {
  
  private static final int BLIP_SIZE = 5;
  private final LEDSubsystem ledSubsystem;
  private final Timer timer = new Timer();
  
  private int blipIndex = -1;

  public LEDBootAnimationCommand(LEDSubsystem ledSubsystem) {
    this.ledSubsystem = ledSubsystem;

    addRequirements(ledSubsystem);
  }

  @Override
  public void initialize() {
    timer.start();
    ledSubsystem.setMode(Mode.CUSTOM);
    animate();
  }

  @Override
  public void execute() {
    if (timer.advanceIfElapsed(0.05)) {
      animate();
    }
  }

  private void animate() {
    for(int strip = 0; strip < LEDSubsystem.STRIP_COUNT; strip++) {
      for(int index = 0; index < LEDSubsystem.STRIP_SIZE; index++) {
        if (index <= blipIndex && index >= blipIndex - (BLIP_SIZE - 1)) {
          ledSubsystem.setLED(strip, index, Color.kOrange);
        } else {
          ledSubsystem.setLED(strip, index, Color.kBlue);
        }
      }
    }
    blipIndex++;
  }

  @Override
  public boolean isFinished() {
    return blipIndex - (BLIP_SIZE + 1) >= LEDSubsystem.STRIP_SIZE;
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }

}
