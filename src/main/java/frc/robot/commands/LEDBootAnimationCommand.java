package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LEDStrips;
import frc.robot.subsystems.LEDSubsystem;

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
    ledSubsystem.setCustomMode(this::animate);
  }

  private void animate(LEDStrips ledStrips) {
    if (timer.advanceIfElapsed(0.05)) {
      for(int strip = 0; strip < LEDSubsystem.STRIP_COUNT; strip++) {
        for(int index = 0; index < LEDSubsystem.STRIP_SIZE; index++) {
          if (index <= blipIndex && index >= blipIndex - (BLIP_SIZE - 1)) {
            ledStrips.setLED(strip, index, Color.kOrange);
          } else {
            ledStrips.setLED(strip, index, Color.kBlue);
          }
        }
      }
      blipIndex++;
      ledStrips.refresh();
    }
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
