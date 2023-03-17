package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LEDStrips;
import frc.robot.subsystems.LEDSubsystem;

/** Command to run the boot animation on the LED strips */
public class LEDBootAnimationCommand extends CommandBase {

  private final LEDSubsystem ledSubsystem;
  private final Timer timer = new Timer();

  private int checker1 = 0;
  private int checker2 = 0;
  private int refreshCount = 0;
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
    if (timer.advanceIfElapsed(.06) && refreshCount < LEDSubsystem.STRIP_SIZE) {
      for (int strip = 0; strip < LEDSubsystem.STRIP_COUNT; strip++) {
        if (strip % 2 == 0) {
          evenStrip(strip, ledStrips);
        } else {
          oddStrip(strip, ledStrips);
        }
      }
      
      checker1 = (checker1 + 1) % LEDSubsystem.STRIP_SIZE;
      checker2 = (checker2 + 1) % LEDSubsystem.STRIP_SIZE;
      refreshCount++;
      ledStrips.refresh();
    }
  }
  
  private void evenStrip(int strip, LEDStrips ledStrips) {
    for (int index = 0; index < LEDSubsystem.STRIP_SIZE; index++) {
      if (refreshCount % 2 == 0 && index == checker1 || index % 2 == 0 && index <= checker1) {
        ledStrips.setLED(strip, index, Color.kOrange);
      } else {
        ledStrips.setLED(strip, index, Color.kBlue);
      }
    }
  }

  private void oddStrip(int strip, LEDStrips ledStrips) {
    for (int index = 0; index < LEDSubsystem.STRIP_SIZE; index++) {
      if (refreshCount % 2 == 1 && index == checker2 || index % 2 == 1 && index <= checker2) {
        ledStrips.setLED(strip, index, Color.kOrange);
      } else {
        ledStrips.setLED(strip, index, Color.kBlue);
      }
    }
  }

  @Override
  public boolean isFinished() {
    return checker2 >= 31;
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }

}
