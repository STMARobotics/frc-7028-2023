package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LEDStrips;
import frc.robot.subsystems.LEDSubsystem;

/** Runs an LED marquee effect */
public class LEDMarqueeCommand extends Command {
  
  private static final int MARQUEE_SIZE = 16;
  private final LEDSubsystem ledSubsystem;
  private final int hue;
  private final int saturation;
  private final int minValue;
  private final int valueStep;
  private final double duration;

  private final Timer timer = new Timer();
  
  private int frame;

  /**
   * Constructor
   * @param ledSubsystem LED subsystem
   * @param hue hue
   * @param saturation saturation
   * @param minValue lowest value, will be increased by valueStep
   * @param valueStep how much to increase the color value on each step
   * @param duration duration of each frame
   */
  public LEDMarqueeCommand(
      LEDSubsystem ledSubsystem, int hue, int saturation, int minValue, int valueStep, double duration) {
    this.ledSubsystem = ledSubsystem;
    this.hue = hue;
    this.saturation = saturation;
    this.minValue = minValue;
    this.valueStep = valueStep;
    this.duration = duration;

    addRequirements(ledSubsystem);
  }

  @Override
  public void initialize() {
    timer.reset();
    timer.start();
    frame = LEDSubsystem.STRIP_SIZE;
    ledSubsystem.setCustomMode(this::animate);
  }

  private void animate(LEDStrips ledStrips) {
    if (timer.advanceIfElapsed(duration)) {
      for(int strip = 0; strip < LEDSubsystem.STRIP_COUNT; strip++) {
        for(int index = 0; index < LEDSubsystem.STRIP_SIZE; index++) {
          int value = minValue + ((index + frame) % MARQUEE_SIZE) * valueStep;
          ledStrips.setHSV(strip, index, hue, saturation, value);
        }
      }
      frame--;
    }
    if (frame == 0) {
      frame = LEDSubsystem.STRIP_SIZE;
    }
    ledStrips.refresh();
  }
  
  @Override
  public boolean runsWhenDisabled() {
    return true;
  }

}
