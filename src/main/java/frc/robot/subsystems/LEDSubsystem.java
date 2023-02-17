package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Subsystem to handle LED lighting
 */
public class LEDSubsystem extends SubsystemBase {
  
  private static final int LED_COUNT = 128;

  /**
   * Lighting mode
   */
  public static enum Mode {
    /** Alternating blue/gold */
    BLUE_GOLD,

    /** All blue */
    BLUE,

    /** All yellow */
    YELLOW
  }

  private final AddressableLED leds = new AddressableLED(0);
  private final AddressableLEDBuffer buffer = new AddressableLEDBuffer(LED_COUNT);

  private Mode currentMode = Mode.BLUE_GOLD;

  public LEDSubsystem() {
    leds.setLength(LED_COUNT);
    leds.setData(buffer);
    leds.start();
  }

  @Override
  public void periodic() {
    switch (currentMode) {
      case BLUE_GOLD:
        blueGold();
        break;
      case BLUE:
        blue();
        break;
      case YELLOW:
        yellow();
        break;
    }
    leds.setData(buffer);
  }

  public void setMode(Mode mode) {
    currentMode = mode;
  }

  private void blueGold() {
    for (var i = 0; i < LED_COUNT; i++) {
      if (i % 2 == ((System.currentTimeMillis() / 1000) % 2)) {
        buffer.setRGB(i, 234, 255, 3);
      } else {
        buffer.setRGB(i, 0, 0, 255);
      }
    }
  }

  private void blue() {
    for (var i = 0; i < LED_COUNT; i++) {
      buffer.setRGB(i, 0, 0, 255);
    }
  }

  private void yellow() {
    for (var i = 0; i < LED_COUNT; i++) {
      buffer.setRGB(i, 234, 255, 3);
    }
  }

}
