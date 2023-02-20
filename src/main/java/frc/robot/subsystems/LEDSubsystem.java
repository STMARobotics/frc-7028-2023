package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Subsystem to handle LED lighting
 */
public class LEDSubsystem extends SubsystemBase {
  
  private static final int STRIP_COUNT = 4;
  private static final int LED_COUNT = 128;
  private static final int STRIP_SIZE = LED_COUNT / STRIP_COUNT;

  /**
   * Lighting mode
   */
  public static enum Mode {
    /** Alternating blue/gold */
    BLUE_GOLD,

    /** Robot has a cube */
    HAS_CUBE,

    /** Robot has a cone */
    HAS_CONE,

    /** Robot is shooting but sees no target */
    SHOOTING_NO_TARGET,

    /** Robot is shooting and sees a target */
    SHOOTING_HAS_TARGET,

    /** Robot is shooting in a position that doesn't need a target */
    SHOOTING_WITHOUT_TARGET,

    /** Robot going to human player stations to get a cone */
    WANT_CONE,

    /** Robot going to human player station to get a cube */
    WANT_CUBE
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
      case HAS_CUBE:
        blue();
        break;
      case HAS_CONE:
        yellow();
        break;
      case SHOOTING_NO_TARGET:
        shootingNoTarget();
        break;
      case SHOOTING_HAS_TARGET:
        shootingHasTarget();
        break;
      case SHOOTING_WITHOUT_TARGET:
        shootingWithoutTarget();
        break;
      case WANT_CONE:
        wantCone();
        break;
      case WANT_CUBE:
        wantCube();
        break;
    }
    leds.setData(buffer);
  }

  public void setMode(Mode mode) {
    currentMode = mode;
  }

  private void blueGold() {
    alternate(Color.kBlue, Color.kGold, 1000);
  }

  private void alternate(Color color1, Color color2, int interval) {
    long currentTime = System.currentTimeMillis();
    for (var strip = 0; strip < STRIP_COUNT; strip++) {
      int offset = strip * STRIP_SIZE;
      Color evenColor;
      Color oddColor;
      if (strip % 2 == 1) {
        evenColor = color1;
        oddColor = color2;
      } else {
        evenColor = color2;
        oddColor = color1;
      }
      for (var i = 0; i < STRIP_SIZE; i++) {
        if (i % 2 == ((currentTime / interval) % 2)) {
          buffer.setLED(i + offset, evenColor);
        } else {
          buffer.setLED(i + offset, oddColor);
        }
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
      buffer.setLED(i, Color.kYellow);
    }
  }

  private void shootingHasTarget() {
    alternate(Color.kYellow, Color.kRed, 500);
  }

  private void shootingNoTarget() {
    for (var i = 0; i < LED_COUNT; i++) {
      buffer.setLED(i, Color.kRed);
    }
  }

  private void shootingWithoutTarget() {
    for (var i = 0; i < LED_COUNT; i++) {
      buffer.setLED(i, Color.kBlack);
    }
  }

  private void wantCone() {
    for (var i = 0; i < LED_COUNT; i++) {
      buffer.setLED(i, Color.kBlack);
    }
  }

  private void wantCube() {
    for (var i = 0; i < LED_COUNT; i++) {
      buffer.setLED(i, Color.kBlack);
    }
  }

}
