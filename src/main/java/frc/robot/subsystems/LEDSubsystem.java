package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Subsystem to handle LED lighting
 */
public class LEDSubsystem extends SubsystemBase {
  
  private static final int LED_COUNT = 128;
  public static final int STRIP_COUNT = 4;
  public static final int STRIP_SIZE = LED_COUNT / STRIP_COUNT;
  public static final Color CUBE_COLOR = new Color(40, 0, 125);
  public static final Color CONE_COLOR = Color.kOrange;

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

    /** Robot is shooting */
    SHOOTING,

    /** Robot is shooting in a position that doesn't need a target */
    SHOOTING_WITHOUT_TARGET,

    /** Robot going to human player stations to get a cone */
    WANT_CONE,

    /** Robot going to human player station to get a cube */
    WANT_CUBE,

    /** Driverstation disconnected */
    DS_DISCONNECT,

    /** A custom mode will be set by calling methods to set LEDs */
    CUSTOM
  }

  private final AddressableLED leds = new AddressableLED(0);
  private final AddressableLEDBuffer buffer = new AddressableLEDBuffer(LED_COUNT);

  private Mode currentMode = Mode.BLUE_GOLD;
  private Timer timer = new Timer();
  private boolean refresh = false;

  public LEDSubsystem() {
    leds.setLength(LED_COUNT);
    leds.setData(buffer);
    leds.start();
  }

  /**
   * Sets a specific LED in the buffer. Must set mode to CUSTOM
   * 
   * @param stripId strip to write
   * @param ledId LED id to write
   * @param color color of the LED
   */
  public void setLED(int stripId, int ledId, Color color) {
    buffer.setLED(customUpdateIndex(stripId, ledId), color);
  }

  /**
   * Sets a specific LED in the buffer. Must set mode to CUSTOM
   *
   * @param stripId strip to write
   * @param ledId LED id to write
   * @param color color of the LED
   */
  public void setLED(int stripId, int ledId, Color8Bit color) {
    buffer.setLED(customUpdateIndex(stripId, ledId), color);
  }

  /**
   * Sets a specific led in the buffer. Must set mode to CUSTOM
   *
   * @param stripId strip to write
   * @param ledId LED id to write
   * @param h hue [0-180)
   * @param s saturation [0-255]
   * @param v value [0-255]
   */
  public void setHSV(int stripId, int ledId, int h, int s, int v) {
    buffer.setHSV(customUpdateIndex(stripId, ledId), h, s, v);
  }

  /**
   * Sets a specific led in the buffer. Must set mode to CUSTOM
   *
   * @param stripId strip to write
   * @param ledId LED id to write
   * @param r red [0-255]
   * @param g green [0-255]
   * @param b blue [0-255]
   */
  public void setRGB(int stripId, int ledId, int r, int g, int b) {
    buffer.setRGB(customUpdateIndex(stripId, ledId), r, g, b);
  }

  /**
   * Calculates the index for an LED on a strip. The strips serpentine - index 0 and 2 start at the bottom of the robot,
   * 1 and 3 start at the top.
   * @param stripId ID of the strip [0,3]
   * @param ledId ID of the LED on the strip, always at the bottom of the robot
   * @return LED index in the buffer
   */
  private int customUpdateIndex(int stripId, int ledId) {
    refresh = true;
    int firstId = stripId * STRIP_SIZE;
    return stripId % 2 == 0 ? firstId + ledId : firstId + STRIP_SIZE - ledId - 1;
  }

  @Override
  public void periodic() {
    switch (currentMode) {
      case BLUE_GOLD:
        alternate(Color.kBlue, Color.kOrange, 1.0);
        break;
      case HAS_CUBE:
        setAll(CUBE_COLOR);
        break;
      case HAS_CONE:
        setAll(CONE_COLOR);
        break;
      case SHOOTING_NO_TARGET:
        setAll(Color.kRed);
        break;
      case SHOOTING_HAS_TARGET:
        alternate(Color.kGreen, Color.kRed, 0.5);
        break;
      case SHOOTING_WITHOUT_TARGET:
        setAll(Color.kOrangeRed);
        break;
      case SHOOTING:
        setAll(Color.kGreen);
        break;
      case WANT_CONE:
        alternate(CONE_COLOR, Color.kBlack, 1.0);
        break;
      case WANT_CUBE:
        alternate(CUBE_COLOR, Color.kBlack, 1.0);
        break;
      case DS_DISCONNECT:
        alternate(Color.kDarkRed, Color.kIndianRed, 0.5);
        break;
      case CUSTOM:
        break;
    }
    if (refresh) {
      leds.setData(buffer);
      refresh = false;
    }
  }

  /**
   * Set the LED mode
   * @param mode mode
   */
  public void setMode(Mode mode) {
    if (mode != currentMode) {
      currentMode = mode;
      refresh = true;
      timer.reset();
    }
  }

  /**
   * Alternate every-other LED between two different colors.
   * @param color1 first color
   * @param color2 second color
   * @param interval interval to alternate, in seconds
   */
  public void alternate(Color color1, Color color2, double interval) {
    timer.start();
    if (timer.advanceIfElapsed(interval) || refresh) {
      long currentTime = System.currentTimeMillis();
      for (int strip = 0; strip < STRIP_COUNT; strip++) {
        for (int index = 0; index < STRIP_SIZE; index++) {
          if (index % 2 == (currentTime / (int) (interval * 1000) % 2)) {
            setLED(strip, index, color1);
          } else {
            setLED(strip, index, color2);
          }
        }
      }
      refresh = true;
    }
  }
  
  public void setAll(Color color) {
    for (var i = 0; i < LED_COUNT; i++) {
      buffer.setLED(i, color);
    }
  }

  public void setAll(int r, int g, int b) {
    for (var i = 0; i < LED_COUNT; i++) {
      buffer.setRGB(i, r, g, b);
    }
  }

}
