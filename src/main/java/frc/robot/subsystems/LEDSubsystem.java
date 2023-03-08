package frc.robot.subsystems;

import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Consumer;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Notifier;
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

  private final AtomicReference<Consumer<LEDStrips>> ledUpdateConsumer = new AtomicReference<Consumer<LEDStrips>>(null);
  private final Notifier ledNotifier;
  private final LEDStripMethods ledStripMethods = new LEDStripMethods();

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

    /** Driverstation disconnected */
    DS_DISCONNECT,

    /** All LEDs off */
    OFF
  }

  private final AddressableLED leds = new AddressableLED(0);
  private final AddressableLEDBuffer buffer = new AddressableLEDBuffer(LED_COUNT);

  private Mode currentMode = null;
  private Timer timer = new Timer();
  private boolean refresh = false;

  public LEDSubsystem() {
    leds.setLength(LED_COUNT);
    leds.setData(buffer);
    leds.start();
    ledNotifier = new Notifier(() ->  {
      var value = ledUpdateConsumer.get();
      if (value != null) {
        // Call the consumer to update the LEDs on this notifier thread
        value.accept(ledStripMethods);
      }
    });
    ledNotifier.setName("LEDs");
    ledNotifier.startPeriodic(0.02);
  }


  /**
   * Calculates the index for an LED on a strip. The strips serpentine - index 0 and 2 start at the bottom of the robot,
   * 1 and 3 start at the top.
   * @param stripId ID of the strip [0,3]
   * @param ledId ID of the LED on the strip, always at the bottom of the robot
   * @return LED index in the buffer
   */
  private int calculateUpdateIndex(int stripId, int ledId) {
    int firstId = stripId * STRIP_SIZE;
    return stripId % 2 == 0 ? firstId + ledId : firstId + STRIP_SIZE - ledId - 1;
  }

  /**
   * Set the LED mode
   * @param mode mode
   */
  public void setMode(Mode mode) {
    if (mode != currentMode) {
      currentMode = mode;
      timer.stop();
      timer.reset();
      switch (currentMode) {
        case BLUE_GOLD:
          ledUpdateConsumer.set((l) -> l.alternate(Color.kBlue, Color.kOrange, 1.0));
          break;
        case HAS_CUBE:
          ledUpdateConsumer.set((l) -> setAllOnce(l, CUBE_COLOR));
          break;
        case HAS_CONE:
          ledUpdateConsumer.set((l) -> setAllOnce(l, CONE_COLOR));
          break;
        case SHOOTING_NO_TARGET:
          ledUpdateConsumer.set((l) -> setAllOnce(l, Color.kRed));
          break;
        case SHOOTING_HAS_TARGET:
          ledUpdateConsumer.set((l) -> l.alternate(Color.kGreen, Color.kRed, 0.5));
          break;
        case SHOOTING_WITHOUT_TARGET:
          ledUpdateConsumer.set((l) -> setAllOnce(l, Color.kOrangeRed));
          break;
        case SHOOTING:
          ledUpdateConsumer.set((l) -> setAllOnce(l, Color.kGreen));
          break;
        case DS_DISCONNECT:
          ledUpdateConsumer.set((l) -> l.alternate(Color.kDarkRed, Color.kIndianRed, 0.5));
          break;
        case OFF:
          ledUpdateConsumer.set((l) -> setAllOnce(l, Color.kBlack));
      }
      refresh = true;
    }
  }

  /**
   * Sets all of the LEDs to the specified color, and removes the consumer from the notifier so it will not call back.
   * @param ledStrips led strips object
   * @param color color to set
   */
  private void setAllOnce(LEDStrips ledStrips, Color color) {
    ledStrips.setAll(color);
    ledUpdateConsumer.set(null);
  }

  /**
   * Sets a custom LED mode. Pass an LEDStrips consumer that will be called on a background thread when it's time
   * to refresh. The updater should update the LEDs with a "set" method and then call {@link #refresh()}
   * @param updater consumer that gets called when it's time to refresh
   */
  public void setCustomMode(Consumer<LEDStrips> updater) {
    ledUpdateConsumer.set(updater);
    currentMode = null;
  }

  private class LEDStripMethods implements LEDStrips {

    public void setLED(int stripId, int ledId, Color color) {
      buffer.setLED(calculateUpdateIndex(stripId, ledId), color);
    }

    public void setLED(int stripId, int ledId, Color8Bit color) {
      buffer.setLED(calculateUpdateIndex(stripId, ledId), color);
    }

    public void setHSV(int stripId, int ledId, int h, int s, int v) {
      buffer.setHSV(calculateUpdateIndex(stripId, ledId), h, s, v);
    }

    public void setRGB(int stripId, int ledId, int r, int g, int b) {
      buffer.setRGB(calculateUpdateIndex(stripId, ledId), r, g, b);
    }
      
    public void refresh() {
      leds.setData(buffer);
    }

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
        refresh();
      }
    }
    
    public void setAll(Color color) {
      for (var i = 0; i < LED_COUNT; i++) {
        buffer.setLED(i, color);
      }
      refresh();
    }

    public void setAll(int r, int g, int b) {
      for (var i = 0; i < LED_COUNT; i++) {
        buffer.setRGB(i, r, g, b);
      }
      refresh();
    }

    public void setLEDSegments(Color color, boolean... segmentValues) {
      int ledsPerStatus = LEDSubsystem.STRIP_SIZE / segmentValues.length;
      for(int stripId = 0; stripId < LEDSubsystem.STRIP_COUNT; stripId++) {
        int ledIndex = 0;
        for (int segmentId = 0; segmentId < segmentValues.length; segmentId++) {
          for(;ledIndex < (ledsPerStatus * (segmentId + 1)); ledIndex++) {
            setLED(stripId, ledIndex, segmentValues[segmentId] ? color : Color.kBlack);
          }
        }
      }
      refresh();
    }
  }

}
