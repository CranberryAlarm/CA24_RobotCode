package frc.robot.subsystems.leds;

import java.util.function.Function;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Constants;
import frc.robot.subsystems.Subsystem;

public class LEDs extends Subsystem {
  private static LEDs m_instance;

  private AddressableLED m_led;
  private AddressableLEDBuffer m_buffer;

  private int m_ledTotalLength = Constants.LEDs.k_totalLength;

  // Main sections
  private Function<Integer, Function<Integer, Function<AddressableLEDBuffer, AddressableLEDBuffer>>> m_rightArmColor = LEDModes
      .setColor(Color.kRed);
  private Function<Integer, Function<Integer, Function<AddressableLEDBuffer, AddressableLEDBuffer>>> m_leftArmColor = LEDModes
      .setColor(Color.kRed);
  private Function<Integer, Function<Integer, Function<AddressableLEDBuffer, AddressableLEDBuffer>>> m_driveColor = LEDModes.rainbow;

  // Front/back overrides
  private Function<Integer, Function<Integer, Function<AddressableLEDBuffer, AddressableLEDBuffer>>> m_rightArmFrontColor = LEDModes.rainbow;
  private Function<Integer, Function<Integer, Function<AddressableLEDBuffer, AddressableLEDBuffer>>> m_rightArmBackColor = LEDModes.rainbow;
  private Function<Integer, Function<Integer, Function<AddressableLEDBuffer, AddressableLEDBuffer>>> m_leftArmFrontColor = LEDModes.rainbow;
  private Function<Integer, Function<Integer, Function<AddressableLEDBuffer, AddressableLEDBuffer>>> m_leftArmBackColor = LEDModes.rainbow;

  public static LEDs getInstance() {
    if (m_instance == null) {
      m_instance = new LEDs();
    }
    return m_instance;
  }

  private LEDs() {
    m_led = new AddressableLED(Constants.LEDs.k_PWMId);
    m_led.setLength(m_ledTotalLength);
    m_buffer = new AddressableLEDBuffer(m_ledTotalLength);
    m_led.start();
  }

  @Override
  public void periodic() {
    setRightArmColorMode();
    setLeftArmColorMode();
    setDriveColorMode();

    m_led.setData(m_buffer);
  }

  public void setColor(Color color) {
    setArmRightColor(LEDModes.setColor(color));
    setArmLeftColor(LEDModes.setColor(color));
  }

  public void setArmLeftColor(Color color) {
    m_leftArmColor = LEDModes.setColor(color);
    m_leftArmFrontColor = LEDModes.setColor(color);
    m_leftArmBackColor = LEDModes.setColor(color);
  }

  public void setArmLeftColor(
      Function<Integer, Function<Integer, Function<AddressableLEDBuffer, AddressableLEDBuffer>>> colorMode) {
    m_leftArmColor = colorMode;
    m_leftArmFrontColor = colorMode;
    m_leftArmBackColor = colorMode;
  }

  public void setArmLeftColor(Color frontColor, Color topColor, Color backColor) {
    m_leftArmColor = LEDModes.setColor(topColor);
    m_leftArmFrontColor = LEDModes.setColor(frontColor);
    m_leftArmBackColor = LEDModes.setColor(backColor);
  }

  public void setArmRightColor(Color color) {
    m_rightArmColor = LEDModes.setColor(color);
    m_rightArmFrontColor = LEDModes.setColor(color);
    m_rightArmBackColor = LEDModes.setColor(color);
  }

  public void setArmRightColor(
      Function<Integer, Function<Integer, Function<AddressableLEDBuffer, AddressableLEDBuffer>>> colorMode) {
    m_rightArmColor = colorMode;
    m_rightArmFrontColor = colorMode;
    m_rightArmBackColor = colorMode;
  }

  public void setArmRightColor(Color frontColor, Color topColor, Color backColor) {
    m_rightArmColor = LEDModes.setColor(topColor);
    m_rightArmFrontColor = LEDModes.setColor(frontColor);
    m_rightArmBackColor = LEDModes.setColor(backColor);
  }

  public void setDriveColor(Color color) {
    m_driveColor = LEDModes.setColor(color);
  }

  public void chase() {
    m_rightArmColor = LEDModes.redChase;
    m_leftArmColor = LEDModes.redChase;
  }

  public void breathe() {
    m_rightArmColor = LEDModes.redBreathe;
    m_leftArmColor = LEDModes.redBreathe;
  }

  public void setRightArmColorMode() {
    // Main color
    // m_buffer = m_rightArmColor
    // .apply(Constants.LEDs.ArmRight.k_start)
    // .apply(Constants.LEDs.ArmRight.k_length)
    // .apply(m_buffer);

    // // Front color
    // m_buffer = m_rightArmFrontColor
    // .apply(Constants.LEDs.ArmRight.k_start)
    // .apply(Constants.LEDs.k_sideLength)
    // .apply(m_buffer);

    // // Back color
    // m_buffer = m_rightArmBackColor
    // .apply(Constants.LEDs.ArmRight.k_backTop + Constants.LEDs.k_sideLengthOffset)
    // .apply(Constants.LEDs.k_sideLength)
    // .apply(m_buffer);
  }

  public void setLeftArmColorMode() {
    // Main color
    // m_buffer = m_leftArmColor
    // .apply(Constants.LEDs.ArmLeft.k_start)
    // .apply(Constants.LEDs.ArmLeft.k_length)
    // .apply(m_buffer);

    // // Front color
    // m_buffer = m_leftArmFrontColor
    // .apply(Constants.LEDs.ArmLeft.k_start)
    // .apply(Constants.LEDs.k_sideLength)
    // .apply(m_buffer);

    // // Back color
    // m_buffer = m_leftArmBackColor
    // .apply(Constants.LEDs.ArmLeft.k_backTop + Constants.LEDs.k_sideLengthOffset)
    // .apply(Constants.LEDs.k_sideLength)
    // .apply(m_buffer);
  }

  public void setDriveColorMode() {
    // m_buffer =
    // m_driveColor.apply(Constants.LEDs.Drive.k_start).apply(Constants.LEDs.Drive.k_length).apply(m_buffer);
  }

  @Override
  public void stop() {
  }

  @Override
  public void reset() {
  }

  @Override
  public void writePeriodicOutputs() {
  }

  @Override
  public void outputTelemetry() {
  }
}
