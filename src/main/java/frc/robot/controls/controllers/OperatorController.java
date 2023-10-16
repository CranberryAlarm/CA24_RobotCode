package frc.robot.controls.controllers;

public class OperatorController extends FilteredController {

  public OperatorController(int port) {
    super(port, false, false);
  }

  public OperatorController(int port, boolean useDeadband, boolean useSquaredInput) {
    super(port, useDeadband, useSquaredInput);
  }

  // Axis
  private final double k_triggerActivationThreshold = 0.5;

  public double getHorizontalChange(double strength) {
    return -this.getFilteredAxis(0) * strength;
  }

  public double getVerticalChange(double strength) {
    return -this.getFilteredAxis(5) * strength;
  }

  public boolean getWantsGripToggle() {
    return this.getRawButtonPressed(5);
  }
  
  public boolean getWantsClockwise() {
    return this.getFilteredAxis(3) > k_triggerActivationThreshold;
  }

  public boolean getWantsCounterClockwise() {
    return this.getFilteredAxis(2) > k_triggerActivationThreshold;
  }

  // Buttons
  public boolean getWantsGroundPickup() {
    return this.getRawButtonPressed(1);
  }

  public boolean getWantsHome() {
    return this.getRawButtonPressed(3);
  }

  public boolean getWantsLowScore() {
    return this.getHatDownPressed();
  }

  public boolean getWantsMidScore() {
    return this.getHatRightPressed();
  }

  public boolean getWantsHighScore() {
    return this.getHatUpPressed();
  }

  public boolean getWantsGroundScore() {
    return this.getHatLeftPressed();
  }

  public boolean getWantsSomething1() {
    return this.getRawButton(8);
  }

  public boolean getWantsSomething2() {
    return this.getRawButtonPressed(6);
  }
}
