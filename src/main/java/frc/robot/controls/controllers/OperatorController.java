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
  
  public boolean getWantsTriggerSomething1() {
    return this.getFilteredAxis(3) > k_triggerActivationThreshold;
  }

  public boolean getWantsTriggerSomething2() {
    return this.getFilteredAxis(2) > k_triggerActivationThreshold;
  }

  // Buttons
  
  // D pad

  public boolean getWantsClimberRelease() {
    return this.getHatDownPressed();
  }

  public boolean getWantsClimberTiltRight() {
    return this.getHatRightPressed();
  }

  public boolean getWantsClimberClimb() {
    return this.getHatUpPressed();
  }

  public boolean getWantsClimberTiltLeft() {
    return this.getHatLeftPressed();
  }
}
