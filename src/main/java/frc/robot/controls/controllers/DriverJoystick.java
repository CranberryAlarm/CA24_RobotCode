package frc.robot.controls.controllers;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriverJoystick extends FilteredController {
  private String m_smartDashboardKey = "DriverInput/";

  public DriverJoystick(int port) {
    super(port, false, false);
  }

  public DriverJoystick(int port, boolean useDeadband, boolean useSquaredInput) {
    super(port, useDeadband, useSquaredInput);
  }

  private final double k_triggerActivationThreshold = 0.5;

  // Drive
  public double getForwardAxis() {
    return -this.getFilteredAxis(1);
  }

  public double getStrafeAxis() {
    return -this.getFilteredAxis(0);
  }

  public double getTurnAxis() {
    return -this.getFilteredAxis(2);
  }

  public boolean getWantsSlowMode() {
    return this.getFilteredAxis(3) > k_triggerActivationThreshold;
  }

  // Intake
  /*
   * public boolean getWantsIntakeOpen() {
   * return this.getLeftBumper();
   * }
   *
   * public boolean getWantsIntakeClose() {
   * return this.getRightBumper();
   * }
   */

  public void outputTelemetry() {
    SmartDashboard.putNumber(m_smartDashboardKey + "Forward", getForwardAxis());
    SmartDashboard.putNumber(m_smartDashboardKey + "Strafe", getStrafeAxis());
    SmartDashboard.putNumber(m_smartDashboardKey + "Turn", getTurnAxis());
  }
}
