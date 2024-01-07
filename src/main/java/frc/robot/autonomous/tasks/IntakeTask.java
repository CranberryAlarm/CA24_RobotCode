package frc.robot.autonomous.tasks;

import frc.robot.subsystems.Intake;

public class IntakeTask extends Task {
  private Intake m_intake;
  private double m_intakeSpeed;

  public IntakeTask(double intakeSpeed) {
    m_intake = Intake.getInstance();
    m_intakeSpeed = intakeSpeed;
  }

  @Override
  public void start() {
    m_intake.setSpeed(m_intakeSpeed);
  }

  @Override
  public void update() {
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
