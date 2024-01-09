package frc.robot.autonomous.tasks;

import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake.IntakeState;

public class IntakeTask extends Task {
  private Intake m_intake;
  private IntakeState m_state;

  public IntakeTask(IntakeState intakeState) {
    m_intake = Intake.getInstance();
    m_state = intakeState;
  }

  @Override
  public void start() {
    m_intake.setState(m_state);
  }

  @Override
  public void update() {
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
