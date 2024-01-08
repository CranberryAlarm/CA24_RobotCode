package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class Intake extends Subsystem {

  /*-------------------------------- Private instance variables ---------------------------------*/
  private static Intake mInstance;
  private PeriodicIO mPeriodicIO;

  public static Intake getInstance() {
    if (mInstance == null) {
      mInstance = new Intake();
    }
    return mInstance;
  }

  private CANSparkFlex mIntakeMotor;

  private Intake() {
    mIntakeMotor = new CANSparkFlex(Constants.kIntakeMotorId, MotorType.kBrushless);

    mIntakeMotor.setInverted(true);

    mIntakeMotor.setIdleMode(CANSparkFlex.IdleMode.kCoast);

    mPeriodicIO = new PeriodicIO();
  }

  private static class PeriodicIO {
    double intake_speed = 0.0;
  }

  /*-------------------------------- Generic Subsystem Functions --------------------------------*/

  @Override
  public void periodic() {
  }

  @Override
  public void writePeriodicOutputs() {
    // TODO: Add intkae pivot motor

    // TODO: Add intake limit switch

    mIntakeMotor.set(mPeriodicIO.intake_speed);
  }

  @Override
  public void stop() {
    stopIntake();
  }

  @Override
  public void outputTelemetry() {
    SmartDashboard.putNumber("Intake speed:", mPeriodicIO.intake_speed);
  }

  @Override
  public void reset() {
  }

  /*---------------------------------- Custom Public Functions ----------------------------------*/

  public void intake() {
    mPeriodicIO.intake_speed = Constants.Intake.k_intakeSpeed;
  }

  public void eject() {
    mPeriodicIO.intake_speed = Constants.Intake.k_ejectSpeed;
  }

  public void feedShooter() {
    mPeriodicIO.intake_speed = Constants.Intake.k_feedShooterSpeed;
  }

  public void setSpeed(double speed) {
    mPeriodicIO.intake_speed = speed;
  }

  public void stopIntake() {
    mPeriodicIO.intake_speed = 0.0;
  }

  /*---------------------------------- Custom Private Functions ---------------------------------*/
}
