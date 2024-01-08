package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class Intake extends Subsystem {

  // New test PID values
  private static final double k_pivotMotorP = 1.0;
  private static final double k_pivotMotorI = 0.0;
  private static final double k_pivotMotorD = 0.0;

  private final PIDController m_pivotPID = new PIDController(k_pivotMotorP, k_pivotMotorI, k_pivotMotorD);

  private final DutyCycleEncoder m_pivotEncoder = new DutyCycleEncoder(Constants.Intake.k_pivotEncoderId);

  /*-------------------------------- Private instance variables ---------------------------------*/
  private static Intake mInstance;
  private PeriodicIO mPeriodicIO;

  public static Intake getInstance() {
    if (mInstance == null) {
      mInstance = new Intake();
    }
    return mInstance;
  }

  private CANSparkMax mIntakeMotor;
  private CANSparkMax mPivotMotor;

  private Intake() {
    mIntakeMotor = new CANSparkMax(Constants.Intake.kIntakeMotorId, MotorType.kBrushless);
    mIntakeMotor.restoreFactoryDefaults();
    // mIntakeMotor.setInverted(true);
    mIntakeMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);

    mPivotMotor = new CANSparkMax(Constants.Intake.kPivotMotorId, MotorType.kBrushless);
    mPivotMotor.restoreFactoryDefaults();
    mPivotMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

    m_pivotPID.enableContinuousInput(0, 360);

    mPeriodicIO = new PeriodicIO();
  }

  private static class PeriodicIO {
    // Automated control
    double pivot_angle = 0.0;

    // Manual control
    double intake_speed = 0.0;
    double pivot_speed = 0.0;
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
