package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Helpers;

public class Intake extends Subsystem {

  // New test PID values
  private static final double k_pivotMotorP = 0.04;
  private static final double k_pivotMotorI = 0.0;
  private static final double k_pivotMotorD = 0.0;

  private final PIDController m_pivotPID = new PIDController(k_pivotMotorP, k_pivotMotorI, k_pivotMotorD);

  private final DutyCycleEncoder m_pivotEncoder = new DutyCycleEncoder(Constants.Intake.k_pivotEncoderId);
  private final DigitalInput m_IntakeLimitSwitch = new DigitalInput(Constants.Intake.k_intakeLimitSwitchId);

  /*-------------------------------- Private instance variables ---------------------------------*/
  private static Intake mInstance;
  private PeriodicIO m_periodicIO;

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
    mPivotMotor.setSmartCurrentLimit(30);

    // m_pivotPID.enableContinuousInput(0, 360);

    // TODO; Figure out how this actually works
    // m_pivotEncoder.setPositionOffset(Constants.Intake.k_pivotEncoderOffset);

    m_periodicIO = new PeriodicIO();
  }

  private static class PeriodicIO {
    // Automated control
    PivotTarget pivot_target = PivotTarget.STOW;

    // Manual control
    double intake_speed = 0.0;
    double intake_pivot_power = 0.0;
  }

  public enum PivotTarget {
    NONE,
    GROUND,
    SOURCE,
    AMP,
    STOW
  }

  /*-------------------------------- Generic Subsystem Functions --------------------------------*/

  @Override
  public void periodic() {
    checkAutoTasks();

    // Set the pivot power based on the PID
    double pivot_angle = pivotTargetToAngle(m_periodicIO.pivot_target);
    m_periodicIO.intake_pivot_power = m_pivotPID.calculate(getPivotAngleDegrees(), pivot_angle);
  }

  @Override
  public void writePeriodicOutputs() {
    mPivotMotor.setVoltage(m_periodicIO.intake_pivot_power);

    // TODO: Add intake limit switch

    mIntakeMotor.set(m_periodicIO.intake_speed);
  }

  @Override
  public void stop() {
    stopIntake();
    m_periodicIO.intake_pivot_power = 0.0;
  }

  @Override
  public void outputTelemetry() {
    SmartDashboard.putNumber("Intake speed:", m_periodicIO.intake_speed);
    SmartDashboard.putNumber("Pivot Abs Enc (get):", m_pivotEncoder.get());
    SmartDashboard.putNumber("Pivot Abs Enc (getAbsolutePosition):", m_pivotEncoder.getAbsolutePosition());
    SmartDashboard.putNumber("Pivot Abs Enc (getPivotAngleDegrees):", getPivotAngleDegrees());
    SmartDashboard.putNumber("Pivot Setpoint:", pivotTargetToAngle(m_periodicIO.pivot_target));

    SmartDashboard.putNumber("Pivot Power:", m_periodicIO.intake_pivot_power);
    SmartDashboard.putNumber("Pivot Current:", mPivotMotor.getOutputCurrent());

    SmartDashboard.putBoolean("Intake Limit Switch:", getIntakeHasNote());
  }

  @Override
  public void reset() {
  }

  public double pivotTargetToAngle(PivotTarget target) {
    switch (target) {
      case GROUND:
        return Constants.Intake.k_pivotAngleGround;
      case SOURCE:
        return Constants.Intake.k_pivotAngleSource;
      case AMP:
        return Constants.Intake.k_pivotAngleAmp;
      case STOW:
        return Constants.Intake.k_pivotAngleStow;
      default:
        // "Safe" default
        return 180;
    }
  }

  /*---------------------------------- Custom Public Functions ----------------------------------*/

  public double getPivotAngleDegrees() {
    double value = m_pivotEncoder.getAbsolutePosition() -
        Constants.Intake.k_pivotEncoderOffset + 0.5;

    return Units.rotationsToDegrees(Helpers.modRotations(value));
  }

  public boolean getIntakeHasNote() {
    // NOTE: this is intentionally inverted, because the limit switch is normally
    // closed
    return !m_IntakeLimitSwitch.get();
  }

  public void intake() {
    m_periodicIO.intake_speed = Constants.Intake.k_intakeSpeed;
  }

  public void goToGround() {
    m_periodicIO.pivot_target = PivotTarget.GROUND;
  }

  public void goToSource() {
    m_periodicIO.pivot_target = PivotTarget.SOURCE;
  }

  public void goToAmp() {
    m_periodicIO.pivot_target = PivotTarget.SOURCE;
  }

  public void goToStow() {
    m_periodicIO.pivot_target = PivotTarget.STOW;
  }

  public void eject() {
    m_periodicIO.intake_speed = Constants.Intake.k_ejectSpeed;
  }

  public void feedShooter() {
    m_periodicIO.intake_speed = Constants.Intake.k_feedShooterSpeed;
  }

  public void setSpeed(double speed) {
    m_periodicIO.intake_speed = speed;
  }

  public void stopIntake() {
    m_periodicIO.intake_speed = 0.0;
  }

  /*---------------------------------- Custom Private Functions ---------------------------------*/
  private void checkAutoTasks() {
    // If the intake is set to GROUND, and the intake has a note, and the pivot is
    // close to it's target
    // Stop the intake and go to the SOURCE position
    if (m_periodicIO.pivot_target == PivotTarget.GROUND && getIntakeHasNote() && isPivotAtTarget()) {
      m_periodicIO.pivot_target = PivotTarget.SOURCE;
      stopIntake();
    }
  }

  private boolean isPivotAtTarget() {
    return Math.abs(getPivotAngleDegrees() - pivotTargetToAngle(m_periodicIO.pivot_target)) < 5;
  }
}
