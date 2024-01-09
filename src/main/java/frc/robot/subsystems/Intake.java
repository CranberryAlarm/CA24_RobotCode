package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Helpers;

public class Intake extends Subsystem {
  private static final double k_pivotMotorP = 0.12;
  private static final double k_pivotMotorI = 0.0;
  private static final double k_pivotMotorD = 0.001;

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
    mIntakeMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);

    mPivotMotor = new CANSparkMax(Constants.Intake.kPivotMotorId, MotorType.kBrushless);
    mPivotMotor.restoreFactoryDefaults();
    mPivotMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    mPivotMotor.setSmartCurrentLimit(30);

    m_periodicIO = new PeriodicIO();
  }

  private static class PeriodicIO {
    // Automated control
    // PivotTarget pivot_target = PivotTarget.SOURCE;
    PivotTarget pivot_target = PivotTarget.STOW;
    IntakeState intake_state = IntakeState.NONE;

    // Manual control
    double intake_pivot_power = 0.0;
    double intake_power = 0.0;
  }

  public enum PivotTarget {
    NONE,
    GROUND,
    SOURCE,
    AMP,
    STOW
  }

  public enum IntakeState {
    NONE,
    INTAKE,
    EJECT,
    PULSE,
    FEED_SHOOTER,
  }

  /*-------------------------------- Generic Subsystem Functions --------------------------------*/

  @Override
  public void periodic() {
    checkAutoTasks();

    // Pivot control
    double pivot_angle = pivotTargetToAngle(m_periodicIO.pivot_target);
    m_periodicIO.intake_pivot_power = m_pivotPID.calculate(getPivotAngleDegrees(), pivot_angle);

    // Intake control
    m_periodicIO.intake_power = intakeStateToSpeed(m_periodicIO.intake_state);
    SmartDashboard.putString("Intake State:", m_periodicIO.intake_state.toString());
  }

  @Override
  public void writePeriodicOutputs() {
    mPivotMotor.setVoltage(m_periodicIO.intake_pivot_power);

    mIntakeMotor.set(m_periodicIO.intake_power);
  }

  @Override
  public void stop() {
    m_periodicIO.intake_pivot_power = 0.0;
    m_periodicIO.intake_power = 0.0;
  }

  @Override
  public void outputTelemetry() {
    SmartDashboard.putNumber("Intake speed:", intakeStateToSpeed(m_periodicIO.intake_state));
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

  public double intakeStateToSpeed(IntakeState state) {
    switch (state) {
      case INTAKE:
        return Constants.Intake.k_intakeSpeed;
      case EJECT:
        return Constants.Intake.k_ejectSpeed;
      case PULSE:
        // Use the timer to pulse the intake on for a 1/16 second,
        // then off for a 15/16 second
        if (Timer.getFPGATimestamp() % 1.0 < (1.0 / 45.0)) {
          return Constants.Intake.k_intakeSpeed;
        }
        return 0.0;
      case FEED_SHOOTER:
        return Constants.Intake.k_feedShooterSpeed;
      default:
        // "Safe" default
        return 0.0;
    }
  }

  /*---------------------------------- Custom Public Functions ----------------------------------*/

  public IntakeState getIntakeState() {
    return m_periodicIO.intake_state;
  }

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

  // Pivot helper functions
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

  // Intake helper functions
  public void intake() {
    m_periodicIO.intake_state = IntakeState.INTAKE;
  }

  public void eject() {
    m_periodicIO.intake_state = IntakeState.EJECT;
  }

  public void pulse() {
    m_periodicIO.intake_state = IntakeState.PULSE;
  }

  public void feedShooter() {
    m_periodicIO.intake_state = IntakeState.FEED_SHOOTER;
  }

  public void stopIntake() {
    m_periodicIO.intake_state = IntakeState.NONE;
    m_periodicIO.intake_power = 0.0;
  }

  public void setState(IntakeState state) {
    m_periodicIO.intake_state = state;
  }

  /*---------------------------------- Custom Private Functions ---------------------------------*/
  private void checkAutoTasks() {
    // If the intake is set to GROUND, and the intake has a note, and the pivot is
    // close to it's target
    // Stop the intake and go to the SOURCE position
    if (m_periodicIO.pivot_target == PivotTarget.GROUND && getIntakeHasNote() && isPivotAtTarget()) {
      // m_periodicIO.pivot_target = PivotTarget.STOW;
      m_periodicIO.pivot_target = PivotTarget.SOURCE;
    }
  }

  private boolean isPivotAtTarget() {
    return Math.abs(getPivotAngleDegrees() - pivotTargetToAngle(m_periodicIO.pivot_target)) < 5;
  }
}
