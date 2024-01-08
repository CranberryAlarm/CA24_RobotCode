package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
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
    double pivot_angle = Constants.Intake.k_pivotAngleStow;

    // Manual control
    double intake_speed = 0.0;
    double intake_pivot_power = 0.0;
  }

  /*-------------------------------- Generic Subsystem Functions --------------------------------*/

  @Override
  public void periodic() {
    m_periodicIO.intake_pivot_power = m_pivotPID.calculate(getPivotAngleDegrees(), m_periodicIO.pivot_angle);
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

    // Set the target angle to the current angle
    // m_periodicIO.pivot_angle = getPivotAngleDegrees();
    m_periodicIO.intake_pivot_power = 0.0;
  }

  @Override
  public void outputTelemetry() {
    SmartDashboard.putNumber("Intake speed:", m_periodicIO.intake_speed);
    SmartDashboard.putNumber("Pivot Abs Enc (get):", m_pivotEncoder.get());
    SmartDashboard.putNumber("Pivot Abs Enc (getAbsolutePosition):", m_pivotEncoder.getAbsolutePosition());
    SmartDashboard.putNumber("Pivot Abs Enc (getPivotAngleDegrees):", getPivotAngleDegrees());
    SmartDashboard.putNumber("Pivot Setpoint:", m_periodicIO.pivot_angle);

    SmartDashboard.putNumber("Pivot Power:", m_periodicIO.intake_pivot_power);
    SmartDashboard.putNumber("Pivot Current:", mPivotMotor.getOutputCurrent());
  }

  @Override
  public void reset() {
  }

  /*---------------------------------- Custom Public Functions ----------------------------------*/

  public double getPivotAngleDegrees() {
    double value = m_pivotEncoder.getAbsolutePosition() -
        Constants.Intake.k_pivotEncoderOffset + 0.5;

    // double value = m_pivotEncoder.get();

    return Units.rotationsToDegrees(Helpers.modRotations(value));
  }

  public void intake() {
    m_periodicIO.intake_speed = Constants.Intake.k_intakeSpeed;
  }

  public void deploy() {
    m_periodicIO.pivot_angle = Constants.Intake.k_pivotAngleGround;
  }

  public void stow() {
    m_periodicIO.pivot_angle = Constants.Intake.k_pivotAngleStow;
  }

  public void source() {
    m_periodicIO.pivot_angle = Constants.Intake.k_pivotAngleSource;
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
}
