package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.SparkLimitSwitch.Type;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.simulation.SimulatableCANSparkMax;

public class Elevator extends Subsystem {
  private static final double kPivotPowerOut = 1.0;
  private static final double kPivotPowerIn = -0.7;
  private static final double kExtensionPowerOut = 0.6;
  private static final double kExtensionPowerIn = -0.6;
  private static final double kPivotAntiBoostAmount = 19.5;
  private static final double kPivotBoost2Amount = -15;

  private static final double kPivotCLRampRate = 0.5;
  private static final double kExtensionCLRampRate = 0.5;

  private static Elevator mInstance;

  public static Elevator getInstance() {
    if (mInstance == null) {
      mInstance = new Elevator();
    }
    return mInstance;
  }

  private CANSparkMax mPivotMotor;
  private RelativeEncoder mPivotEncoder;
  private SparkPIDController mPivotPIDController;
  private SparkLimitSwitch mPivotLowerLimit;

  private SimulatableCANSparkMax mExtensionMotor;
  private RelativeEncoder mExtensionEncoder;
  private SparkPIDController mExtensionPIDController;
  private SparkLimitSwitch mExtensionLowerLimit;
  private SparkLimitSwitch mExtensionUpperLimit;

  private PeriodicIO mPeriodicIO = new PeriodicIO();

  private Elevator() {
    mPivotMotor = new CANSparkMax(Constants.kElevatorPivotMotorId, MotorType.kBrushless);
    mPivotMotor.restoreFactoryDefaults();
    mPivotPIDController = mPivotMotor.getPIDController();
    mPivotEncoder = mPivotMotor.getEncoder();

    mPivotPIDController.setP(0.1);
    mPivotPIDController.setI(1e-8);
    mPivotPIDController.setD(1);
    mPivotPIDController.setIZone(0);
    mPivotPIDController.setFF(0);
    mPivotPIDController.setOutputRange(kPivotPowerIn, kPivotPowerOut);
    mPivotMotor.setClosedLoopRampRate(kPivotCLRampRate);

    mPivotLowerLimit = mPivotMotor.getForwardLimitSwitch(Type.kNormallyOpen);

    mExtensionMotor = new SimulatableCANSparkMax(Constants.kElevatorExtensionMotorId, MotorType.kBrushless);
    mExtensionMotor.restoreFactoryDefaults();
    mExtensionMotor.setIdleMode(IdleMode.kBrake);
    mExtensionMotor.setInverted(true);
    mExtensionPIDController = mExtensionMotor.getPIDController();
    mExtensionEncoder = mExtensionMotor.getEncoder();

    mExtensionPIDController.setP(0.1);
    mExtensionPIDController.setI(1e-8);
    mExtensionPIDController.setD(1);
    mExtensionPIDController.setIZone(0);
    mExtensionPIDController.setFF(0);
    mExtensionPIDController.setOutputRange(kExtensionPowerIn, kExtensionPowerOut);
    mExtensionMotor.setClosedLoopRampRate(kExtensionCLRampRate);

    mExtensionLowerLimit = mExtensionMotor.getReverseLimitSwitch(Type.kNormallyOpen);
    mExtensionUpperLimit = mExtensionMotor.getForwardLimitSwitch(Type.kNormallyOpen);

    mPeriodicIO = new PeriodicIO();
  }

  private static class PeriodicIO {
    double pivot_power = 0.0;
    double pivot_target = 0.0;
    boolean is_pivot_pos_control = false;
    boolean is_pivot_anti_boosted = false;
    boolean is_pivot_boosted2 = false;

    double extension_power = 0.0;
    double extension_target = 0.0;
    boolean is_extension_pos_control = false;
  }

  public void extend() {
    mPeriodicIO.is_extension_pos_control = false;
    mPeriodicIO.extension_power = kExtensionPowerOut;
  }

  public void retract() {
    mPeriodicIO.is_extension_pos_control = false;
    mPeriodicIO.extension_power = kExtensionPowerIn;
  }

  public void raise() {
    mPeriodicIO.is_pivot_pos_control = false;
    mPeriodicIO.pivot_power = kPivotPowerIn;
  }

  public void lower() {
    mPeriodicIO.is_pivot_pos_control = false;
    mPeriodicIO.pivot_power = kPivotPowerOut;
  }

  public void goToPivotGround() {
    mPeriodicIO.is_pivot_pos_control = true;
    mPeriodicIO.pivot_target = Constants.kPivotGroundCount;
  }

  public void goToPivotScore() {
    mPeriodicIO.is_pivot_pos_control = true;
    mPeriodicIO.pivot_target = Constants.kPivotScoreCount;
  }

  public void goToPivotPreScore() {
    mPeriodicIO.is_pivot_pos_control = true;
    mPeriodicIO.pivot_target = Constants.kPivotPreScoreCount;
  }

  public void goToPivotStow() {
    mPeriodicIO.is_pivot_pos_control = true;
    mPeriodicIO.pivot_target = Constants.kPivotStowCount;
  }

  public void goToExtensionStow() {
    mPeriodicIO.is_extension_pos_control = true;
    mPeriodicIO.extension_target = Constants.kExtensionStowCount;
  }

  public void goToExtensionMidGoal() {
    mPeriodicIO.is_extension_pos_control = true;
    mPeriodicIO.extension_target = Constants.kExtensionMidGoalCount;
  }

  public void goToExtensionHighGoal() {
    mPeriodicIO.is_extension_pos_control = true;
    mPeriodicIO.extension_target = Constants.kExtensionHighGoalCount;
  }

  public void antiBoostPivot(boolean boost) {
    mPeriodicIO.is_pivot_anti_boosted = boost;
  }

  public void boostPivot2(boolean boost) {
    mPeriodicIO.is_pivot_boosted2 = boost;
  }

  @Override
  public void periodic() {
    writePeriodicOutputs();
  }

  @Override
  public void writePeriodicOutputs() {
    if (mPeriodicIO.is_pivot_pos_control) {
      if (mPeriodicIO.is_pivot_anti_boosted) {
        mPivotPIDController.setReference(mPeriodicIO.pivot_target + kPivotAntiBoostAmount,
            CANSparkMax.ControlType.kPosition);
      } else if (mPeriodicIO.is_pivot_boosted2) {
        mPivotPIDController.setReference(mPeriodicIO.pivot_target + kPivotBoost2Amount,
            CANSparkMax.ControlType.kPosition);
      } else {
        mPivotPIDController.setReference(mPeriodicIO.pivot_target,
            CANSparkMax.ControlType.kPosition);
      }
    } else {
      mPivotMotor.set(mPeriodicIO.pivot_power);
    }

    if (mPeriodicIO.is_extension_pos_control) {
      mExtensionPIDController.setReference(mPeriodicIO.extension_target, CANSparkMax.ControlType.kPosition);
    } else {
      mExtensionMotor.set(mPeriodicIO.extension_power);
    }
  }

  @Override
  public void stop() {
    stopPivot();
    stopExtension();
  }

  public void stopPivot() {
    if (!mPeriodicIO.is_pivot_pos_control) {
      mPeriodicIO.pivot_power = 0.0;
      mPivotMotor.set(0.0);
    }
  }

  public void stopExtension() {
    if (!mPeriodicIO.is_extension_pos_control) {
      mPeriodicIO.extension_power = 0.0;
      mExtensionMotor.set(0.0);
    }
  }

  public void resetPivotEncoder() {
    mPivotEncoder.setPosition(0);
  }

  public void resetExtensionEncoder() {
    mExtensionEncoder.setPosition(0);
  }

  @Override
  public void outputTelemetry() {
    // Pivot telemetry
    SmartDashboard.putNumber("Pivot motor power:", mPeriodicIO.pivot_power);
    SmartDashboard.putNumber("Pivot encoder count:", mPivotEncoder.getPosition());
    SmartDashboard.putNumber("Pivot PID target:", mPeriodicIO.pivot_power);
    SmartDashboard.putBoolean("Pivot lower limit:", mPivotLowerLimit.isPressed());

    // Extension telemetry
    SmartDashboard.putNumber("Extension motor power:", mPeriodicIO.extension_power);
    SmartDashboard.putNumber("Extension encoder count:", mExtensionEncoder.getPosition());
    SmartDashboard.putBoolean("Extension lower limit:", mExtensionLowerLimit.isPressed());
    SmartDashboard.putBoolean("Extension Upper limit:", mExtensionUpperLimit.isPressed());
  }

  @Override
  public void reset() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'reset'");
  }
}
