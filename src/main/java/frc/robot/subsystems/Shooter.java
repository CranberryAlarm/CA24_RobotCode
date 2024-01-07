package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class Shooter extends Subsystem {

  /*-------------------------------- Private instance variables ---------------------------------*/
  private static Shooter mInstance;
  private PeriodicIO mPeriodicIO;

  public static Shooter getInstance() {
    if (mInstance == null) {
      mInstance = new Shooter();
    }
    return mInstance;
  }

  private CANSparkFlex mLeftShooterMotor;
  private CANSparkFlex mRightShooterMotor;

  private Shooter() {
    mPeriodicIO = new PeriodicIO();

    mLeftShooterMotor = new CANSparkFlex(Constants.kShooterLeftMotorId, MotorType.kBrushless);
    mRightShooterMotor = new CANSparkFlex(Constants.kShooterRightMotorId, MotorType.kBrushless);

    mLeftShooterMotor.setIdleMode(CANSparkFlex.IdleMode.kCoast);
    mRightShooterMotor.setIdleMode(CANSparkFlex.IdleMode.kCoast);

    mLeftShooterMotor.setInverted(true);
  }

  private static class PeriodicIO {
    double shooter_speed = 0.0;
  }

  /*-------------------------------- Generic Subsystem Functions --------------------------------*/

  @Override
  public void periodic() {
  }

  @Override
  public void writePeriodicOutputs() {
    mLeftShooterMotor.set(mPeriodicIO.shooter_speed);
    mRightShooterMotor.set(mPeriodicIO.shooter_speed);
  }

  @Override
  public void stop() {
  }

  @Override
  public void outputTelemetry() {
    SmartDashboard.putNumber("Shooter speed:", mPeriodicIO.shooter_speed);
  }

  @Override
  public void reset() {
  }

  /*---------------------------------- Custom Public Functions ----------------------------------*/

  public void setSpeed(double speed) {
    mPeriodicIO.shooter_speed = speed;
  }

  public void stopShooter() {
    mPeriodicIO.shooter_speed = 0.0;
  }

  /*---------------------------------- Custom Private Functions ---------------------------------*/
}
