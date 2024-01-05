package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
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

  private Solenoid mIntakeSolenoid;

  private Intake() {
    mIntakeSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.kIntakeSolenoidForwardId);

    mPeriodicIO = new PeriodicIO();
  }

  private static class PeriodicIO {
    boolean intake_solenoid_state = true;
  }

  /*-------------------------------- Generic Subsystem Functions --------------------------------*/
  
  @Override
  public void periodic() {}

  @Override
  public void writePeriodicOutputs() {
    mIntakeSolenoid.set(mPeriodicIO.intake_solenoid_state);
  }
  
  @Override
  public void stop() {
    mIntakeSolenoid.set(false);
  }
  
  @Override
  public void outputTelemetry() {
    SmartDashboard.putBoolean("Intake state:", mIntakeSolenoid.get());
  }
  
  @Override
  public void reset() {}
  
  /*---------------------------------- Custom Public Functions ----------------------------------*/

  public void open() {
    mPeriodicIO.intake_solenoid_state = false;
  }
  
  public void close() {
    mPeriodicIO.intake_solenoid_state = true;
  }

  /*---------------------------------- Custom Private Functions ---------------------------------*/
}
