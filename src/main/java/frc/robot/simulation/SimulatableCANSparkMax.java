package frc.robot.simulation;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;

public class SimulatableCANSparkMax extends CANSparkMax {
  SimDeviceSim mCANSparkMaxSim;

  SimDouble mCANSparkMaxSimAppliedOutput;

  public SimulatableCANSparkMax(int deviceId, MotorType type) {
    super(deviceId, type);

    mCANSparkMaxSim = new SimDeviceSim("SPARK MAX ", deviceId);
    mCANSparkMaxSimAppliedOutput = mCANSparkMaxSim.getDouble("Applied Output");

    // TODO: Add other simulation fields
  }

  @Override
  public void set(double speed) {
    super.set(speed);

    // TODO: Figure out why this is mad when running on a real robot
    // mCANSparkMaxSimAppliedOutput.set(speed);
  }
}
