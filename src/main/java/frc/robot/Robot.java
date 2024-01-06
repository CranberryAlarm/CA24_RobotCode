// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.subsystems.Compressor;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Subsystem;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  // Robot subsystems
  private List<Subsystem> m_allSubsystems = new ArrayList<>();
  private final Intake m_intake = Intake.getInstance();
  private final Compressor m_compressor = Compressor.getInstance();
  private final Drivetrain m_drivetrain = Drivetrain.getInstance();

  /**
   * This function is run when the robot is first started up.
   */
  @Override
  public void robotInit() {
    m_allSubsystems.add(m_intake);
    m_allSubsystems.add(m_compressor);
    m_allSubsystems.add(m_drivetrain);
  }

  @Override
  public void robotPeriodic() {
    m_allSubsystems.forEach(subsystem -> subsystem.periodic());
    m_allSubsystems.forEach(subsystem -> subsystem.writePeriodicOutputs());
    m_allSubsystems.forEach(subsystem -> subsystem.outputTelemetry());
    m_allSubsystems.forEach(subsystem -> subsystem.writeToLog());
  }

  @Override
  public void autonomousInit() {}

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {}

  @Override
  public void disabledInit() {
    m_allSubsystems.forEach(subsystem -> subsystem.stop());
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}
