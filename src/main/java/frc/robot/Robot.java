// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.autonomous.AutoChooser;
import frc.robot.autonomous.AutoRunner;
import frc.robot.autonomous.tasks.Task;
import frc.robot.controls.controllers.DriverController;
import frc.robot.controls.controllers.OperatorController;
import frc.robot.simulation.Field;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Compressor;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Subsystem;
import frc.robot.subsystems.leds.LEDs;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  // Controller
  private final DriverController m_driverController = new DriverController(0, true, true);
  private final OperatorController m_operatorController = new OperatorController(1, true, true);

  private final SlewRateLimiter m_speedLimiter = new SlewRateLimiter(3); // 3 seconds to go from 0.0 to 1.0
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3); // 3 seconds to go from 0.0 to 1.0

  // Robot subsystems
  private List<Subsystem> m_allSubsystems = new ArrayList<>();
  private final Intake m_intake = Intake.getInstance();
  private final Compressor m_compressor = Compressor.getInstance();
  private final Drivetrain m_drive = Drivetrain.getInstance();
  private final Shooter m_shooter = Shooter.getInstance();
  private final Climber m_climber = Climber.getInstance();
  public final LEDs m_leds = LEDs.getInstance();

  // Auto stuff
  private Task m_currentTask;
  private AutoRunner m_autoRunner = AutoRunner.getInstance();
  private AutoChooser m_autoChooser = new AutoChooser();

  private final Field m_field = Field.getInstance();

  /**
   * This function is run when the robot is first started up.
   */
  @Override
  public void robotInit() {
    m_allSubsystems.add(m_intake);
    m_allSubsystems.add(m_compressor);
    m_allSubsystems.add(m_drive);
    m_allSubsystems.add(m_shooter);
    m_allSubsystems.add(m_climber);
    m_allSubsystems.add(m_leds);

    // Set up the Field2d object for simulation
    SmartDashboard.putData("Field", m_field);
  }

  @Override
  public void robotPeriodic() {
    m_allSubsystems.forEach(subsystem -> subsystem.periodic());
    m_allSubsystems.forEach(subsystem -> subsystem.writePeriodicOutputs());
    m_allSubsystems.forEach(subsystem -> subsystem.outputTelemetry());
    m_allSubsystems.forEach(subsystem -> subsystem.writeToLog());

    // Update the odometry in the sim.
    m_field.setRobotPose(m_drive.getPose());
  }

  @Override
  public void autonomousInit() {
    m_autoRunner.setAutoMode(m_autoChooser.getSelectedAuto());
    m_currentTask = m_autoRunner.getNextTask();

    // Start the first task
    if (m_currentTask != null) {
      m_currentTask.start();
    }
  }

  @Override
  public void autonomousPeriodic() {
    // If there is a current task, run it
    if (m_currentTask != null) {
      // Run the current task
      m_currentTask.update();
      m_currentTask.updateSim();

      // If the current task is finished, get the next task
      if (m_currentTask.isFinished()) {
        m_currentTask.done();
        m_currentTask = m_autoRunner.getNextTask();

        // Start the next task
        if (m_currentTask != null) {
          m_currentTask.start();
        }
      }
    }
  }

  @Override
  public void teleopInit() {
  }

  double speed = 0;

  @Override
  public void teleopPeriodic() {
    // Get the x speed. We are inverting this because Xbox controllers return
    // negative values when we push forward.
    double xSpeed = m_speedLimiter.calculate(-m_driverController.getForwardAxis()) *
        Drivetrain.kMaxSpeed;

    // Get the rate of angular rotation. We are inverting this because we want a
    // positive value when we pull to the left (remember, CCW is positive in
    // mathematics). Xbox controllers return positive values when you pull to
    // the right by default.
    m_drive.slowMode(m_driverController.getWantsSlowMode());
    m_drive.speedMode(m_driverController.getWantsSpeedMode());
    double rot = m_rotLimiter.calculate(m_driverController.getTurnAxis()) *
        Drivetrain.kMaxAngularSpeed;

    m_drive.drive(xSpeed, rot);

    // Shooter variable speed
    if (m_driverController.getWantsMoreSpeed() || m_operatorController.getWantsMoreSpeed()) {
      m_leds.setColor(Color.kPink);
      speed = 10000;
    } else if (m_driverController.getWantsLessSpeed() || m_operatorController.getWantsLessSpeed()) {
      m_leds.setColor(Color.kOrange);
      speed = 430;
    } else if (m_driverController.getWantsShooterStop() || m_operatorController.getWantsShooterStop()) {
      m_leds.defaultLEDS();
      speed = 0;
    }
    speed = MathUtil.clamp(speed, -6000, 10000);
    m_shooter.setSpeed(speed);

    // Intake
    if (m_driverController.getWantsFullIntake()) {
      m_intake.goToGround();
    } else if (m_driverController.getWantsIntake()) {
      if (m_intake.getIntakeHasNote()) {
        m_intake.pulse();
      } else {
        m_intake.intake();
      }
    } else if (m_driverController.getWantsEject()) {
      m_intake.eject();
    } else if (m_driverController.getWantsSource()) {
      m_intake.goToSource();
    } else if (m_driverController.getWantsStow()) {
      m_intake.goToStow();
    } else {
      m_intake.stopIntake();
    }

    // Climber
    if (m_operatorController.getWantsClimberClimb()) {
      m_climber.climb();
    } else if (m_operatorController.getWantsClimberRelease()) {
      m_climber.release();
    } else if (m_operatorController.getWantsClimberTiltLeft()) {
      m_climber.tiltLeft();
    } else if (m_operatorController.getWantsClimberTiltRight()) {
      m_climber.tiltRight();
    } else {
      m_climber.stopClimber();
    }

    if (m_operatorController.getWantsBrakeMode()) {
      m_climber.setBrakeMode();
    } else if (m_operatorController.getWantsCoastMode()) {
      m_climber.setCoastMode();
    }
  }

  @Override
  public void disabledInit() {
    m_leds.rainbow();
    speed = 0;
    m_allSubsystems.forEach(subsystem -> subsystem.stop());
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void disabledExit() {
  }

  @Override
  public void testInit() {
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void simulationInit() {
  }

  @Override
  public void simulationPeriodic() {
  }
}
