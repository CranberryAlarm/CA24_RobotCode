// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.ReplanningConfig;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.simulation.Field;
import frc.robot.simulation.SimulatableCANSparkMax;

public class Drivetrain extends Subsystem {
  // 1 meters per second.
  public static final double kMaxSpeed = 2.0;
  public static final double kMaxBoostSpeed = 4.0;

  // 3 meters per second.
  public static final double kMaxAcceleration = 8.0;

  // 0.7 rotations per second.
  public static final double kMaxAngularSpeed = Math.PI * 1.0;

  private static final double kSlowModeRotScale = 0.1;
  private static final double kSpeedModeScale = 2.0;
  private static final double kTrackWidth = Units.inchesToMeters(22.0);
  private static final double kWheelRadius = Units.inchesToMeters(3.0);
  private static final double kGearRatio = 10.71;
  private static final double kMetersPerRev = (2.0 * Math.PI * kWheelRadius) / kGearRatio;

  private final SimulatableCANSparkMax mLeftLeader = new SimulatableCANSparkMax(Constants.Drive.kFLMotorId,
      MotorType.kBrushless);
  private final SimulatableCANSparkMax mLeftFollower = new SimulatableCANSparkMax(Constants.Drive.kBLMotorId,
      MotorType.kBrushless);
  private final SimulatableCANSparkMax mRightLeader = new SimulatableCANSparkMax(Constants.Drive.kFRMotorId,
      MotorType.kBrushless);
  private final SimulatableCANSparkMax mRightFollower = new SimulatableCANSparkMax(Constants.Drive.kBRMotorId,
      MotorType.kBrushless);

  private final MotorControllerGroup mLeftGroup = new MotorControllerGroup(mLeftLeader, mLeftFollower);
  private final MotorControllerGroup mRightGroup = new MotorControllerGroup(mRightLeader, mRightFollower);

  private final RelativeEncoder mLeftEncoder;
  private final RelativeEncoder mRightEncoder;

  private final PIDController mLeftPIDController = new PIDController(Constants.Drive.kP, Constants.Drive.kI,
      Constants.Drive.kD);
  private final PIDController mRightPIDController = new PIDController(Constants.Drive.kP, Constants.Drive.kI,
      Constants.Drive.kD);

  private final AHRS mGyro = new AHRS();

  private final DifferentialDriveKinematics mKinematics = new DifferentialDriveKinematics(kTrackWidth);

  private final DifferentialDriveOdometry mOdometry;

  private final SimpleMotorFeedforward mLeftFeedforward = new SimpleMotorFeedforward(Constants.Drive.kS,
      Constants.Drive.kV);
  private final SimpleMotorFeedforward mRightFeedforward = new SimpleMotorFeedforward(Constants.Drive.kS,
      Constants.Drive.kV);

  /*********
   * SysId *
   *********/

  // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
  private final MutableMeasure<Voltage> m_appliedVoltage = mutable(Volts.of(0));
  // Mutable holder for unit-safe linear distance values, persisted to avoid
  // reallocation.
  private final MutableMeasure<Distance> m_distance = mutable(Meters.of(0));
  // Mutable holder for unit-safe linear velocity values, persisted to avoid
  // reallocation.
  private final MutableMeasure<Velocity<Distance>> m_velocity = mutable(MetersPerSecond.of(0));

  private final SysIdRoutine mSysIdRoutine;

  // Simulation classes help us simulate our robot
  // private final AnalogGyroSim mGyroSim = new AnalogGyroSim(mgyro);
  // private final EncoderSim mLeftEncoderSim = new EncoderSim(mleftEncoder);
  // private final EncoderSim mRightEncoderSim = new EncoderSim(mrightEncoder);
  private final Field m_field = Field.getInstance();
  private final LinearSystem<N2, N2, N2> mDrivetrainSystem = LinearSystemId.identifyDrivetrainSystem(1.98, 0.2, 1.5,
      0.3);
  private final DifferentialDrivetrainSim mDrivetrainSimulator = new DifferentialDrivetrainSim(
      mDrivetrainSystem, DCMotor.getCIM(2), kGearRatio, kTrackWidth, kWheelRadius, null);

  private static Drivetrain mInstance;
  private static PeriodicIO mPeriodicIO;

  public static Drivetrain getInstance() {
    if (mInstance == null) {
      mInstance = new Drivetrain();
    }
    return mInstance;
  }

  public Drivetrain() {
    super("Drivetrain");

    mGyro.reset();

    mLeftLeader.restoreFactoryDefaults();
    mLeftLeader.setIdleMode(IdleMode.kCoast);
    mLeftFollower.restoreFactoryDefaults();
    mLeftFollower.setIdleMode(IdleMode.kCoast);
    mRightLeader.restoreFactoryDefaults();
    mRightLeader.setIdleMode(IdleMode.kCoast);
    mRightFollower.restoreFactoryDefaults();
    mRightFollower.setIdleMode(IdleMode.kCoast);

    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    mRightGroup.setInverted(true);

    mLeftEncoder = mLeftLeader.getEncoder();
    mRightEncoder = mRightLeader.getEncoder();

    // The "native units" for the SparkMax is motor rotations:
    // Conversion factor = (distance traveled per motor shaft rotation)
    mLeftEncoder.setPositionConversionFactor(kMetersPerRev);
    mRightEncoder.setPositionConversionFactor(kMetersPerRev);
    // The "native units" for the SparkMax is RPM:
    // Conversion factor = (distance traveled per motor shaft rotation) / (60
    // seconds)
    mLeftEncoder.setVelocityConversionFactor(kMetersPerRev / 60);
    mRightEncoder.setVelocityConversionFactor(kMetersPerRev / 60);

    mLeftEncoder.setPosition(0.0);
    mRightEncoder.setPosition(0.0);

    mOdometry = new DifferentialDriveOdometry(mGyro.getRotation2d(),
        mLeftEncoder.getPosition(), mRightEncoder.getPosition());

    mPeriodicIO = new PeriodicIO();

    // Configure AutoBuilder last
    AutoBuilder.configureRamsete(
        this::getPose, // Robot pose supplier
        this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
        this::getCurrentSpeeds, // Current ChassisSpeeds supplier
        this::drive, // Method that will drive the robot given ChassisSpeeds
        new ReplanningConfig(), // Default path replanning config. See the API for the options here
        this // Reference to this subsystem to set requirements
    );

    mSysIdRoutine = new SysIdRoutine(
        // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
        new SysIdRoutine.Config(),
        new SysIdRoutine.Mechanism(
            // Tell SysId how to plumb the driving voltage to the motors.
            (Measure<Voltage> volts) -> {
              // System.out.println("OPE:" + volts);
              mLeftGroup.setVoltage(volts.in(Volts));
              mRightGroup.setVoltage(volts.in(Volts));
            },
            // Tell SysId how to record a frame of data for each motor on the mechanism
            // being characterized.
            log -> {
              // Record a frame for the left motors. Since these share an encoder, we consider
              // the entire group to be one motor.
              log.motor("drive-left")
                  .voltage(m_appliedVoltage.mut_replace(
                      mLeftLeader.getAppliedOutput() * RobotController.getBatteryVoltage(), Volts))
                  .linearPosition(m_distance.mut_replace(mLeftEncoder.getPosition(), Meters))
                  .linearVelocity(
                      m_velocity.mut_replace(mLeftEncoder.getVelocity(), MetersPerSecond));
              // Record a frame for the right motors. Since these share an encoder, we
              // consider the entire group to be one motor.
              log.motor("drive-right")
                  .voltage(m_appliedVoltage.mut_replace(
                      -mRightLeader.getAppliedOutput() * RobotController
                          .getBatteryVoltage(),
                      Volts))
                  .linearPosition(m_distance.mut_replace(-mRightEncoder.getPosition(), Meters))
                  .linearVelocity(
                      m_velocity.mut_replace(-mRightEncoder.getVelocity(), MetersPerSecond));
            },
            // Tell SysId to make generated commands require this subsystem, suffix test
            // state in WPILog with this subsystem's name ("drive")
            this));

  }

  private static class PeriodicIO {
    DifferentialDriveWheelSpeeds diffWheelSpeeds = new DifferentialDriveWheelSpeeds(0.0, 0.0);
    boolean slowMode = false;
    boolean speedMode = false;
    double leftVoltage = 0.0;
    double rightVoltage = 0.0;
  }

  /**
   * Sets whether slow mode should be used
   *
   * @param slowMode Should slow mode be used
   */
  public void slowMode(boolean slowMode) {
    mPeriodicIO.slowMode = slowMode;
  }

  /**
   * Sets whether speed mode should be used
   *
   * @param speedMode Should speed mode be used
   */
  public void speedMode(boolean speedMode) {
    mPeriodicIO.speedMode = speedMode;
  }

  /**
   * Controls the robot using arcade drive.
   *
   * @param xSpeed the speed for the x axis
   * @param rot    the rotation
   */
  public void drive(double xSpeed, double rot) {
    if (mPeriodicIO.slowMode) {
      mPeriodicIO.diffWheelSpeeds = mKinematics.toWheelSpeeds(new ChassisSpeeds(xSpeed, 0, rot * kSlowModeRotScale));
    } else if (mPeriodicIO.speedMode) {
      mPeriodicIO.diffWheelSpeeds = mKinematics
          .toWheelSpeeds(new ChassisSpeeds(xSpeed * kSpeedModeScale, 0, rot * kSlowModeRotScale));
    } else {
      mPeriodicIO.diffWheelSpeeds = mKinematics.toWheelSpeeds(new ChassisSpeeds(xSpeed, 0, rot));
    }
  }

  public void drive(ChassisSpeeds speeds) {
    mPeriodicIO.diffWheelSpeeds = mKinematics.toWheelSpeeds(speeds);
  }

  public void clearTurnPIDAccumulation() {
    mLeftPIDController.reset();
    mRightPIDController.reset();
  }

  public void setGyroAngleAdjustment(double angle) {
    mGyro.setAngleAdjustment(angle);
  }

  /** Update robot odometry. */
  public void updateOdometry() {
    mOdometry.update(mGyro.getRotation2d(), mLeftEncoder.getPosition(), -mRightEncoder.getPosition());
  }

  /** Resets robot odometry. */
  public void resetOdometry(Pose2d pose) {
    mLeftEncoder.setPosition(0.0);
    mRightEncoder.setPosition(0.0);
    mDrivetrainSimulator.setPose(pose);

    mOdometry.resetPosition(pose.getRotation(), 0.0, 0.0, pose);
  }

  /** Check the current robot pose. */
  public Pose2d getPose() {
    return mOdometry.getPoseMeters();
  }

  public void setPose(Pose2d pose) {
    mOdometry.resetPosition(
        mGyro.getRotation2d(),
        mLeftEncoder.getPosition(),
        -mRightEncoder.getPosition(),
        pose);
  }

  public ChassisSpeeds getCurrentSpeeds() {
    DifferentialDriveWheelSpeeds wheelSpeeds = new DifferentialDriveWheelSpeeds(
        mLeftEncoder.getVelocity(),
        -mRightEncoder.getVelocity());

    return mKinematics.toChassisSpeeds(wheelSpeeds);
  }

  /** Update our simulation. This should be run every robot loop in simulation. */
  public void simulationPeriodic() {
    // To update our simulation, we set motor voltage inputs, update the
    // simulation, and write the simulated positions and velocities to our
    // simulated encoder and gyro. We negate the right side so that positive
    // voltages make the right side move forward.
    mDrivetrainSimulator.setInputs(
        mLeftGroup.get() * RobotController.getInputVoltage(),
        mRightGroup.get() * RobotController.getInputVoltage());
    mDrivetrainSimulator.update(0.02);

    // mLeftEncoderSim.setDistance(mdrivetrainSimulator.getLeftPositionMeters());
    // mLeftEncoderSim.setRate(mdrivetrainSimulator.getLeftVelocityMetersPerSecond());
    // mRightEncoderSim.setDistance(mdrivetrainSimulator.getRightPositionMeters());
    // mRightEncoderSim.setRate(mdrivetrainSimulator.getRightVelocityMetersPerSecond());
    // mGyroSim.setAngle(-mdrivetrainSimulator.getHeading().getDegrees());
  }

  @Override
  public void periodic() {
    var leftFeedforward = mLeftFeedforward.calculate(mPeriodicIO.diffWheelSpeeds.leftMetersPerSecond);
    var rightFeedforward = mRightFeedforward.calculate(mPeriodicIO.diffWheelSpeeds.rightMetersPerSecond);
    double leftOutput = mLeftPIDController.calculate(mLeftEncoder.getVelocity(),
        mPeriodicIO.diffWheelSpeeds.leftMetersPerSecond);
    double rightOutput = mRightPIDController.calculate(-mRightEncoder.getVelocity(),
        mPeriodicIO.diffWheelSpeeds.rightMetersPerSecond);

    mPeriodicIO.leftVoltage = leftOutput + leftFeedforward;
    mPeriodicIO.rightVoltage = rightOutput + rightFeedforward;

    updateOdometry();

    m_field.setRobotPose(getPose());
  }

  @Override
  public void reset() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'reset'");
  }

  @Override
  public void writePeriodicOutputs() {
    mLeftGroup.setVoltage(mPeriodicIO.leftVoltage);
    mRightGroup.setVoltage(mPeriodicIO.rightVoltage);
  }

  @Override
  public void stop() {
    mPeriodicIO.diffWheelSpeeds = new DifferentialDriveWheelSpeeds(0.0, 0.0);
  }

  @Override
  public void outputTelemetry() {
    putNumber("leftVelocitySetPoint", mPeriodicIO.diffWheelSpeeds.leftMetersPerSecond);
    putNumber("rightVelocitySetPoint", mPeriodicIO.diffWheelSpeeds.rightMetersPerSecond);
    putNumber("leftVelocity", mLeftEncoder.getVelocity());
    putNumber("rightVelocity", -mRightEncoder.getVelocity());
    putNumber("leftMeters", mLeftEncoder.getPosition());
    putNumber("rightMeters", -mRightEncoder.getPosition());
    putNumber("Gyro", mGyro.getAngle());
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return mSysIdRoutine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return mSysIdRoutine.dynamic(direction);
  }
}
