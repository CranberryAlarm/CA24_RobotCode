// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
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
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.simulation.SimulatableCANSparkMax;

public class Drivetrain extends Subsystem {
  // 1 meters per second.
  public static final double kMaxSpeed = 5.0;

  // 3 meters per second.
  public static final double kMaxAcceleration = 2.0;

  // 0.7 rotations per second.
  public static final double kMaxAngularSpeed = Math.PI * 0.8;

  private static final double kSlowModeRotScale = 0.1;
  private static final double kSpeedModeScale = 2.0;
  private static final double kTrackWidth = Units.inchesToMeters(22.0);
  private static final double kWheelRadius = Units.inchesToMeters(3.0);
  private static final double kGearRatio = 10.61;
  private static final double kMetersPerRev = (2 * Math.PI * kWheelRadius) / kGearRatio;

  private final SimulatableCANSparkMax mLeftLeader = new SimulatableCANSparkMax(Constants.kDrivetrainFLMotorId,
      MotorType.kBrushless);
  private final SimulatableCANSparkMax mLeftFollower = new SimulatableCANSparkMax(Constants.kDrivetrainBLMotorId,
      MotorType.kBrushless);
  private final SimulatableCANSparkMax mRightLeader = new SimulatableCANSparkMax(Constants.kDrivetrainFRMotorId,
      MotorType.kBrushless);
  private final SimulatableCANSparkMax mRightFollower = new SimulatableCANSparkMax(Constants.kDrivetrainBRMotorId,
      MotorType.kBrushless);

  private final MotorControllerGroup mLeftGroup = new MotorControllerGroup(mLeftLeader, mLeftFollower);
  private final MotorControllerGroup mRightGroup = new MotorControllerGroup(mRightLeader, mRightFollower);

  private final RelativeEncoder mLeftEncoder;
  private final RelativeEncoder mRightEncoder;

  private final PIDController mLeftPIDController = new PIDController(2, 0, 0);
  private final PIDController mRightPIDController = new PIDController(2, 0, 0);

  private final AHRS mGyro = new AHRS();

  private final DifferentialDriveKinematics mKinematics = new DifferentialDriveKinematics(kTrackWidth);

  private final DifferentialDriveOdometry mOdometry;

  private final SimpleMotorFeedforward mLeftFeedforward = new SimpleMotorFeedforward(0.65, 1.5);
  private final SimpleMotorFeedforward mRightFeedforward = new SimpleMotorFeedforward(0.65, 1.5);

  // Simulation classes help us simulate our robot
  // private final AnalogGyroSim mGyroSim = new AnalogGyroSim(mgyro);
  // private final EncoderSim mLeftEncoderSim = new EncoderSim(mleftEncoder);
  // private final EncoderSim mRightEncoderSim = new EncoderSim(mrightEncoder);
  private final Field2d mFieldSim = new Field2d();
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
        mLeftEncoder.getPosition(), -mRightEncoder.getPosition());

    mPeriodicIO = new PeriodicIO();

    SmartDashboard.putData("Field", mFieldSim);
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

  /** Update robot odometry. */
  public void updateOdometry() {
    mOdometry.update(mGyro.getRotation2d(), mLeftEncoder.getPosition(), -mRightEncoder.getPosition());
  }

  /** Resets robot odometry. */
  public void resetOdometry(Pose2d pose) {
    mLeftEncoder.setPosition(0.0);
    mRightEncoder.setPosition(0.0);
    mDrivetrainSimulator.setPose(pose);

    mOdometry.resetPosition(pose.getRotation(), mLeftEncoder.getPosition(),
        -mRightEncoder.getPosition(), pose);
  }

  /** Check the current robot pose. */
  public Pose2d getPose() {
    return mOdometry.getPoseMeters();
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
    mFieldSim.setRobotPose(getPose());
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
    SmartDashboard.putNumber("leftVelocitySetPoint", mPeriodicIO.diffWheelSpeeds.leftMetersPerSecond);
    SmartDashboard.putNumber("rightVelocitySetPoint", mPeriodicIO.diffWheelSpeeds.rightMetersPerSecond);
    SmartDashboard.putNumber("leftVelocity", mLeftEncoder.getVelocity());
    SmartDashboard.putNumber("rightVelocity", -mRightEncoder.getVelocity());
    SmartDashboard.putNumber("leftMeters", mLeftEncoder.getPosition());
    SmartDashboard.putNumber("rightMeters", -mRightEncoder.getPosition());
    SmartDashboard.putNumber("Gyro", mGyro.getAngle());
  }
}
