package frc.robot;

import edu.wpi.first.math.util.Units;

public class Constants {
  public static class Robot {
    public static final double k_width = 26; // Inches
    public static final double k_length = 28; // Inches
  }

  public static class Intake {
    public static final double k_intakeSpeed = 0.7;
    public static final double k_ejectSpeed = -0.3;
  }

  // PCM
  public static final int kPCMId = 0;
  public static final int kIntakeSolenoidForwardId = 2;

  // DIO

  // Intake
  public static final int kIntakeMotorId = 11;

  // Shooter
  public static final int kShooterLeftMotorId = 12;
  public static final int kShooterRightMotorId = 13;

  // Drivetrain
  public static final int kDrivetrainFLMotorId = 5;
  public static final int kDrivetrainBLMotorId = 6;
  public static final int kDrivetrainFRMotorId = 7;
  public static final int kDrivetrainBRMotorId = 8;

  public static class Field {
    // All dimensions from Figure 5-16 in the manual
    // public static final double k_lowGoalX = 22.75; // Inches
    // public static final double k_lowGoalHeight = 34; // Inches

    // public static final double k_highGoalX = 39.75; // Inches
    // public static final double k_highGoalHeight = 46; // Inches

    public static final double k_width = Units.feetToMeters(54.0);
    public static final double k_length = Units.feetToMeters(27.0);
  }
}
