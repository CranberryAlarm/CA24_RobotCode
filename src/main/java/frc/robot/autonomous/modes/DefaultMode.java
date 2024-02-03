package frc.robot.autonomous.modes;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.autonomous.tasks.BrakeTask;
import frc.robot.autonomous.tasks.DriveForwardTask;
import frc.robot.autonomous.tasks.IntakeTask;
import frc.robot.autonomous.tasks.ParallelTask;
import frc.robot.autonomous.tasks.ShooterTask;
import frc.robot.autonomous.tasks.WaitTask;
import frc.robot.subsystems.Intake.IntakeState;
import frc.robot.subsystems.Intake.PivotTarget;

public class DefaultMode extends AutoModeBase {
  @Override
  public Pose2d getBlueStartingPosition() {
    return new Pose2d(2.374, 5.636, Rotation2d.fromDegrees(0));
  }

  public void queueTasks() {
    queueTask(new ParallelTask(
        new IntakeTask(IntakeState.NONE, PivotTarget.STOW),
        new ShooterTask(3000),
        new WaitTask(2.0)));

    queueTask(new ParallelTask(
        new IntakeTask(IntakeState.EJECT, PivotTarget.STOW),
        new WaitTask(1.0)));

    queueTask(new ParallelTask(
        new IntakeTask(IntakeState.INTAKE, PivotTarget.GROUND),
        new WaitTask(0.1)));

    queueTask(new DriveForwardTask(1.3, 0.4));

    queueTask(new ParallelTask(
        new IntakeTask(IntakeState.PULSE, PivotTarget.STOW),
        new WaitTask(0.1)));

    queueTask(new DriveForwardTask(1.2, -0.4));

    queueTask(new ParallelTask(
        new IntakeTask(IntakeState.EJECT, PivotTarget.STOW),
        new WaitTask(2.0)));

    queueTask(new ParallelTask(
        new IntakeTask(IntakeState.NONE, PivotTarget.STOW),
        new WaitTask(2.0),
        new ShooterTask(0)));

    queueTask(new BrakeTask());
  }
}
