package frc.robot.autonomous.modes;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.autonomous.tasks.BrakeTask;
import frc.robot.autonomous.tasks.IntakeTask;
import frc.robot.autonomous.tasks.ParallelTask;
import frc.robot.autonomous.tasks.ShooterTask;
import frc.robot.autonomous.tasks.WaitTask;
import frc.robot.subsystems.Intake.IntakeState;
import frc.robot.subsystems.Intake.PivotTarget;

public class DefaultMode extends AutoModeBase {
  @Override
  public Pose2d getRedStartingPosition() {
    return new Pose2d(14.655021228445234, 4.458172598636864, Rotation2d.fromDegrees(180));
  }

  public void queueTasks() {
    queueTask(new ParallelTask(
        new IntakeTask(IntakeState.NONE, PivotTarget.STOW),
        new ShooterTask(0.6),
        new WaitTask(5.0)));

    // queueTask(new
    // ArmTrajectoryTask(Constants.Arm.Preset.SCORE_HIGH_CUBE.getPose()));

    // queueTask(new WaitTask(Constants.Auto.k_defaultGripperWait + 0.5));

    queueTask(new ShooterTask(0.0));

    // queueTask(new WaitTask(Constants.Auto.k_defaultGripperWait));

    // queueTask(new ParallelTask(
    // new ArmTrajectoryTask(Constants.Arm.Preset.HOME.getPose()),
    // new DriveTrajectoryTask("ForwardOnly", 1.5, 0.5, true)));

    queueTask(new BrakeTask());
  }
}
