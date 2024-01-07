package frc.robot.autonomous.modes;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;
import frc.robot.autonomous.tasks.ArmTrajectoryTask;
import frc.robot.autonomous.tasks.BrakeTask;
import frc.robot.autonomous.tasks.DriveTrajectoryTask;
import frc.robot.autonomous.tasks.GripperTask;
import frc.robot.autonomous.tasks.ParallelTask;
import frc.robot.autonomous.tasks.PointForwardTask;
import frc.robot.autonomous.tasks.WaitTask;

public class DefaultMode extends AutoModeBase {
  @Override
  public Pose2d getRedStartingPosition() {
    return new Pose2d(14.655021228445234, 4.458172598636864, Rotation2d.fromDegrees(180));
  }

  public void queueTasks() {
    queueTask(new ParallelTask(
        new PointForwardTask(),
        new WaitTask(0.5)));

    queueTask(new ArmTrajectoryTask(Constants.Arm.Preset.SCORE_HIGH_CUBE.getPose()));

    queueTask(new WaitTask(Constants.Auto.k_defaultGripperWait + 0.5));

    queueTask(new GripperTask(false));

    queueTask(new WaitTask(Constants.Auto.k_defaultGripperWait));

    queueTask(new ParallelTask(
        new ArmTrajectoryTask(Constants.Arm.Preset.HOME.getPose()),
        new DriveTrajectoryTask("ForwardOnly", 1.5, 0.5, true)));

    queueTask(new BrakeTask(true));
  }
}
