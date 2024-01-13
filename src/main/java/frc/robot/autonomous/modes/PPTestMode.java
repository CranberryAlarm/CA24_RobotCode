package frc.robot.autonomous.modes;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.autonomous.tasks.BrakeTask;
import frc.robot.autonomous.tasks.DriveTrajectoryTask;

public class PPTestMode extends AutoModeBase {
  @Override
  public Pose2d getBlueStartingPosition() {
    return new Pose2d(2.374, 5.636, Rotation2d.fromDegrees(0));
  }

  public void queueTasks() {
    queueTask(new DriveTrajectoryTask("Test Path", 1, 1));
    // queueTask(new DriveForwardTask(1, 0.9));

    // queueTask(new DriveForwardTask(1, -0.9));

    queueTask(new BrakeTask());
  }
}
