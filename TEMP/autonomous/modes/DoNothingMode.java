package frc.robot.autonomous.modes;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.autonomous.tasks.DoNothingTask;

public class DoNothingMode extends AutoModeBase {
  @Override
  public Pose2d getRedStartingPosition() {
    return new Pose2d(0, 0, Rotation2d.fromDegrees(0));
  }

  public void queueTasks() {
    queueTask(new DoNothingTask());
    // queueTask(new ArmThrowTask());
  }
}
