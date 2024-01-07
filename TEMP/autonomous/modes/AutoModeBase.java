package frc.robot.autonomous.modes;

import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.path.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;
import frc.robot.autonomous.tasks.Task;
import frc.robot.subsystems.drivetrain.SwerveDrive;

public abstract class AutoModeBase {
  private ArrayList<Task> m_tasks;

  public AutoModeBase() {
    m_tasks = new ArrayList<>();

    // Reset the gyro and set the starting position
    Pose2d startingPosition = getStartingPosition();
    SwerveDrive swerve = SwerveDrive.getInstance();
    swerve.setGyroAngleAdjustment(startingPosition.getRotation().getDegrees());
    swerve.resetOdometry(startingPosition);
  }

  public Task getNextTask() {
    // Pop the first task off the list and return it
    try {
      return m_tasks.remove(0);
    } catch (IndexOutOfBoundsException ex) {
      return null;
    }
  }

  public void queueTask(Task task) {
    m_tasks.add(task);
  }

  public abstract void queueTasks();

  public abstract Pose2d getRedStartingPosition();

  private Pose2d getBlueStartingPosition() {
    Rotation2d blueStartingRotation = Rotation2d.fromDegrees(getRedStartingPosition().getRotation().getDegrees() - 180);

    Translation2d blueStartingTranslation = new Translation2d(
        Constants.Field.k_width - getRedStartingPosition().getX(),
        getRedStartingPosition().getY());

    return new Pose2d(blueStartingTranslation, blueStartingRotation);
  };

  private Pose2d getStartingPosition() {
    if (DriverStation.getAlliance() == DriverStation.Alliance.Blue) {
      return getBlueStartingPosition();
    } else {
      return getRedStartingPosition();
    }
  }

  public static PathPlannerTrajectory transformTrajectoryForAlliance(PathPlannerTrajectory trajectory) {
    if (DriverStation.getAlliance() == DriverStation.Alliance.Red) {
      return trajectory;
    }

    // Flip the trajectory for blue
    List<State> transformedStates = new ArrayList<>();

    for (State s : trajectory.getStates()) {
      PathPlannerState state = (PathPlannerState) s;

      transformedStates.add(transformStateForAlliance(state));
    }

    return new PathPlannerTrajectory(
        transformedStates,
        trajectory.getMarkers(),
        trajectory.getStartStopEvent(),
        trajectory.getEndStopEvent(),
        trajectory.fromGUI);
  }

  public static PathPlannerState transformStateForAlliance(PathPlannerState state) {
    // Create a new state so that we don't overwrite the original
    PathPlannerState transformedState = new PathPlannerState();

    Translation2d transformedTranslation = new Translation2d(
        Constants.Field.k_width - state.poseMeters.getX(), state.poseMeters.getY());

    Rotation2d transformedHeading = Rotation2d.fromDegrees(state.poseMeters.getRotation().getDegrees() + 180);
    Rotation2d transformedHolonomicRotation = Rotation2d.fromDegrees(state.holonomicRotation.getDegrees() + 180);

    transformedState.timeSeconds = state.timeSeconds;
    transformedState.velocityMetersPerSecond = state.velocityMetersPerSecond;
    transformedState.accelerationMetersPerSecondSq = state.accelerationMetersPerSecondSq;
    transformedState.poseMeters = new Pose2d(transformedTranslation, transformedHeading);
    transformedState.angularVelocityRadPerSec = -state.angularVelocityRadPerSec;
    transformedState.holonomicRotation = transformedHolonomicRotation;
    transformedState.holonomicAngularVelocityRadPerSec = -state.holonomicAngularVelocityRadPerSec;
    // transformedState.curveRadius = -state.curveRadius;
    transformedState.curvatureRadPerMeter = -state.curvatureRadPerMeter;
    // transformedState.deltaPos = state.deltaPos;

    return transformedState;
  }
}
