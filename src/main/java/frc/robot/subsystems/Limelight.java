package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.AprilTagLocations;
import frc.robot.Constants;

public class Limelight {
    private final PhotonCamera m_limelight;

    private String m_limelightName;

    public Limelight(String limelightName) {
        m_limelightName = limelightName;

        m_limelight = new PhotonCamera(m_limelightName);
    }

    public PhotonPipelineResult getLatestResult() {
        return m_limelight.getLatestResult();
    }

    public PhotonTrackedTarget getBestTarget() {
        return getLatestResult().getBestTarget();
    }

    public PhotonTrackedTarget getTarget(int tagID) {
        PhotonPipelineResult latest = getLatestResult();

        for (PhotonTrackedTarget target: latest.targets) {
            if (target.getFiducialId() == tagID) {
                return target;
            }
        }

        return null;
    }

    public Transform3d getTransform3d() {
        return getBestTarget().getBestCameraToTarget();
    }

    public double getDistanceFromBestTarget() {
        if (getLatestResult().hasTargets()) {
            return PhotonUtils.calculateDistanceToTargetMeters(
                Constants.Limelight.k_height,
                AprilTagLocations.Red.k_speakerTag3.getZ(),
                Units.degreesToRadians(Constants.Limelight.k_pitch),
                Units.degreesToRadians(getTarget(3).getPitch()));
        }

        return -1;
    }

    public double getDistanceFromTarget(Pose2d robotPose, Pose2d targetPose) {
        return PhotonUtils.getDistanceToPose(robotPose, targetPose);
    }

    public void outputTelemetry() {
        SmartDashboard.putBoolean("Limelight/ + " + m_limelightName + "/SeesAprilTag", getLatestResult().hasTargets());

        if (getLatestResult().hasTargets()) {
            SmartDashboard.putNumber("Limelight/ + " + m_limelightName + "/CurrentVisibleTag", getBestTarget().getFiducialId());
            SmartDashboard.putNumber("Limelight/ + " + m_limelightName + "/PitchDegrees", getBestTarget().getPitch());
            SmartDashboard.putNumber("Limelight/ + " + m_limelightName + "/DistanceFromBestTarget", getDistanceFromBestTarget());

            SmartDashboard.putNumber("Limelight/ + " + m_limelightName + "/X", getTransform3d().getX());
            SmartDashboard.putNumber("Limelight/ + " + m_limelightName + "/Y", getTransform3d().getY());
            SmartDashboard.putNumber("Limelight/ + " + m_limelightName + "/Z", getTransform3d().getZ());
        }
    }
}
