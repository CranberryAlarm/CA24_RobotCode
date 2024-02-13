package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

    public Transform3d getTransform3d() {
        return getBestTarget().getBestCameraToTarget();
    }

    public double getRotation() {
        return Units.radiansToDegrees(getTransform3d().getRotation().getZ());
    }

    public double getDistanceFromBestTarget() {
        if (getLatestResult().hasTargets()) {
            
        }
    }

    public void outputTelemetry() {
        SmartDashboard.putBoolean("Limelight/ + " + m_limelightName + "/SeesAprilTag", getLatestResult().hasTargets());
        SmartDashboard.putNumber("Limelight/ + " + m_limelightName + "/CurrentVisibleTag", getBestTarget().getFiducialId());
        SmartDashboard.putNumber("Limelight/ + " + m_limelightName + "/X", getTransform3d().getX());
        SmartDashboard.putNumber("Limelight/ + " + m_limelightName + "/Y", getTransform3d().getY());
        SmartDashboard.putNumber("Limelight/ + " + m_limelightName + "/YawDegrees", getRotation());
    }
}
