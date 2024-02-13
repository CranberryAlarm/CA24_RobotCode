package frc.robot;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public class AprilTagLocations {
    public static class Red {
        public static final Transform3d k_sourceTag1 = new Transform3d(
            new Translation3d(15.079472, 0.245872, 1.355852),
            new Rotation3d(0, 0, Units.degreesToRadians(120)));

        public static final Transform3d k_sourceTag2 = new Transform3d(
            new Translation3d(16.185134, 0.883666, 1.355852),
            new Rotation3d(0, 0, Units.degreesToRadians(120)));

        public static final Transform3d k_speakerTag3 = new Transform3d(
            new Translation3d(16.579342, 4.982718, 1.451102),
            new Rotation3d(0, 0, Units.degreesToRadians(180)));

        public static final Transform3d k_speakerTag4 = new Transform3d(
            new Translation3d(16.579342, 5.547868, 1.451102),
            new Rotation3d(0, 0, Units.degreesToRadians(180)));

        public static final Transform3d k_ampTag5 = new Transform3d(
            new Translation3d(14.700758, 8.2042, 1.355852),
            new Rotation3d(0, 0, Units.degreesToRadians(270)));

        public static final Transform3d k_stageSourceSideTag11 = new Transform3d(
            new Translation3d(11.904726, 3.713226, 1.3208),
            new Rotation3d(0, 0, Units.degreesToRadians(300)));

        public static final Transform3d k_stageAmpSideTag12 = new Transform3d(
            new Translation3d(11.904726, 4.49834, 1.3208),
            new Rotation3d(0, 0, Units.degreesToRadians(60)));

        public static final Transform3d k_stageCenterSideTag13 = new Transform3d(
            new Translation3d(11.220196, 4.105148, 1.3208),
            new Rotation3d(0, 0, Units.degreesToRadians(180)));
    }
}
