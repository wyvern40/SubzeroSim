package frc.robot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.units.measure.Distance;
import frc.robot.subsystems.Swerve.SwerveConstants;

public class FieldConstants {

    public enum GamePiece {
        NONE,
        CORAL,
        ALGAE
    }

    public enum BranchSide {
        LEFT(Inches.of(-6.5)),
        RIGHT(Inches.of(6.5));

        public Distance offset;

        BranchSide(Distance offset) {
            this.offset = offset;
        }
    }

    public static final Pose2d[] REEF_FACES = new Pose2d[6];

    static {
        var aprilTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

        REEF_FACES[0] = aprilTagLayout.getTagPose(18).get().toPose2d().transformBy(
            new Transform2d(SwerveConstants.BUMPER_RADIUS, Meters.of(0.0), Rotation2d.k180deg
        ));
        REEF_FACES[1] = aprilTagLayout.getTagPose(17).get().toPose2d().transformBy(
            new Transform2d(SwerveConstants.BUMPER_RADIUS, Meters.of(0.0), Rotation2d.k180deg
        ));
        REEF_FACES[2] = aprilTagLayout.getTagPose(20).get().toPose2d().transformBy(
            new Transform2d(SwerveConstants.BUMPER_RADIUS, Meters.of(0.0), Rotation2d.k180deg
        ));
        REEF_FACES[3] = aprilTagLayout.getTagPose(21).get().toPose2d().transformBy(
            new Transform2d(SwerveConstants.BUMPER_RADIUS, Meters.of(0.0), Rotation2d.k180deg
        ));
        REEF_FACES[4] = aprilTagLayout.getTagPose(22).get().toPose2d().transformBy(
            new Transform2d(SwerveConstants.BUMPER_RADIUS, Meters.of(0.0), Rotation2d.k180deg
        ));
        REEF_FACES[5] = aprilTagLayout.getTagPose(19).get().toPose2d().transformBy(
            new Transform2d(SwerveConstants.BUMPER_RADIUS, Meters.of(0.0), Rotation2d.k180deg
        ));
    }
}