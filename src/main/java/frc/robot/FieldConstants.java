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

    public enum BranchHeight {
        L2,
        L3,
        L4;

        public BranchHeight higher() {
            switch(this) {
               case L2 -> {return L3;}
               case L3 -> {return L4;}
               case L4 -> {return L4;}
               default -> {return L4;}
            }
        }

        public BranchHeight lower() {
            switch(this) {
               case L2 -> {return L2;}
               case L3 -> {return L2;}
               case L4 -> {return L3;}
               default -> {return L2;}
            }
        }
    }

    public static final Pose2d[] reefFaces = new Pose2d[6];

    static {
        var aprilTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

        reefFaces[0] = aprilTagLayout.getTagPose(18).get().toPose2d().transformBy(
            new Transform2d(SwerveConstants.BUMPER_RADIUS, Meters.of(0.0), Rotation2d.k180deg
        ));
        reefFaces[1] = aprilTagLayout.getTagPose(17).get().toPose2d().transformBy(
            new Transform2d(SwerveConstants.BUMPER_RADIUS, Meters.of(0.0), Rotation2d.k180deg
        ));
        reefFaces[2] = aprilTagLayout.getTagPose(20).get().toPose2d().transformBy(
            new Transform2d(SwerveConstants.BUMPER_RADIUS, Meters.of(0.0), Rotation2d.k180deg
        ));
        reefFaces[3] = aprilTagLayout.getTagPose(21).get().toPose2d().transformBy(
            new Transform2d(SwerveConstants.BUMPER_RADIUS, Meters.of(0.0), Rotation2d.k180deg
        ));
        reefFaces[4] = aprilTagLayout.getTagPose(22).get().toPose2d().transformBy(
            new Transform2d(SwerveConstants.BUMPER_RADIUS, Meters.of(0.0), Rotation2d.k180deg
        ));
        reefFaces[5] = aprilTagLayout.getTagPose(19).get().toPose2d().transformBy(
            new Transform2d(SwerveConstants.BUMPER_RADIUS, Meters.of(0.0), Rotation2d.k180deg
        ));
    }
}