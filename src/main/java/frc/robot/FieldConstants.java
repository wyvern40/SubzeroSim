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

    public static final Pose2d REEF_FRONT;
    public static final Pose2d REEF_FRONT_LEFT;
    public static final Pose2d REEF_FRONT_RIGHT;
    public static final Pose2d REEF_BACK_RIGHT;
    public static final Pose2d REEF_BACK;
    public static final Pose2d REEF_BACK_LEFT;

    public static final Distance REEF_POLE_OFFSET = Inches.of(6.5);

    static {
        var aprilTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

        REEF_FRONT_LEFT = aprilTagLayout.getTagPose(19).get().toPose2d().transformBy(
            new Transform2d(SwerveConstants.BUMPER_RADIUS.plus(SwerveConstants.LINEUP_DISTANCE), REEF_POLE_OFFSET, Rotation2d.k180deg
        ));
        REEF_FRONT = aprilTagLayout.getTagPose(18).get().toPose2d().transformBy(
            new Transform2d(SwerveConstants.BUMPER_RADIUS, REEF_POLE_OFFSET, Rotation2d.k180deg
        ));
        REEF_FRONT_RIGHT = aprilTagLayout.getTagPose(17).get().toPose2d().transformBy(
            new Transform2d(SwerveConstants.BUMPER_RADIUS, REEF_POLE_OFFSET, Rotation2d.k180deg
        ));
        REEF_BACK_LEFT = aprilTagLayout.getTagPose(20).get().toPose2d().transformBy(
            new Transform2d(SwerveConstants.BUMPER_RADIUS, REEF_POLE_OFFSET, Rotation2d.k180deg
        ));
        REEF_BACK = aprilTagLayout.getTagPose(21).get().toPose2d().transformBy(
            new Transform2d(SwerveConstants.BUMPER_RADIUS, REEF_POLE_OFFSET, Rotation2d.k180deg
        ));
        REEF_BACK_RIGHT = aprilTagLayout.getTagPose(22).get().toPose2d().transformBy(
            new Transform2d(SwerveConstants.BUMPER_RADIUS, REEF_POLE_OFFSET, Rotation2d.k180deg
        ));
    }
}