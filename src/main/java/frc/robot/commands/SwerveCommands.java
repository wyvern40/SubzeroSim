package frc.robot.commands;

import static edu.wpi.first.units.Units.*;

import java.util.Arrays;
import java.util.Map;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.FieldConstants;
import frc.robot.subsystems.Swerve.SwerveConstants;
import frc.robot.subsystems.Swerve.SwerveDrive.TargetSide;

public class SwerveCommands {

    private static Map<Pose2d, String> pathNames = Map.of(
        FieldConstants.REEF_FACES[0], "Front Left Align ",
        FieldConstants.REEF_FACES[1], "Front Align ",
        FieldConstants.REEF_FACES[2], "Front Right Align ",
        FieldConstants.REEF_FACES[3], "Back Left Align ",
        FieldConstants.REEF_FACES[4], "Back Align ",
        FieldConstants.REEF_FACES[5], "Back Right Align "
    );

    public static Command pathToReef(Pose2d currentPose, TargetSide side) {

        var nearestPose = currentPose.nearest(Arrays.asList(FieldConstants.REEF_FACES));
        var targetPose = nearestPose.transformBy(
            new Transform2d(SwerveConstants.LINEUP_DISTANCE.in(Meters), FieldConstants.REEF_POLE_OFFSET.in(Meters) * side.direction, Rotation2d.kZero)
        );

        return AutoBuilder.pathfindToPose(
            targetPose,
            null
        );
    }

    public static Command alignToReef(Pose2d currentPose, TargetSide side) {
        var closestFace = currentPose.nearest(Arrays.asList(FieldConstants.REEF_FACES));

        try {
            PathPlannerPath path = PathPlannerPath.fromPathFile(pathNames.get(closestFace) + "(" + side.name() + ")");
            return AutoBuilder.followPath(path);
        } catch (Exception e) {
            DriverStation.reportError(e.getMessage(), e.getStackTrace());
            return Commands.none();
        }
    }

}
