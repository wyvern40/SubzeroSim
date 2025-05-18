package frc.robot.commands;

import java.util.Arrays;
import java.util.Map;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.FieldConstants;

public class DriveCommands {

    private static Map<Pose2d, String> pathNames = Map.of(
        FieldConstants.REEF_FRONT_LEFT, "Front Left Align ",
        FieldConstants.REEF_FRONT, "Front Align ",
        FieldConstants.REEF_FRONT_RIGHT, "Front Right Align ",
        FieldConstants.REEF_BACK_LEFT, "Back Left Align ",
        FieldConstants.REEF_BACK, "Back Align ",
        FieldConstants.REEF_BACK_RIGHT, "Back Right Align "
    );

    public static Command driveToReef(Pose2d currentPose, String direction) {
        var closestFace = currentPose.nearest(Arrays.asList(
            FieldConstants.REEF_FRONT_LEFT,
            FieldConstants.REEF_FRONT,
            FieldConstants.REEF_FRONT_RIGHT,
            FieldConstants.REEF_BACK_LEFT,
            FieldConstants.REEF_BACK,
            FieldConstants.REEF_BACK_RIGHT
        ));

        try {
            PathPlannerPath path = PathPlannerPath.fromPathFile(pathNames.get(closestFace) + "(" + direction + ")");
            return AutoBuilder.followPath(path);
        } catch (Exception e) {
            DriverStation.reportError(e.getMessage(), e.getStackTrace());
            return Commands.none();
        }
    }

}
