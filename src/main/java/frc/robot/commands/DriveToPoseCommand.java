package frc.robot.commands;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve.SwerveDrive;
import frc.robot.subsystems.Swerve.TunerConstants;

public class DriveToPoseCommand extends Command {

    private final SwerveDrive swerve = SwerveDrive.getInstance();

    private TrapezoidProfile driveProfile;

    private final HolonomicDriveController controller = new HolonomicDriveController(
        new PIDController(2.5, 0, 0),
        new PIDController(2.5, 0, 0),
        new ProfiledPIDController(100.0, 0, 0.5, new Constraints(4, 5))
    );

    private Pose2d targetPose;

    public DriveToPoseCommand(Pose2d targetPose) {
        this.targetPose = targetPose;
    }

    @Override
    public void initialize() {
        if(DriverStation.getAlliance().get() == DriverStation.Alliance.Red)
    }

    @Override
    public void execute() {
        
    }

}
