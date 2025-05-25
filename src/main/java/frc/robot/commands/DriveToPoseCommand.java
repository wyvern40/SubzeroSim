package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.Swerve.SwerveDrive;

public class DriveToPoseCommand extends Command {

    private final SwerveDrive swerve = SwerveDrive.getInstance();

    private final PIDController xController = new PIDController(5.0, 0, 0.0);
    private final PIDController yController = new PIDController(5.0, 0, 0.0);

    private final ProfiledPIDController thetaController = new ProfiledPIDController(15.0, 0, 0.05, new Constraints(Math.PI * 2.0, Math.PI));

    public DriveToPoseCommand(Pose2d targetPose) {

        this.addRequirements(swerve);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        xController.setTolerance(0.05, 10.0);
        yController.setTolerance(0.05, 10.0);
        thetaController.setTolerance(Math.PI / 36.0, 10.0);

        thetaController.reset(swerve.getState().Pose.getRotation().getRadians()); 

        xController.setSetpoint(targetPose.getX());
        yController.setSetpoint(targetPose.getY());
        thetaController.setGoal(targetPose.getRotation().getRadians());
    }

    @Override public void initialize() {}

    @Override
    public void execute() {

        Pose2d currentPose = swerve.getState().Pose;

        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            new ChassisSpeeds(
                xController.calculate(currentPose.getX()),
                yController.calculate(currentPose.getY()),
                thetaController.calculate(currentPose.getRotation().getRadians())
            ),
            currentPose.getRotation()
        );

        swerve.setControl(new SwerveRequest.ApplyRobotSpeeds().withSpeeds(speeds));

    }

    public boolean atSetpoint() {
        return xController.atSetpoint() && yController.atSetpoint() && thetaController.atSetpoint();
    }

}
