// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.FieldConstants.BranchSide;
import frc.robot.FieldConstants.GamePiece;
import frc.robot.subsystems.Superstructure.Superstructure;
import frc.robot.subsystems.Superstructure.SuperstructureState;
import frc.robot.subsystems.Swerve.SwerveDrive;
import frc.robot.subsystems.Swerve.TunerConstants;

public class RobotController {

	private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

	// Setting up bindings for necessary control of the swerve drive platform
	private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
		.withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
		.withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
	
	private final CommandXboxController controller = new CommandXboxController(0);

	private final Superstructure superstructure = Superstructure.getInstance();

	private final SwerveDrive swerve = SwerveDrive.getInstance();

  	public RobotController() {
    	configureBindings();
  	}

  	private void configureBindings() {

		swerve.setDefaultCommand(
            swerve.applyRequest(() -> 
                drive.withVelocityX(-controller.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-controller.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-controller.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

		controller.leftBumper().onTrue(
			superstructure.setTargetSide(BranchSide.LEFT)
			.andThen(superstructure.requestState(SuperstructureState.DRIVE_TO_REEF))
		);
		
		controller.rightBumper().onTrue(
			superstructure.setTargetSide(BranchSide.RIGHT)
			.andThen(superstructure.requestState(SuperstructureState.DRIVE_TO_REEF))
		);
		
		controller.a().and(() -> superstructure.getCurrentState() != SuperstructureState.INTAKE_CORAL).onTrue(
			superstructure.requestState(SuperstructureState.INTAKE_CORAL)
			.andThen(superstructure.setGamePiece(GamePiece.CORAL))
		);

		controller.a().and(() -> superstructure.getCurrentState() == SuperstructureState.INTAKE_CORAL).onTrue(
			superstructure.requestState(SuperstructureState.STOW)
			.andThen(superstructure.setGamePiece(GamePiece.CORAL))
		);

		controller.povDown().onTrue(
			superstructure.forceReset()
		);

  	}

	public Command getAutoCommand() {
		return Commands.none();
	}

}
