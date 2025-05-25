// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.FieldConstants.BranchSide;
import frc.robot.subsystems.Arm.Arm;
import frc.robot.subsystems.Arm.Arm.ArmState;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Superstructure.Superstructure;
import frc.robot.subsystems.Swerve.SwerveDrive;
import frc.robot.subsystems.Swerve.TunerConstants;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
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

	private final Intake intake = Intake.getInstance();

	private final Elevator elevator = Elevator.getInstance();

	private final Arm arm = Arm.getInstance();

	private final Telemetry telemetry = new Telemetry();

  	public RobotController() {
    	configureBindings();

		telemetry.registerSuppliers(
			() -> intake.getData(),
			() -> elevator.getData(),
			() -> arm.getData()
		);

		swerve.registerTelemetry(telemetry::updateSwerveTelemetry);
  	}

  	private void configureBindings() {

		swerve.setDefaultCommand(
            swerve.applyRequest(() ->
                drive.withVelocityX(-controller.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-controller.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-controller.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

		controller.leftBumper().onTrue(superstructure.scoreCoral(BranchSide.LEFT));
		controller.rightBumper().onTrue(superstructure.scoreCoral(BranchSide.RIGHT));
		
  	}

	public void updateTelemetry() {
		telemetry.updateSuperstructureTelemetry();
		telemetry.updateIntakeTelemetry();
		telemetry.updateElevatorTelemetry();
		telemetry.updateArmTelemetry();
	}

	public Command getAutoCommand() {
		return arm.requestState(ArmState.CORAL_ALIGN);
	}

}
