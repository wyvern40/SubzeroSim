package frc.robot.subsystems.Superstructure;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Arm.Arm;
import frc.robot.subsystems.Arm.Arm.ArmState;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Elevator.Elevator.ElevatorState;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Intake.Intake.IntakeState;
import frc.robot.subsystems.Swerve.SwerveDrive;
import frc.robot.subsystems.Swerve.SwerveDrive.SwerveState;

public class Superstructure extends SubsystemBase {
    
    private static Superstructure instance;

    public static synchronized Superstructure getInstance() {
		if (instance == null) {
			instance = new Superstructure();
		}

		return instance;
	}

	public enum SuperstructureState {
		CORAL_STOW(SwerveState.DRIVER_CONTROL, IntakeState.STOW, ElevatorState.CORAL_STOW, ArmState.CORAL_STOW),
		REEF_PATHING(SwerveState.PATHING, IntakeState.STOW, ElevatorState.CORAL_STOW, ArmState.CORAL_STOW),
		ALIGN_L2(SwerveState.ALIGNING, IntakeState.STOW, ElevatorState.L2, ArmState.CORAL_ALIGN),
		ALIGN_L3(SwerveState.ALIGNING, IntakeState.STOW, ElevatorState.L3, ArmState.CORAL_ALIGN),
		ALIGN_L4(SwerveState.ALIGNING, IntakeState.STOW, ElevatorState.L4, ArmState.CORAL_ALIGN),
		SCORE_L2(SwerveState.STOPPED, IntakeState.STOW, ElevatorState.L2, ArmState.CORAL_SCORE),
		SCORE_L3(SwerveState.STOPPED, IntakeState.STOW, ElevatorState.L3, ArmState.CORAL_SCORE),
		SCORE_L4(SwerveState.STOPPED, IntakeState.STOW, ElevatorState.L4, ArmState.CORAL_SCORE);

		SwerveState swerveState;
		IntakeState intakeState;
		ElevatorState elevatorState;
		ArmState armState;

		SuperstructureState(SwerveState driveState, IntakeState intakeState, ElevatorState elevatorState, ArmState armState) {
			this.swerveState = swerveState;
			this.intakeState = intakeState;
			this.elevatorState = elevatorState;
			this.armState = armState;
		}
	}

	private final SwerveDrive swerve = SwerveDrive.getInstance();

	private final Intake intake = Intake.getInstance();

	private final Elevator elevator = Elevator.getInstance();

	private final Arm arm = Arm.getInstance();
	
    public Superstructure() {
		
	}

	public Command requestState(SuperstructureState state) {
		return Commands.parallel(
			intake.requestState(state.intakeState),
			elevator.requestState(state.elevatorState),
			arm.requestState(state.armState)
		);
	}

}
