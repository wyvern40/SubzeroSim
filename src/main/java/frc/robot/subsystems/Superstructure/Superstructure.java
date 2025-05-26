package frc.robot.subsystems.Superstructure;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.FieldConstants.BranchSide;
import frc.robot.FieldConstants.GamePiece;
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
		CORAL_INTAKE(SwerveState.DRIVER_CONTROL, IntakeState.INTAKE, ElevatorState.CORAL_STOW, ArmState.CORAL_STOW),
		DRIVE_TO_REEF(SwerveState.PATH_TO_REEF, IntakeState.STOW, ElevatorState.CORAL_STOW, ArmState.CORAL_STOW),
		ALIGN_L2(SwerveState.ALIGN_TO_REEF, IntakeState.STOW, ElevatorState.L2, ArmState.CORAL_ALIGN),
		ALIGN_L3(SwerveState.ALIGN_TO_REEF, IntakeState.STOW, ElevatorState.L3, ArmState.CORAL_ALIGN),
		ALIGN_L4(SwerveState.ALIGN_TO_REEF, IntakeState.STOW, ElevatorState.L4, ArmState.CORAL_ALIGN),
		SCORE_L2(SwerveState.STOPPED, IntakeState.STOW, ElevatorState.L2, ArmState.CORAL_SCORE),
		SCORE_L3(SwerveState.STOPPED, IntakeState.STOW, ElevatorState.L3, ArmState.CORAL_SCORE),
		SCORE_L4(SwerveState.STOPPED, IntakeState.STOW, ElevatorState.L4, ArmState.CORAL_SCORE);

		SwerveState swerveState;
		IntakeState intakeState;
		ElevatorState elevatorState;
		ArmState armState;

		SuperstructureState(SwerveState swerveState, IntakeState intakeState, ElevatorState elevatorState, ArmState armState) {
			this.swerveState = swerveState;
			this.intakeState = intakeState;
			this.elevatorState = elevatorState;
			this.armState = armState;
		}
	}

	private SuperstructureState state = SuperstructureState.CORAL_STOW;

	private GamePiece heldGamePiece = GamePiece.NONE;

	private final SwerveDrive swerve = SwerveDrive.getInstance();

	private final Intake intake = Intake.getInstance();

	private final Elevator elevator = Elevator.getInstance();

	private final Arm arm = Arm.getInstance();
	
	private Superstructure() {}
	

	public void simulationPeriodic() {

		switch(state) {
			case DRIVE_TO_REEF -> {
				if(swerve.atTargetPose()) {
					swapState(SuperstructureState.ALIGN_L4);
				}
			}

			default -> {}
		}

		
	}

	public void swapState(SuperstructureState state) {

		this.state = state;

		switch(state) {
			case SCORE_L2, SCORE_L3, SCORE_L4 -> {
				heldGamePiece = GamePiece.NONE;
			}
			case CORAL_INTAKE -> {
				heldGamePiece = GamePiece.CORAL;
			}
			default -> {}
		}

		swerve.requestState(state.swerveState).schedule();
		intake.requestState(state.intakeState).schedule();
		elevator.requestState(state.elevatorState).schedule();
		arm.requestState(state.armState).schedule();

	}
	
	public void initState() {
		swapState(SuperstructureState.CORAL_STOW);
	}

	public Command scoreCoral(BranchSide side) {
		return this.runOnce(() -> {
			swerve.setTargetSide(side);
			if(state == SuperstructureState.CORAL_STOW && heldGamePiece == GamePiece.NONE) {
				swapState(SuperstructureState.DRIVE_TO_REEF);
			}
		});
	}

}
