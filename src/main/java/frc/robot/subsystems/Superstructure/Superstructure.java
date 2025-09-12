package frc.robot.subsystems.Superstructure;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.FieldConstants.BranchSide;
import frc.robot.FieldConstants.GamePiece;
import frc.robot.subsystems.Arm.Arm;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Swerve.SwerveDrive;
import frc.robot.util.RobotVisualizer;
import lombok.Getter;

public class Superstructure extends SubsystemBase {
	
	@Getter
	@Logged(name = "State")
	private SuperstructureState currentState = SuperstructureState.STOW;

	@Logged(name = "Target Reef State")
	private SuperstructureState targetReefState = SuperstructureState.L4_ALIGN;

	@Logged(name = "Game Piece")
	private GamePiece gamePiece = GamePiece.NONE;

	private final SwerveDrive swerve;
	private final Intake intake;
	private final Elevator elevator;
	private final Arm arm;

	private final RobotVisualizer visualizer = new RobotVisualizer();
	
	public Superstructure(SwerveDrive swerve, Intake intake, Elevator elevator, Arm arm) {
		this.swerve = swerve;
		this.intake = intake;
		this.elevator = elevator;
		this.arm = arm;
	}

	private void swapState(SuperstructureState state) {
		this.currentState = state;

		swerve.swapState(state.data.getSwerveState());
		elevator.swapState(state.data.getElevatorState());
		arm.swapState(state.data.getArmState());
		intake.swapState(state.data.getIntakeState());
	}

	public void simulationPeriodic() {

		visualizer.updatePoses(
			intake.getData().position, 
			elevator.getData().position, 
			arm.getData().position
		);

		// State Transistions
		switch(currentState) {
			case DRIVE_TO_REEF -> {
				if(swerve.atTargetPose() && gamePiece == GamePiece.CORAL) {
					requestState(targetReefState);
				}
			}
			case L2_ALIGN -> {
				if(arm.atSetpoint() && elevator.atSetpoint() && swerve.atTargetPose()) {
					requestState(SuperstructureState.L2_SCORE);
				}
			}
			case L3_ALIGN -> {
				if(arm.atSetpoint() && elevator.atSetpoint() && swerve.atTargetPose()) {
					requestState(SuperstructureState.L3_SCORE);
				}
			}
			case L4_ALIGN -> {
				if(arm.atSetpoint() && elevator.atSetpoint() && swerve.atTargetPose()) {
					requestState(SuperstructureState.L4_SCORE);
				}
			}
			case L2_SCORE, L3_SCORE, L4_SCORE -> {
				if(arm.atSetpoint()) {
					gamePiece = GamePiece.NONE;
					requestState(SuperstructureState.STOW);
				}
			}
			default -> {}
		}
	}

	public Command setGamePiece(GamePiece gamePiece) {
		return this.runOnce(() -> this.gamePiece = gamePiece);
	}

	public Command setTargetSide(BranchSide side) {
		return this.runOnce(() -> swerve.setTargetSide(side));
	}

	// Only call when robot is stuck
	public Command forceReset() {
		return this.runOnce(() -> {
			gamePiece = GamePiece.NONE;
			swapState(SuperstructureState.STOW);
		});
	}

	public Command requestState(SuperstructureState requestedState) {
		return this.runOnce(() -> {
			if(currentState.getConnectedStates().contains(requestedState)) {
				swapState(requestedState);
			}
		});
	}
}
