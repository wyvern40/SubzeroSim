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

	private SuperstructureState targetState = SuperstructureState.STOW;
	private SuperstructureState currentState = SuperstructureState.STOW;

	private GamePiece heldGamePiece = GamePiece.NONE;

	private final SwerveDrive swerve = SwerveDrive.getInstance();
	private final Intake intake = Intake.getInstance();
	private final Elevator elevator = Elevator.getInstance();
	private final Arm arm = Arm.getInstance();
	
	private Superstructure() {}
	
	private void swapState(SuperstructureState state) {
		
	}

	public Command requestState(SuperstructureState state) {
		return this.runOnce(() -> {
			if(currentState.getConnectedStates().contains(state)) {
				targetState = state;
				swapState(state);
			}
		});
	}

}
