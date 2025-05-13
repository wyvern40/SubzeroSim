package frc.robot.subsystems.Superstructure;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Elevator.Elevator.ElevatorState;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Intake.Intake.IntakeState;
import frc.robot.subsystems.Swerve.SwerveDrive;

public class Superstructure extends SubsystemBase {
    
    private static Superstructure instance;

    public static synchronized Superstructure getInstance() {
		if (instance == null) {
			instance = new Superstructure();
		}

		return instance;
	}

	public enum SuperstructureState {
		CORAL_STOW(IntakeState.STOW, ElevatorState.CORAL_STOW),
		SCORE_L2(IntakeState.STOW, ElevatorState.L2),
		SCORE_L3(IntakeState.STOW, ElevatorState.L2),
		SCORE_L4(IntakeState.STOW, ElevatorState.L2);

		IntakeState intakeState;
		ElevatorState elevatorState;

		SuperstructureState(IntakeState intakeState, ElevatorState elevatorState) {
			this.intakeState = intakeState;
			this.elevatorState = elevatorState;
		}
	}

	private final SwerveDrive swerve = SwerveDrive.getInstance();

	private final Intake intake = Intake.getInstance();

	private final Elevator elevator = Elevator.getInstance();

	private ElevatorState elevatorTargetState = ElevatorState.L4;
	
    public Superstructure() {
		
	}

	//public Command requestState(SuperstructureState state) {
	//	return Commands.parallel(
	//		intake.requestState(state.elevatorState),
	//		elevator.requestState(state.elevatorState)
	//	);
	//}

}
