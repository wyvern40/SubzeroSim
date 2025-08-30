package frc.robot.subsystems.Superstructure;

import frc.robot.subsystems.Swerve.SwerveDrive.SwerveState;
import frc.robot.subsystems.Elevator.Elevator.ElevatorState;

import java.util.HashMap;
import java.util.List;

import frc.robot.subsystems.Arm.Arm.ArmState;
import frc.robot.subsystems.Intake.Intake.IntakeState;

public enum SuperstructureState {
    
    STOW(SuperstructureStateData.builder()
        .swerveState(SwerveState.DRIVER_CONTROL)
        .elevatorState(ElevatorState.CORAL_STOW)
        .armState(ArmState.CORAL_STOW)
        .intakeState(IntakeState.STOW)
        .build()
    ),
    INTAKE_CORAL(SuperstructureStateData.builder()
        .swerveState(SwerveState.DRIVER_CONTROL)
        .elevatorState(ElevatorState.CORAL_STOW)
        .armState(ArmState.CORAL_STOW)
        .intakeState(IntakeState.INTAKE)
        .build()
    ),
    DRIVE_TO_REEF(SuperstructureStateData.builder()
        .swerveState(SwerveState.PATH_TO_REEF)
        .elevatorState(ElevatorState.CORAL_STOW)
        .armState(ArmState.CORAL_STOW)
        .intakeState(IntakeState.STOW)
        .build()
    ),
    L2_ALIGN(SuperstructureStateData.builder()
        .swerveState(SwerveState.ALIGN_TO_REEF)
        .elevatorState(ElevatorState.L2)
        .armState(ArmState.CORAL_ALIGN)
        .build()
    ),
    L3_ALIGN(SuperstructureStateData.builder()
        .swerveState(SwerveState.ALIGN_TO_REEF)
        .elevatorState(ElevatorState.L3)
        .armState(ArmState.CORAL_ALIGN)
        .build()
    ),
    L4_ALIGN(SuperstructureStateData.builder()
        .swerveState(SwerveState.ALIGN_TO_REEF)
        .elevatorState(ElevatorState.L4)
        .armState(ArmState.CORAL_ALIGN)
        .build()
    ),
    L2_SCORE(SuperstructureStateData.builder()
        .swerveState(SwerveState.STOPPED)
        .elevatorState(ElevatorState.L2)
        .armState(ArmState.CORAL_SCORE)
        .build()
    ),
    L3_SCORE(SuperstructureStateData.builder()
        .swerveState(SwerveState.STOPPED)
        .elevatorState(ElevatorState.L3)
        .armState(ArmState.CORAL_SCORE)
        .build()
    ),
    L4_SCORE(SuperstructureStateData.builder()
        .swerveState(SwerveState.STOPPED)
        .elevatorState(ElevatorState.L4)
        .armState(ArmState.CORAL_SCORE)
        .build()
    );

    SuperstructureStateData data;

    private SuperstructureState(SuperstructureStateData data) {
        this.data = data;
    }

    private static final HashMap<SuperstructureState, List<SuperstructureState>> stateConnections;

    static {
        stateConnections = new HashMap<>();
        stateConnections.put(STOW, List.of(DRIVE_TO_REEF, INTAKE_CORAL));
        stateConnections.put(INTAKE_CORAL, List.of(STOW));
        stateConnections.put(DRIVE_TO_REEF, List.of(L2_ALIGN, L3_ALIGN, L4_ALIGN));
        stateConnections.put(L2_ALIGN, List.of(L2_SCORE));
        stateConnections.put(L3_ALIGN, List.of(L3_SCORE));
        stateConnections.put(L4_ALIGN, List.of(L4_SCORE));
        stateConnections.put(L2_SCORE, List.of(STOW));
        stateConnections.put(L3_SCORE, List.of(STOW));
        stateConnections.put(L4_SCORE, List.of(STOW));
    }
    
    public List<SuperstructureState> getConnectedStates() {
        return stateConnections.get(this);
    }
}
