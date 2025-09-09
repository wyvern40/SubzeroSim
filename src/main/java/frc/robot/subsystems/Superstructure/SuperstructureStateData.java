package frc.robot.subsystems.Superstructure;

import frc.robot.subsystems.Arm.Arm.ArmState;
import frc.robot.subsystems.Elevator.Elevator.ElevatorState;
import frc.robot.subsystems.Intake.Intake.IntakeState;
import frc.robot.subsystems.Swerve.SwerveDrive.SwerveState;

import lombok.Getter;
import lombok.Builder;

@Getter
@Builder
public class SuperstructureStateData {
    @Builder.Default private SwerveState swerveState = SwerveState.DRIVER_CONTROL;
    @Builder.Default private ElevatorState elevatorState = ElevatorState.CORAL_STOW;
    @Builder.Default private ArmState armState = ArmState.CORAL_STOW;
    @Builder.Default private IntakeState intakeState = IntakeState.STOW;
}
