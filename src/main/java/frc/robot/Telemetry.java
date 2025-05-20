package frc.robot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator.ElevatorConstants;
import frc.robot.subsystems.Elevator.Elevator.ElevatorOutput;
import frc.robot.subsystems.Intake.Intake.IntakeOutput;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;

public class Telemetry {
    
    private final NetworkTableInstance inst = NetworkTableInstance.getDefault();

    private final NetworkTable driveStateTable = inst.getTable("Subsystems/Swerve");

    private final StructPublisher<Pose2d> drivePose = driveStateTable.getStructTopic("Pose", Pose2d.struct).publish();
    private final StructPublisher<ChassisSpeeds> driveSpeeds = driveStateTable.getStructTopic("Speeds", ChassisSpeeds.struct).publish();
    private final StructArrayPublisher<SwerveModuleState> driveModuleStates = driveStateTable.getStructArrayTopic("Module States", SwerveModuleState.struct).publish();
    private final StructArrayPublisher<SwerveModuleState> driveModuleTargets = driveStateTable.getStructArrayTopic("Module Targets", SwerveModuleState.struct).publish();
    private final StructArrayPublisher<SwerveModulePosition> driveModulePositions = driveStateTable.getStructArrayTopic("Module Positions", SwerveModulePosition.struct).publish();

    private final NetworkTable intakeStateTable = inst.getTable("Subsystems/Intake");

    private final StringPublisher intakeState = intakeStateTable.getStringTopic("State").publish();
    private final DoublePublisher intakePosition = intakeStateTable.getDoubleTopic("Position").publish();
    private final DoublePublisher intakeVelocity = intakeStateTable.getDoubleTopic("Velocity").publish();
    private final DoublePublisher intakeTargetPosition = intakeStateTable.getDoubleTopic("Target Position").publish();

    private final NetworkTable elevatorStateTable = inst.getTable("Subsystems/Elevator");

    private final StringPublisher elevatorState = elevatorStateTable.getStringTopic("State").publish();
    private final DoublePublisher elevatorPosition = elevatorStateTable.getDoubleTopic("Position").publish();
    private final DoublePublisher elevatorVelocity = elevatorStateTable.getDoubleTopic("Velocity").publish();
    private final DoublePublisher elevatorTargetPosition = elevatorStateTable.getDoubleTopic("Target Position").publish();
    private final DoublePublisher elevatorTargetVelocity = elevatorStateTable.getDoubleTopic("Target Velocity").publish();
    private final DoublePublisher elevatorRotorPosition = elevatorStateTable.getDoubleTopic("Rotor Position").publish();
    private final DoublePublisher elevatorRotorVelocity = elevatorStateTable.getDoubleTopic("Rotor Velocity").publish();

    private final NetworkTable simStateTable = inst.getTable("Simulation");

    private final StructArrayPublisher<Pose3d> mechanismPoses = simStateTable.getStructArrayTopic("MechanismPoses", Pose3d.struct).publish();
    private final StructPublisher<Pose2d> transformTest = simStateTable.getStructTopic("Please Fucking Work1", Pose2d.struct).publish();
    private final StructPublisher<Pose2d> transformTest1 = simStateTable.getStructTopic("Please Fucking Work2", Pose2d.struct).publish();
    private final StructPublisher<Pose2d> transformTest2 = simStateTable.getStructTopic("Please Fucking Work3", Pose2d.struct).publish();
    private final StructPublisher<Pose2d> transformTest3 = simStateTable.getStructTopic("Please Fucking Work4", Pose2d.struct).publish();
    private final StructPublisher<Pose2d> transformTest4 = simStateTable.getStructTopic("Please Fucking Work5", Pose2d.struct).publish();
    private final StructPublisher<Pose2d> transformTest5 = simStateTable.getStructTopic("Please Fucking Work6", Pose2d.struct).publish();
    private final StringPublisher command = simStateTable.getStringTopic("yippee").publish();

    private Pose3d[] mechanismPoseArray = new Pose3d[3];

    public Telemetry() {
        DataLogManager.start();
    }

    public void updateSuperstructureTelemetry(Command swerveCommand) {
        mechanismPoses.set(mechanismPoseArray);
        transformTest.set(FieldConstants.REEF_FACES[0]);
        transformTest1.set(FieldConstants.REEF_FACES[1]);
        transformTest2.set(FieldConstants.REEF_FACES[2]);
        transformTest3.set(FieldConstants.REEF_FACES[3]);
        transformTest4.set(FieldConstants.REEF_FACES[4]);
        transformTest5.set(FieldConstants.REEF_FACES[5]);
        if(swerveCommand != null) {command.set(swerveCommand.toString());};
    }

    public void updateSwerveTelemetry(SwerveDriveState state) {
        drivePose.set(state.Pose);
        driveSpeeds.set(state.Speeds);
        driveModuleStates.set(state.ModuleStates);
        driveModuleTargets.set(state.ModuleTargets);
        driveModulePositions.set(state.ModulePositions);
    }

    public void updateIntakeTelemetry(IntakeOutput output) {
        mechanismPoseArray[0] = new Pose3d(
            new Translation3d(0.0, 0.3302, 0.17145),
            new Rotation3d(output.position.in(Radians), 0.0, 0.0)
        );

        intakeState.set(output.state.toString());
        intakePosition.set(output.position.in(Degrees));
        intakeVelocity.set(output.velocity.in(DegreesPerSecond));
        intakeTargetPosition.set(output.targetPosition.in(Degrees));
    }

    public void updateElevatorTelemetry(ElevatorOutput output) {
        mechanismPoseArray[1] = new Pose3d(
            new Translation3d(0.0, 0.0, Math.max(0.0, output.position.in(Meters) - ElevatorConstants.MAX_CARRIAGE_DISTANCE.in(Meters))),
            new Rotation3d()
        );

        mechanismPoseArray[2] = new Pose3d(
            new Translation3d(0.0, 0.0, output.position.in(Meters)),
            new Rotation3d()
        );

        elevatorState.set(output.state.toString());
        elevatorPosition.set(output.position.in(Meters));
        elevatorVelocity.set(output.velocity.in(MetersPerSecond));
        elevatorTargetPosition.set(output.targetPosition.in(Meters));
        elevatorTargetVelocity.set(output.targetVelocity.in(MetersPerSecond));
        elevatorPosition.set(output.position.in(Meters));
        elevatorRotorPosition.set(output.rotorPosition.in(Rotations));
        elevatorRotorVelocity.set(output.rotorVelocity.in(RotationsPerSecond));
    }
}
