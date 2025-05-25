package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.DataLogManager;
import frc.robot.subsystems.Elevator.ElevatorConstants;
import frc.robot.subsystems.Arm.Arm.ArmData;
import frc.robot.subsystems.Elevator.Elevator.ElevatorData;
import frc.robot.subsystems.Intake.Intake.IntakeData;

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

    private final NetworkTable armStateTable = inst.getTable("Subsystems/Arm");

    private final StringPublisher armState = armStateTable.getStringTopic("State").publish();
    private final DoublePublisher armPosition = armStateTable.getDoubleTopic("Position").publish();
    private final DoublePublisher armVelocity = armStateTable.getDoubleTopic("Velocity").publish();
    private final DoublePublisher armTargetPosition = armStateTable.getDoubleTopic("Target Position").publish();

    private final NetworkTable simStateTable = inst.getTable("Simulation");

    private final StructArrayPublisher<Pose3d> mechanismPoses = simStateTable.getStructArrayTopic("MechanismPoses", Pose3d.struct).publish();

    private Pose3d[] mechanismPoseArray = new Pose3d[3];

    private Supplier<IntakeData> intakeSupplier;
    private Supplier<ElevatorData> elevatorSupplier;
    private Supplier<ArmData> armSupplier;

    public Telemetry() {
        DataLogManager.start();
    }

    public void registerSuppliers(Supplier<IntakeData> intakeSupplier, Supplier<ElevatorData> elevatorSupplier, Supplier<ArmData> armSupplier) {
        this.intakeSupplier = intakeSupplier;
        this.elevatorSupplier = elevatorSupplier;
        this.armSupplier = armSupplier;
    }

    public void updateSuperstructureTelemetry() {
        mechanismPoses.set(mechanismPoseArray);
    }

    public void updateSwerveTelemetry(SwerveDriveState data) {
        drivePose.set(data.Pose);
        driveSpeeds.set(data.Speeds);
        driveModuleStates.set(data.ModuleStates);
        driveModuleTargets.set(data.ModuleTargets);
        driveModulePositions.set(data.ModulePositions);
    }

    public void updateIntakeTelemetry() {

        IntakeData data = intakeSupplier.get();

        mechanismPoseArray[0] = new Pose3d(
            new Translation3d(0.0, 0.3302, 0.17145),
            new Rotation3d(data.position.in(Radians), 0.0, 0.0)
        );

        intakeState.set(data.state.toString());
        intakePosition.set(data.position.in(Degrees));
        intakeVelocity.set(data.velocity.in(DegreesPerSecond));
        intakeTargetPosition.set(data.targetPosition.in(Degrees));
    }

    public void updateElevatorTelemetry() {
        
        ElevatorData data = elevatorSupplier.get();

        mechanismPoseArray[1] = new Pose3d(
            new Translation3d(0.0, 0.0, Math.max(0.0, data.position.in(Meters) - ElevatorConstants.MAX_CARRIAGE_DISTANCE.in(Meters))),
            new Rotation3d()
        );

        mechanismPoseArray[2] = new Pose3d(
            new Translation3d(0.0, 0.0, data.position.in(Meters)),
            new Rotation3d()
        );

        elevatorState.set(data.state.toString());
        elevatorPosition.set(data.position.in(Meters));
        elevatorVelocity.set(data.velocity.in(MetersPerSecond));
        elevatorTargetPosition.set(data.targetPosition.in(Meters));
        elevatorTargetVelocity.set(data.targetVelocity.in(MetersPerSecond));
        elevatorPosition.set(data.position.in(Meters));
        elevatorRotorPosition.set(data.rotorPosition.in(Rotations));
        elevatorRotorVelocity.set(data.rotorVelocity.in(RotationsPerSecond));
    }

    public void updateArmTelemetry() {

        ArmData data = armSupplier.get();

        armState.set(data.state.toString());
        armPosition.set(data.position.in(Degrees));
        armVelocity.set(data.velocity.in(DegreesPerSecond));
        armTargetPosition.set(data.position.in(Degrees));
    }
}
