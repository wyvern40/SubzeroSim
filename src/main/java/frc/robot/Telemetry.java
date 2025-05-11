package frc.robot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj.DataLogManager;
import frc.robot.subsystems.Intake.Intake.IntakeOutput;
import frc.robot.subsystems.Intake.Intake.IntakeState;

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

    private final NetworkTable simStateTable = inst.getTable("Simulation");

    private final StructArrayPublisher<Pose3d> mechanismPoses = simStateTable.getStructArrayTopic("MechanismPoses", Pose3d.struct).publish();

    public Telemetry() {
        DataLogManager.start();
        //mechanismPoses.set(new Pose3d[] {new Pose3d(new Translation3d(0.0, 0.3302, 0.17145), new Rotation3d())});
    }

    public void updateSwerveTelemetry(SwerveDriveState state) {
        drivePose.set(state.Pose);
        driveSpeeds.set(state.Speeds);
        driveModuleStates.set(state.ModuleStates);
        driveModuleTargets.set(state.ModuleTargets);
        driveModulePositions.set(state.ModulePositions);
    }

    public void updateIntakeTelemetry(IntakeOutput output) {
        mechanismPoses.set(new Pose3d[] {new Pose3d(
            new Translation3d(0.0, 0.3302, 0.17145), 
            new Rotation3d(output.position.in(Radians), 0.0, 0.0)
        )});

        intakeState.set(output.state.toString());
        intakePosition.set(output.position.in(Degrees));
        intakeVelocity.set(output.velocity.in(DegreesPerSecond));
        intakeTargetPosition.set(output.targetPosition.in(Degrees));
    }
}
