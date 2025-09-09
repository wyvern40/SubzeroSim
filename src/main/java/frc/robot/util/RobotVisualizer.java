package frc.robot.util;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj.DataLogManager;
import frc.robot.subsystems.Elevator.ElevatorConstants;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

public class RobotVisualizer {
    
    private final NetworkTableInstance inst = NetworkTableInstance.getDefault();

    private final NetworkTable simStateTable = inst.getTable("Simulation");

    private final StructArrayPublisher<Pose3d> mechanismPoses = simStateTable.getStructArrayTopic("MechanismPoses", Pose3d.struct).publish();

    private Pose3d[] mechanismPoseArray = new Pose3d[4];

    public RobotVisualizer() {
        DataLogManager.start();
    }

    public void updatePoses(Angle intakePosition, Distance elevatorPosition, Angle armPosition) {

        mechanismPoseArray[0] = new Pose3d(
            new Translation3d(0.0, 0.3302, 0.17145),
            new Rotation3d(intakePosition.in(Radians), 0.0, 0.0)
        );

        mechanismPoseArray[1] = new Pose3d(
            new Translation3d(0.0, 0.0, Math.max(0.0, elevatorPosition.in(Meters) - ElevatorConstants.maxCarriageDistance.in(Meters))),
            new Rotation3d()
        );

        mechanismPoseArray[2] = new Pose3d(
            new Translation3d(0.0, 0.0, elevatorPosition.in(Meters)),
            new Rotation3d()
        );

        mechanismPoseArray[3] = new Pose3d(
            new Translation3d(0.0, 0.0, mechanismPoseArray[2].getMeasureZ().in(Meters) + 0.22225),
            new Rotation3d(0.0, -armPosition.in(Radians), 0.0)
        );

        mechanismPoses.set(mechanismPoseArray);
    }
}