package frc.robot.subsystems.Elevator;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.Slot0Configs;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;

public class ElevatorConstants  {
    
	public static final int LEADER_MOTOR_ID = 44;
	public static final int FOLLOWER_MOTOR_ID = 45;

	public static final double STATOR_CURRENT_LIMIT = 120.0;
	public static final double SUPPLY_CURRENT_LIMIT = 40.0;

	public static final double MM_VELOCITY = 2.0;
	public static final double MM_ACCELERATION = 4.0;

	public static final Slot0Configs PID_CONFIGS = new Slot0Configs()
		.withKP(5.0)
		.withKS(0.0)
		.withKG(0.0)
		.withKV(10.1)
		.withKA(0.3);
	
	public static final double GEAR_RATIO = 4.0;

	public static final Mass MASS = Pounds.of(6.85);
	public static final Distance SPOOL_RADIUS = Inches.of(0.75);

	public static final Distance MIN_POSITION = Inches.of(0.0);
	public static final Distance MAX_POSITION = Inches.of(55.0);

	public static final Distance START_POSITION = Inches.of(0.0);

}