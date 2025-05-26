package frc.robot.subsystems.Elevator;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.Slot0Configs;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;

public class ElevatorConstants  {
    
	public static final int LEADER_MOTOR_ID = 44;
	public static final int FOLLOWER_MOTOR_ID = 45;

	public static final double STATOR_CURRENT_LIMIT = 120.0;
	public static final double SUPPLY_CURRENT_LIMIT = 40.0;

	public static final double MM_VELOCITY = 3.0;
	public static final double MM_ACCELERATION = 14.0;

	public static final Slot0Configs PID_CONFIGS = new Slot0Configs()
		.withKP(50.0)
		.withKS(0.0)
		.withKG(0.199707)
		.withKV(5.0)
		.withKV(3.98443)
		.withKA(0.15);
	
	public static final double GEAR_RATIO = 4.0;

	public static final Mass MASS = Pounds.of(10.95);
	public static final Distance SPOOL_RADIUS = Inches.of(0.75); 

	public static final double ROTATIONS_TO_DISTANCE = (2.0 * Math.PI * SPOOL_RADIUS.in(Meters)) / GEAR_RATIO;
	public static final double DISTANCE_TO_ROTATIONS = GEAR_RATIO / ((2.0 * Math.PI * SPOOL_RADIUS.in(Meters)));

	// Max height the Carriage can travel to without moving the first stage
	public static final Distance MAX_CARRIAGE_DISTANCE = Inches.of(26);

	public static final Distance MIN_POSITION = Inches.of(0.0);
	public static final Distance MAX_POSITION = Inches.of(55.0);

	public static final Distance START_POSITION = Inches.of(0.0);

	public static final double CORAL_STOW_SETPOINT = 1.25;

	public static final double CORAL_L2_SETPOINT = 1.0;
	public static final double CORAL_L3_SETPOINT = 1.5;
	public static final double CORAL_L4_SETPOINT = 2.0;
}