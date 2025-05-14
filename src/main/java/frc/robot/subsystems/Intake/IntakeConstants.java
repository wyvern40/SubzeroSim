package frc.robot.subsystems.Intake;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.GravityTypeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

public final class IntakeConstants {

	public static final int PIVOT_MOTOR_ID = 41;
	public static final int GRAB_MOTOR_ID = 42;
	public static final int ALIGN_MOTOR_ID = 43;

	public static final double PIVOT_STATOR_CURRENT_LIMIT = 120.0;
	public static final double PIVOT_SUPPLY_CURRENT_LIMIT = 40.0;

	public static final double GRAB_STATOR_CURRENT_LIMIT = 120.0;
	public static final double GRAB_SUPPLY_CURRENT_LIMIT = 40.0;

	public static final double ALIGN_STATOR_CURRENT_LIMIT = 120.0;
	public static final double ALIGN_SUPPLY_CURRENT_LIMIT = 40.0;

	public static final double PIVOT_MM_VELOCITY = 2.0;
	public static final double PIVOT_MM_ACCELERATION = 4.0;

	public static final Slot0Configs PIVOT_PID_CONFIGS = new Slot0Configs()
		.withKP(5.0)
		.withKS(0.0)
		.withKG(0.0)
		.withKV(10.1)
		.withKA(0.3)
		.withGravityType(GravityTypeValue.Arm_Cosine);
	
	public static final double PIVOT_GEAR_RATIO = 75.7201646;

	public static final double MOI = 0.06280341;
	public static final Distance LENGTH = Inches.of(13.669);

	public static final Angle MIN_ANGLE = Degrees.of(0.0);
	public static final Angle MAX_ANGLE = Degrees.of(130.0);

	public static final Angle START_ANGLE = Degrees.of(130);

	public static final Angle INTAKE_DOWN_ANGLE = Degrees.of(0.0);
	public static final Angle INTAKE_UP_ANGLE = Degrees.of(130.0);
}
