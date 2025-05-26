package frc.robot.subsystems.Arm;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.GravityTypeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

public class ArmConstants {

	public static final int PIVOT_MOTOR_ID = 46;
	public static final int ROLLER_MOTOR_ID = 47;

	public static final double PIVOT_STATOR_CURRENT_LIMIT = 120.0;
	public static final double PIVOT_SUPPLY_CURRENT_LIMIT = 40.0;

	public static final double ROLLER_STATOR_CURRENT_LIMIT = 120.0;
	public static final double ROLLER_SUPPLY_CURRENT_LIMIT = 40.0;

	public static final double MM_VELOCITY = 2.0;
	public static final double MM_ACCELERATION = 4.0;

	public static final Slot0Configs PIVOT_PID_CONFIGS = new Slot0Configs()
		.withKP(10.0)
		.withKS(0.0)
		// I'm pretty sure the simulator wont let me tune this further
		.withKG(0.072)
		.withKV(8.8)
		.withKA(0.1)
		.withGravityType(GravityTypeValue.Arm_Cosine);
	
	public static final double GEAR_RATIO = 67.5;

	public static final double MOI = 0.10113578;
	public static final Distance LENGTH = Meters.of(0.57022782);

	public static final Angle MIN_ANGLE = Degrees.of(-290.0);
	public static final Angle MAX_ANGLE = Degrees.of(290.0);

	public static final Angle START_ANGLE = Degrees.of(0.0);

	public static final Angle CORAL_STOW_ANGLE = Degrees.of(-90.0);
	public static final Angle ALGAE_STOW_ANGLE = Degrees.of(0.0);

    public static final Angle CORAL_ALIGN_ANGLE = Degrees.of(45.0);
    public static final Angle CORAL_SCORE_ANGLE = Degrees.of(45.0);

    public static final Angle BARGE_SCORE_ANGLE = Degrees.of(60.0);
}
