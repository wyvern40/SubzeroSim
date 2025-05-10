// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

	public static final int INTAKE_PIVOT_MOTOR_ID = 41;
	public static final int INTAKE_GRAB_MOTOR_ID = 42;
	public static final int INTAKE_ALIGN_MOTOR_ID = 42;

	public static final double INTAKE_PIVOT_REDUCTION = 75.7201646;

	public static final double INTAKE_MASS = 13.1956993;
	public static final double INTAKE_MOI = 0.13845787;
	public static final double INTAKE_PIVOT_LENGTH = 13.669;

	public static final double INTAKE_MIN_ANGLE_DEGREES = -160;
	public static final double INTAKE_MAX_ANGLE_DEGREES = -30;

	public static final double INTAKE_STARTING_ANGLE_DEGREES = -159.787;

	public static final double INTAKE_PIVOT_STATOR_CURRENT_LIMIT = 120.0;
	public static final double INTAKE_PIVOT_SUPPLY_CURRENT_LIMIT = 40.0;
}
