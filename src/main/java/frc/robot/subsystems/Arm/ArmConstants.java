package frc.robot.subsystems.Arm;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import frc.robot.util.LoggedTunableNumber;

public class ArmConstants {

	public static final Angle minAngle = Degrees.of(-290.0);
	public static final Angle maxAngle = Degrees.of(290.0);

	public static final Angle startingAngle = Degrees.of(0.0);

	public static final Angle coralStowSetpoint = Degrees.of(-90.0);
    public static final Angle coralAlignSetpoint = Degrees.of(45.0);
    public static final Angle coralScoreSetpoint = Degrees.of(30.0);

	public static final double statorCurrentLimit = 120.0;
	public static final double supplyCurrentLimit = 40.0;

	public static final double gearRatio = 67.5;

	public static final int motorID = 46;

	public static final double moi = 0.10113578;
	public static final Distance length = Meters.of(0.57022782);

    public static final LoggedTunableNumber profileMaxVelocity = 
        new LoggedTunableNumber("/Arm/Profile/Max Velocity", 2.0);

    public static final LoggedTunableNumber profileMaxAcceleration = 
        new LoggedTunableNumber("/Arm/Profile/Max Acceleration", 4.0);
    
    public static final LoggedTunableNumber kP = 
        new LoggedTunableNumber("/Arm/PID/kP", 10.0);
    
    public static final LoggedTunableNumber kS = 
        new LoggedTunableNumber("/Arm/PID/kS", 0.0);
    
    public static final LoggedTunableNumber kG = 
        new LoggedTunableNumber("/Arm/PID/kG", 0.072);

    public static final LoggedTunableNumber kV = 
        new LoggedTunableNumber("/Arm/PID/kV", 8.8);

    public static final LoggedTunableNumber kA = 
        new LoggedTunableNumber("/Arm/PID/kA", 0.1);

}
