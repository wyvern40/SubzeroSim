package frc.robot.subsystems.Intake;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import frc.robot.util.LoggedTunableNumber;

public final class IntakeConstants {

	public static final Angle minAngle = Degrees.of(0.0);
	public static final Angle maxAngle = Degrees.of(130.0);

	public static final Angle startingAngle = Degrees.of(130);

	public static final Angle setpointTolerance = Degrees.of(2.0);

	public static final Angle pivotIntakeSetpoint = Degrees.of(0.0);
	public static final Angle pivotStowSetpoint = Degrees.of(130.0);

	public static final int pivotMotorID = 41;
	public static final int grabMotorID = 42;
	public static final int alignMotorID = 43;

	public static final double pivotStatorCurrentLimit = 120.0;
	public static final double pivotSupplyCurrentLimit = 40.0;

	public static final double grabStatorCurrentLimit = 120.0;
	public static final double grabSupplyCurrentLmit = 40.0;

	public static final double alignStatorCurrentLimit = 120.0;
	public static final double alignSupplyCurrentLimit = 40.0;
	
	public static final double pivotGearRatio = 75.7201646;

	public static final double moi = 0.06280341;
	public static final Distance length = Inches.of(13.669);

	
	public static final LoggedTunableNumber profileMaxVelocity = 
        new LoggedTunableNumber("/Elevator/Profile/Max Velocity", 2.0);

    public static final LoggedTunableNumber profileMaxAcceleration = 
        new LoggedTunableNumber("/Elevator/Profile/Max Acceleration", 4.0);
    
    public static final LoggedTunableNumber kP = 
        new LoggedTunableNumber("/Elevator/PID/kP", 5.0);
    
    public static final LoggedTunableNumber kS = 
        new LoggedTunableNumber("/Elevator/PID/kS", 0.0);
    
    public static final LoggedTunableNumber kG = 
        new LoggedTunableNumber("/Elevator/PID/kG", 0.0);

    public static final LoggedTunableNumber kV = 
        new LoggedTunableNumber("/Elevator/PID/kV", 10.1);

    public static final LoggedTunableNumber kA = 
        new LoggedTunableNumber("/Elevator/PID/kA", 0.3);
	
}
