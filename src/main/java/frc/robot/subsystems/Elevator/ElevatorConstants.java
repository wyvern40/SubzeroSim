package frc.robot.subsystems.Elevator;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import frc.robot.util.LoggedTunableNumber;

public class ElevatorConstants  {
	
	public static final Distance minPosition = Inches.of(0.0);
	public static final Distance maxPosition = Inches.of(55.0);

	public static final Distance startingPosition = Inches.of(0.0);

	public static final Distance setpointTolerance = Inches.of(0.0);

	public static final Distance coralStowSetpoint = Inches.of(0.0);

	public static final Distance coralL2Setpoint = Inches.of(0.0);
	public static final Distance coralL3Setpoint = Inches.of(0.0);
	public static final Distance coralL4Setpoint = Inches.of(0.0);

	public static final int leaderMotorID = 44;
	public static final int followerMotorID = 45;

	public static final double statorCurrentLimit = 120.0;
	public static final double supplyCurrentLimit = 40.0;

	public static final double gearRatio = 4.0;

	public static final Mass mass = Pounds.of(10.95);
	public static final Distance spoolRadius = Inches.of(0.75); 

	public static final double rotationsToDistance = (2.0 * Math.PI * spoolRadius.in(Meters)) / gearRatio;
	public static final double distanceToRotations = gearRatio / ((2.0 * Math.PI * spoolRadius.in(Meters)));

	public static final LoggedTunableNumber profileMaxVelocity = 
        new LoggedTunableNumber("/Elevator/Profile/Max Velocity", 3.0);

    public static final LoggedTunableNumber profileMaxAcceleration = 
        new LoggedTunableNumber("/Elevator/Profile/Max Acceleration", 14.0);
    
    public static final LoggedTunableNumber kP = 
        new LoggedTunableNumber("/Elevator/PID/kP", 50.0);
    
    public static final LoggedTunableNumber kS = 
        new LoggedTunableNumber("/Elevator/PID/kS", 0.0);
    
    public static final LoggedTunableNumber kG = 
        new LoggedTunableNumber("/Elevator/PID/kG", 0.199707);

    public static final LoggedTunableNumber kV = 
        new LoggedTunableNumber("/Elevator/PID/kV", 3.98443);

    public static final LoggedTunableNumber kA = 
        new LoggedTunableNumber("/Elevator/PID/kA", 0.15);
	
}