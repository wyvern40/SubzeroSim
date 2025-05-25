package frc.robot.subsystems.Elevator;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
    
	private static Elevator instance;

	public static synchronized Elevator getInstance() {
		if (instance == null) {
			instance = new Elevator();
		}

		return instance;
	}

	public enum ElevatorState {
		CORAL_STOW(ElevatorConstants.CORAL_STOW_SETPOINT),
        L2(ElevatorConstants.CORAL_L2_SETPOINT),
        L3(ElevatorConstants.CORAL_L3_SETPOINT),
		L4(ElevatorConstants.CORAL_L4_SETPOINT);

		private final double setpoint;

		ElevatorState(double setpoint) {
			this.setpoint = setpoint;
		}
	}

	public class ElevatorData {

		public ElevatorState state;

		public Distance position;
		public LinearVelocity velocity;

		public Distance targetPosition;
		public LinearVelocity targetVelocity;

		public Angle rotorPosition;
		public AngularVelocity rotorVelocity;
		
	}

	private ElevatorData data = new ElevatorData();

	private ElevatorState state;

	private TalonFX leaderMotor;
    private TalonFX followerMotor;

	private TalonFXSimState leaderMotorSim;

	private final MotionMagicVoltage motionMagic = new MotionMagicVoltage(0);

	private final ElevatorSim elevatorSim = new ElevatorSim(
		DCMotor.getKrakenX60(2), 
		ElevatorConstants.GEAR_RATIO,
		ElevatorConstants.MASS.in(Kilograms),
		ElevatorConstants.SPOOL_RADIUS.in(Meters),
		ElevatorConstants.MIN_POSITION.in(Meters),
		ElevatorConstants.MAX_POSITION.in(Meters),
		true,
		ElevatorConstants.START_POSITION.in(Meters)
	);

	private Elevator() {
		setUpMotors();

		state = ElevatorState.CORAL_STOW;
	}

	void setUpMotors() {

		leaderMotor = new TalonFX(ElevatorConstants.LEADER_MOTOR_ID);
        followerMotor = new TalonFX(ElevatorConstants.FOLLOWER_MOTOR_ID);

        followerMotor.setControl(new Follower(ElevatorConstants.LEADER_MOTOR_ID, true));

		var talonFXConfigs = new TalonFXConfiguration();

		talonFXConfigs.Slot0 = ElevatorConstants.PID_CONFIGS;

		talonFXConfigs.MotionMagic.MotionMagicCruiseVelocity = ElevatorConstants.MM_VELOCITY;
		talonFXConfigs.MotionMagic.MotionMagicAcceleration = ElevatorConstants.MM_ACCELERATION;

		var limitConfigs = new CurrentLimitsConfigs();

		limitConfigs.StatorCurrentLimit = ElevatorConstants.STATOR_CURRENT_LIMIT;
		limitConfigs.StatorCurrentLimitEnable = true;

		limitConfigs.SupplyCurrentLimit = ElevatorConstants.SUPPLY_CURRENT_LIMIT;
		limitConfigs.SupplyCurrentLimitEnable = true;

		var feedbackConfigs = new FeedbackConfigs().withSensorToMechanismRatio(ElevatorConstants.DISTANCE_TO_ROTATIONS);

		leaderMotor.getConfigurator().apply(talonFXConfigs);
		leaderMotor.getConfigurator().apply(limitConfigs);
		leaderMotor.getConfigurator().apply(feedbackConfigs);
		
		leaderMotorSim = leaderMotor.getSimState();
	}

	public void simulationPeriodic() {
		
		leaderMotorSim.setSupplyVoltage(RobotController.getBatteryVoltage());

		elevatorSim.setInput(leaderMotor.getMotorVoltage().getValueAsDouble());

		elevatorSim.update(0.020);
		
		leaderMotorSim.setRawRotorPosition(Rotations.of(elevatorSim.getPositionMeters() * ElevatorConstants.DISTANCE_TO_ROTATIONS));
		leaderMotorSim.setRotorVelocity(RotationsPerSecond.of(elevatorSim.getVelocityMetersPerSecond() * ElevatorConstants.DISTANCE_TO_ROTATIONS));

		RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(elevatorSim.getCurrentDrawAmps()));
		
		data.position = Meters.of(leaderMotor.getPosition().getValueAsDouble());
		data.velocity = MetersPerSecond.of(leaderMotor.getVelocity().getValueAsDouble());
		
		data.targetPosition = Meters.of(leaderMotor.getClosedLoopReference().getValue());
		data.targetVelocity = MetersPerSecond.of(leaderMotor.getClosedLoopReferenceSlope().getValue());

		data.rotorPosition = leaderMotor.getRotorPosition().getValue();
		data.rotorVelocity = leaderMotor.getRotorVelocity().getValue();

		data.state = state;
	}

	public ElevatorData getData() {
		return data;
	}
	
	private double setpointToPosition(double setpoint) {
		if(setpoint <= 1.0) {
			return setpoint * ElevatorConstants.MAX_CARRIAGE_DISTANCE.in(Meters);
		} else {
			return setpoint * (ElevatorConstants.MAX_POSITION.in(Meters) / 2.0);
		}
	}

	public Command requestState(ElevatorState state) {
		this.state = state;
		return this.run(() -> {
			leaderMotor.setControl(motionMagic
				.withSlot(0)
				.withPosition(setpointToPosition(state.setpoint))
			);
		});
	}

}
