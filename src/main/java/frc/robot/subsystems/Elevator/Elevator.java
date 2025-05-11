package frc.robot.subsystems.Elevator;

import static edu.wpi.first.units.Units.*;

import java.util.function.Consumer;

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
    
	public enum ElevatorState {
        L2(0.0),
        L3(0.0),
		L4(0.0),
		CORAL_STOW(0.0);

		private final double position;

		ElevatorState(double position) {
			this.position = position;
		}
	}

	public class ElevatorOutput {

		public ElevatorState state;

		public Angle position;
		public AngularVelocity velocity;

		public Angle targetPosition;
		
	}

	private ElevatorOutput output;

	private ElevatorState state;

	private TalonFX leaderMotor;
    private TalonFX followerMotor;

	private TalonFXSimState leaderMotorSim;

	private final MotionMagicVoltage motionMagic = new MotionMagicVoltage(0);

	private Consumer<ElevatorOutput> telemetryConsumer;

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

	public Elevator() {
		setUpMotors();

		state = ElevatorState.CORAL_STOW;

		output = new ElevatorOutput();
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

		var feedbackConfigs = new FeedbackConfigs().withSensorToMechanismRatio(ElevatorConstants.GEAR_RATIO);

		leaderMotor.getConfigurator().apply(talonFXConfigs);
		leaderMotor.getConfigurator().apply(limitConfigs);
		leaderMotor.getConfigurator().apply(feedbackConfigs);
		
		leaderMotorSim = leaderMotor.getSimState();
	}

	public void simulationPeriodic() {
		
		leaderMotorSim.setSupplyVoltage(RobotController.getBatteryVoltage());

		elevatorSim.setInput(leaderMotor.getMotorVoltage().getValueAsDouble());

		elevatorSim.update(0.020);

		leaderMotorSim.setRawRotorPosition(Radians.of(elevatorSim.getPositionMeters() * ElevatorConstants.GEAR_RATIO));
		leaderMotorSim.setRotorVelocity(RadiansPerSecond.of(elevatorSim.getVelocityMetersPerSecond() * ElevatorConstants.GEAR_RATIO));

		RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(elevatorSim.getCurrentDrawAmps()));
		
		output.position = leaderMotor.getPosition().getValue();
		output.velocity = leaderMotor.getVelocity().getValue();

		output.targetPosition = Rotations.of(leaderMotor.getClosedLoopReference().getValue());

		output.state = state;

		if(telemetryConsumer != null) {
			telemetryConsumer.accept(output);
		}
	}

	public void registerTelemetry(Consumer<ElevatorOutput> telemetryFunction) {
		telemetryConsumer = telemetryFunction;
	}

	public ElevatorOutput getOutput() {
		return output;
	}

}
