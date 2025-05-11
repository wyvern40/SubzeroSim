package frc.robot.subsystems.Intake;

import static edu.wpi.first.units.Units.*;

import java.util.function.Consumer;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    
	public enum IntakeState {
		DOWN(IntakeConstants.INTAKE_DOWN_ANGLE),
		UP(IntakeConstants.INTAKE_UP_ANGLE);

		private final Angle angle;

		IntakeState(Angle angle) {
			this.angle = angle;
		}
	}

	public class IntakeOutput {

		public IntakeState state;

		public Angle position;
		public AngularVelocity velocity;

		public Angle targetPosition;
		
	}

	private IntakeOutput output;

	private IntakeState state;

	private TalonFX pivotMotor;
	private TalonFX grabMotor;
	private TalonFX alignMotor;

	private TalonFXSimState pivotMotorSim;

	private final MotionMagicVoltage motionMagic = new MotionMagicVoltage(0);

	private Consumer<IntakeOutput> telemetryConsumer;

	private final SingleJointedArmSim armSim = new SingleJointedArmSim(
		DCMotor.getKrakenX60(1), 
		IntakeConstants.PIVOT_GEAR_RATIO,
		IntakeConstants.MOI,
		IntakeConstants.LENGTH,
		IntakeConstants.MIN_ANGLE.in(Radians),
		IntakeConstants.MAX_ANGLE.in(Radians),
		true,
		IntakeConstants.START_ANGLE.in(Radians)
	);

	public Intake() {

		setUpPivotMotor();
		setUpGrabMotor();
		setUpAlignMotor();

		state = IntakeState.UP;

		this.output = new IntakeOutput();
	}

	void setUpPivotMotor() {

		pivotMotor = new TalonFX(IntakeConstants.PIVOT_MOTOR_ID);

		var talonFXConfigs = new TalonFXConfiguration();

		talonFXConfigs.Slot0 = IntakeConstants.PIVOT_PID_CONFIGS;

		talonFXConfigs.MotionMagic.MotionMagicCruiseVelocity = IntakeConstants.PIVOT_MM_VELOCITY;
		talonFXConfigs.MotionMagic.MotionMagicAcceleration = IntakeConstants.PIVOT_MM_ACCELERATION;

		var limitConfigs = new CurrentLimitsConfigs();

		limitConfigs.StatorCurrentLimit = IntakeConstants.PIVOT_STATOR_CURRENT_LIMIT;
		limitConfigs.StatorCurrentLimitEnable = true;

		limitConfigs.SupplyCurrentLimit = IntakeConstants.PIVOT_SUPPLY_CURRENT_LIMIT;
		limitConfigs.SupplyCurrentLimitEnable = true;

		var feedbackConfigs = new FeedbackConfigs().withSensorToMechanismRatio(IntakeConstants.PIVOT_GEAR_RATIO);

		pivotMotor.getConfigurator().apply(talonFXConfigs);
		pivotMotor.getConfigurator().apply(limitConfigs);
		pivotMotor.getConfigurator().apply(feedbackConfigs);
		
		pivotMotorSim = pivotMotor.getSimState();
	}

	void setUpGrabMotor() {
		grabMotor = new TalonFX(IntakeConstants.GRAB_MOTOR_ID);

		var limitConfigs = new CurrentLimitsConfigs();

		limitConfigs.StatorCurrentLimit = IntakeConstants.GRAB_STATOR_CURRENT_LIMIT;
		limitConfigs.StatorCurrentLimitEnable = true;

		limitConfigs.SupplyCurrentLimit = IntakeConstants.GRAB_SUPPLY_CURRENT_LIMIT;
		limitConfigs.SupplyCurrentLimitEnable = true;

		grabMotor.getConfigurator().apply(limitConfigs);
	}

	void setUpAlignMotor() {
		alignMotor = new TalonFX(IntakeConstants.ALIGN_MOTOR_ID);

		var limitConfigs = new CurrentLimitsConfigs();

		limitConfigs.StatorCurrentLimit = IntakeConstants.ALIGN_STATOR_CURRENT_LIMIT;
		limitConfigs.StatorCurrentLimitEnable = true;

		limitConfigs.SupplyCurrentLimit = IntakeConstants.ALIGN_SUPPLY_CURRENT_LIMIT;
		limitConfigs.SupplyCurrentLimitEnable = true;

		alignMotor.getConfigurator().apply(limitConfigs);
	}

	public void simulationPeriodic() {
		
		pivotMotorSim.setSupplyVoltage(RobotController.getBatteryVoltage());

		armSim.setInput(pivotMotor.getMotorVoltage().getValueAsDouble());

		armSim.update(0.020);

		pivotMotorSim.setRawRotorPosition(Radians.of(armSim.getAngleRads() * IntakeConstants.PIVOT_GEAR_RATIO));
		pivotMotorSim.setRotorVelocity(RadiansPerSecond.of(armSim.getVelocityRadPerSec() * IntakeConstants.PIVOT_GEAR_RATIO));

		RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(armSim.getCurrentDrawAmps()));
		
		output.position = pivotMotor.getPosition().getValue();
		output.velocity = pivotMotor.getVelocity().getValue();

		output.targetPosition = Rotations.of(pivotMotor.getClosedLoopReference().getValue());

		output.state = state;

		if(telemetryConsumer != null) {
			telemetryConsumer.accept(output);
		}
	}

	public void registerTelemetry(Consumer<IntakeOutput> telemetryFunction) {
		telemetryConsumer = telemetryFunction;
	}

	public IntakeOutput getOutput() {
		return output;
	}

	private void stopRollers() {
		grabMotor.set(0.0);
		alignMotor.set(0.0);
	}
	
	private Command setPosition(Angle position) {
		return this.run(() -> {
			pivotMotor.setControl(motionMagic
				.withSlot(0)
				.withPosition(position)
			);
		});
	}

	public Command goUp() {
		this.state = IntakeState.UP;
		return this.setPosition(state.angle);
	}

	public Command goDown() {
		this.state = IntakeState.DOWN;
		return this.setPosition(state.angle);
	}

	public Command runRollers() {
		return this.runEnd(() -> {
			grabMotor.set(1.0);
			alignMotor.set(1.0);
		}, this::stopRollers);
	}
}
