package frc.robot.subsystems;

import java.util.function.Consumer;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;

public class Intake extends SubsystemBase {
    
	public class IntakeState {
		public double position;
		public double targetPosition;
	}

	IntakeState state;

	private TalonFX pivotMotor;
	private TalonFX grabMotor;
	private TalonFX alignMotor;

	private final MotionMagicVoltage motionMagic = new MotionMagicVoltage(0);

	private Consumer<IntakeState> updateTelemetry;

	private final SingleJointedArmSim armSim = new SingleJointedArmSim(
		DCMotor.getKrakenX60(1), 
		Constants.INTAKE_PIVOT_REDUCTION,
		Constants.INTAKE_MOI,
		Constants.INTAKE_PIVOT_LENGTH,
		Math.toRadians(Constants.INTAKE_MIN_ANGLE_DEGREES),
		Math.toRadians(Constants.INTAKE_MAX_ANGLE_DEGREES),
		true,
		Math.toRadians(Constants.INTAKE_STARTING_ANGLE_DEGREES)
	);

	public Intake() {

		pivotMotor = new TalonFX(Constants.INTAKE_PIVOT_MOTOR_ID);
		grabMotor = new TalonFX(Constants.INTAKE_GRAB_MOTOR_ID);
		alignMotor = new TalonFX(Constants.INTAKE_ALIGN_MOTOR_ID);

		var talonFXConfigs = new TalonFXConfiguration();

		talonFXConfigs.Slot0.kS = 0.0;
		talonFXConfigs.Slot0.kG = 0.0;
		talonFXConfigs.Slot0.kV = 1.0;
		talonFXConfigs.Slot0.kA = 0.0;
		talonFXConfigs.Slot0.kP = 0.0;

		talonFXConfigs.MotionMagic.MotionMagicCruiseVelocity = 1.0;
		talonFXConfigs.MotionMagic.MotionMagicAcceleration = 5.0;

		var limitConfigs = new CurrentLimitsConfigs();

		limitConfigs.StatorCurrentLimit = Constants.INTAKE_PIVOT_STATOR_CURRENT_LIMIT;
		limitConfigs.StatorCurrentLimitEnable = true;

		limitConfigs.SupplyCurrentLimit = Constants.INTAKE_PIVOT_SUPPLY_CURRENT_LIMIT;
		limitConfigs.SupplyCurrentLimitEnable = true;

		var feedbackConfigs = new FeedbackConfigs();

		feedbackConfigs.SensorToMechanismRatio = Constants.INTAKE_PIVOT_REDUCTION;

		pivotMotor.getConfigurator().apply(talonFXConfigs);
		pivotMotor.getConfigurator().apply(limitConfigs);
		pivotMotor.getConfigurator().apply(feedbackConfigs);

		this.state = new IntakeState();
	}

	public void simulationPeriodic() {

		armSim.setInput(pivotMotor.getMotorVoltage().getValueAsDouble());

		armSim.update(0.020);

		state.position = pivotMotor.getPosition().getValueAsDouble();
		state.targetPosition = motionMagic.Position;

		if(updateTelemetry != null) {
			updateTelemetry.accept(state);
		}
	}

	public void registerTelemetry(Consumer<IntakeState> telemetryFunction) {
		updateTelemetry = telemetryFunction;
	}

	public Command test() {
		return this.run(() -> {
			pivotMotor.setControl(motionMagic
					.withSlot(0)
					.withPosition(1.0)
			);
		});
	}

}
