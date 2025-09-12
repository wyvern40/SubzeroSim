package frc.robot.subsystems.Intake;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import lombok.Getter;

public class Intake extends SubsystemBase {

	public enum IntakeState {
		INTAKE(IntakeConstants.pivotIntakeSetpoint, true),
		STOW(IntakeConstants.pivotStowSetpoint, false);

		private final Angle angle;
		private final boolean runRollers;

		IntakeState(Angle angle, boolean runRollers) {
			this.angle = angle;
			this.runRollers = runRollers;
		}
	}

	public class IntakeData {

		@Logged(name = "State")
		public IntakeState state;

		@Logged(name = "Position")
		public Angle position;
		@Logged(name = "Velocity")
		public AngularVelocity velocity;

		@Logged(name = "TargetPosition")
		public Angle targetPosition;
		
	}

	@Getter
	@Logged(name = "Data")
	private IntakeData data;

	private IntakeState state = IntakeState.STOW;

	private final TalonFX pivotMotor = new TalonFX(IntakeConstants.pivotMotorID);
	private final TalonFX grabMotor = new TalonFX(IntakeConstants.grabMotorID);
	private final TalonFX alignMotor = new TalonFX(IntakeConstants.alignMotorID);

	private final TalonFXSimState pivotMotorSim = pivotMotor.getSimState();
	private final TalonFXSimState grabMotorSim = grabMotor.getSimState();
	private final TalonFXSimState alignMotorSim = alignMotor.getSimState();

	private final MotionMagicVoltage motionMagic = new MotionMagicVoltage(0);

	private final SingleJointedArmSim armSim = new SingleJointedArmSim(
		DCMotor.getKrakenX60(1), 
		IntakeConstants.pivotGearRatio,
		IntakeConstants.moi,
		IntakeConstants.length.in(Meters),
		IntakeConstants.minAngle.in(Radians),
		IntakeConstants.maxAngle.in(Radians),
		true,
		IntakeConstants.startingAngle.in(Radians)
	);

	public Intake() {

		setUpPivotMotor();
		setUpGrabMotor();
		setUpAlignMotor();

		this.data = new IntakeData();

	}

	private void applyPivotPIDConfigs() {

        var talonFXConfigs = new TalonFXConfiguration();

        talonFXConfigs.Slot0 = new Slot0Configs()
            .withKP(IntakeConstants.kP.get())
            .withKS(IntakeConstants.kS.get())
            .withKG(IntakeConstants.kG.get())
            .withKV(IntakeConstants.kV.get())
            .withKV(IntakeConstants.kA.get())
            .withGravityType(GravityTypeValue.Arm_Cosine);

        talonFXConfigs.MotionMagic.MotionMagicCruiseVelocity = IntakeConstants.profileMaxVelocity.get();
		talonFXConfigs.MotionMagic.MotionMagicAcceleration = IntakeConstants.profileMaxAcceleration.get();

        pivotMotor.getConfigurator().apply(talonFXConfigs);
    }

	void setUpPivotMotor() {

		applyPivotPIDConfigs();

		var limitConfigs = new CurrentLimitsConfigs();

		limitConfigs.StatorCurrentLimit = IntakeConstants.pivotStatorCurrentLimit;
		limitConfigs.StatorCurrentLimitEnable = true;

		limitConfigs.SupplyCurrentLimit = IntakeConstants.pivotSupplyCurrentLimit;
		limitConfigs.SupplyCurrentLimitEnable = true;

		var feedbackConfigs = new FeedbackConfigs().withSensorToMechanismRatio(IntakeConstants.pivotGearRatio);

		pivotMotor.getConfigurator().apply(limitConfigs);
		pivotMotor.getConfigurator().apply(feedbackConfigs);

	}

	void setUpGrabMotor() {

		var limitConfigs = new CurrentLimitsConfigs();

		limitConfigs.StatorCurrentLimit = IntakeConstants.grabStatorCurrentLimit;
		limitConfigs.StatorCurrentLimitEnable = true;

		limitConfigs.SupplyCurrentLimit = IntakeConstants.grabSupplyCurrentLmit;
		limitConfigs.SupplyCurrentLimitEnable = true;

		grabMotor.getConfigurator().apply(limitConfigs);
	}

	void setUpAlignMotor() {

		var limitConfigs = new CurrentLimitsConfigs();

		limitConfigs.StatorCurrentLimit = IntakeConstants.alignStatorCurrentLimit;
		limitConfigs.StatorCurrentLimitEnable = true;

		limitConfigs.SupplyCurrentLimit = IntakeConstants.alignSupplyCurrentLimit;
		limitConfigs.SupplyCurrentLimitEnable = true;

		alignMotor.getConfigurator().apply(limitConfigs);
	}

	public void simulationPeriodic() {
		
		pivotMotor.setControl(motionMagic
			.withSlot(0)
			.withPosition(state.angle)
		);

		if(state.runRollers) {
			grabMotor.set(1.0);
			alignMotor.set(1.0);
		} else {
			grabMotor.set(0.0);
			alignMotor.set(0.0);
		}

		if(
            IntakeConstants.kP.hasChanged() ||
            IntakeConstants.kS.hasChanged() ||
            IntakeConstants.kG.hasChanged() ||
            IntakeConstants.kV.hasChanged() ||
            IntakeConstants.kA.hasChanged() ||
            IntakeConstants.profileMaxVelocity.hasChanged() ||
            IntakeConstants.profileMaxAcceleration.hasChanged()
        ) {
            applyPivotPIDConfigs();
        }

		pivotMotorSim.setSupplyVoltage(RobotController.getBatteryVoltage());
		grabMotorSim.setSupplyVoltage(RobotController.getBatteryVoltage());
		alignMotorSim.setSupplyVoltage(RobotController.getBatteryVoltage());

		armSim.setInput(pivotMotor.getMotorVoltage().getValueAsDouble());

		armSim.update(0.020);

		pivotMotorSim.setRawRotorPosition(Radians.of(armSim.getAngleRads() * IntakeConstants.pivotGearRatio));
		pivotMotorSim.setRotorVelocity(RadiansPerSecond.of(armSim.getVelocityRadPerSec() * IntakeConstants.pivotGearRatio));

		RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(armSim.getCurrentDrawAmps()));
		RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(grabMotor.getStatorCurrent().getValue().in(Amps)));
		RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(grabMotor.getStatorCurrent().getValue().in(Amps)));
		
		data.position = pivotMotor.getPosition().getValue();
		data.velocity = pivotMotor.getVelocity().getValue();

		data.targetPosition = Rotations.of(pivotMotor.getClosedLoopReference().getValue());

		data.state = state;
	}

	public boolean atSetpoint() {
		return data.position.minus(state.angle).abs(Degrees) < IntakeConstants.setpointTolerance.in(Degrees);
	}

	public Command swapState(IntakeState state) {
		return this.runOnce(() -> this.state = state);
	}

}
