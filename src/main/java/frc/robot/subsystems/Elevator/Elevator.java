package frc.robot.subsystems.Elevator;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.epilogue.Logged;
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
import lombok.Getter;

public class Elevator extends SubsystemBase {
    
	private static Elevator instance;

	public static synchronized Elevator getInstance() {
		if (instance == null) {
			instance = new Elevator();
		}

		return instance;
	}

	public enum ElevatorState {
		CORAL_STOW(ElevatorConstants.coralStowSetpoint),
        L2(ElevatorConstants.coralL2Setpoint),
        L3(ElevatorConstants.coralL3Setpoint),
		L4(ElevatorConstants.coralL4Setpoint);

		private final Distance setpoint;

		ElevatorState(Distance setpoint) {
			this.setpoint = setpoint;
		}
	}

	@Getter
	public class ElevatorData {

		@Logged(name = "State")
		public ElevatorState state;

		@Logged(name = "Position")
		public Distance position;
		@Logged(name = "Velocity")
		public LinearVelocity velocity;

		@Logged(name = "Target Position")
		public Distance targetPosition;
		@Logged(name = "Target Velocity")
		public LinearVelocity targetVelocity;

		@Logged(name = "Rotor Position")
		public Angle rotorPosition;
		@Logged(name = "Rotor Velocity")
		public AngularVelocity rotorVelocity;
		
	}

	@Logged(name = "Data")
	private final ElevatorData data = new ElevatorData();

	private ElevatorState state;

	private final TalonFX leaderMotor = new TalonFX(ElevatorConstants.leaderMotorID);
    private final TalonFX followerMotor = new TalonFX(ElevatorConstants.followerMotorID);

	private final TalonFXSimState leaderMotorSim = leaderMotor.getSimState();

	private final MotionMagicVoltage motionMagic = new MotionMagicVoltage(0);

	private final ElevatorSim elevatorSim = new ElevatorSim(
		DCMotor.getKrakenX60(2), 
		ElevatorConstants.gearRatio,
		ElevatorConstants.mass.in(Kilograms),
		ElevatorConstants.spoolRadius.in(Meters),
		ElevatorConstants.minPosition.in(Meters),
		ElevatorConstants.maxPosition.in(Meters),
		true,
		ElevatorConstants.startingPosition.in(Meters)
	);

	public boolean atSetpoint() {
        return data.position.minus(state.setpoint).abs(Meters) < ElevatorConstants.setpointTolerance.in(Meters);
    }

	private Elevator() {
		setUpMotors();
		state = ElevatorState.CORAL_STOW;
	}

	private void applyPIDConfigs() {
        var talonFXConfigs = new TalonFXConfiguration();

        talonFXConfigs.Slot0 = new Slot0Configs()
            .withKP(ElevatorConstants.kP.get())
            .withKS(ElevatorConstants.kS.get())
            .withKG(ElevatorConstants.kG.get())
            .withKV(ElevatorConstants.kV.get())
            .withKV(ElevatorConstants.kA.get())
            .withGravityType(GravityTypeValue.Elevator_Static);

        talonFXConfigs.MotionMagic.MotionMagicCruiseVelocity = ElevatorConstants.profileMaxVelocity.get();
		talonFXConfigs.MotionMagic.MotionMagicAcceleration = ElevatorConstants.profileMaxAcceleration.get();

        leaderMotor.getConfigurator().apply(talonFXConfigs);
    }

	void setUpMotors() {

		applyPIDConfigs();

        followerMotor.setControl(new Follower(ElevatorConstants.leaderMotorID, true));

		var limitConfigs = new CurrentLimitsConfigs();

		limitConfigs.StatorCurrentLimit = ElevatorConstants.statorCurrentLimit;
		limitConfigs.StatorCurrentLimitEnable = true;

		limitConfigs.SupplyCurrentLimit = ElevatorConstants.supplyCurrentLimit;
		limitConfigs.SupplyCurrentLimitEnable = true;

		var feedbackConfigs = new FeedbackConfigs().withSensorToMechanismRatio(ElevatorConstants.distanceToRotations);

		leaderMotor.getConfigurator().apply(limitConfigs);
		leaderMotor.getConfigurator().apply(feedbackConfigs);

	}

	public void simulationPeriodic() {
		
		leaderMotorSim.setSupplyVoltage(RobotController.getBatteryVoltage());

		elevatorSim.setInput(leaderMotor.getMotorVoltage().getValueAsDouble());

		elevatorSim.update(0.020);
		
		leaderMotorSim.setRawRotorPosition(Rotations.of(elevatorSim.getPositionMeters() * ElevatorConstants.distanceToRotations));
		leaderMotorSim.setRotorVelocity(RotationsPerSecond.of(elevatorSim.getVelocityMetersPerSecond() * ElevatorConstants.distanceToRotations));

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

	public Command requestState(ElevatorState state) {
		this.state = state;
		return this.run(() -> {
			leaderMotor.setControl(motionMagic
				.withSlot(0)
				.withPosition(state.setpoint.in(Meters) * ElevatorConstants.distanceToRotations)
			);
		});
	}

}
