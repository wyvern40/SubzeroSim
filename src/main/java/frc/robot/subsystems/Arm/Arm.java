package frc.robot.subsystems.Arm;

import static edu.wpi.first.units.Units.*;

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

public class Arm extends SubsystemBase {

    private static Arm instance;

    	public static synchronized Arm getInstance() {
		if (instance == null) {
			instance = new Arm();
		}

		return instance;
	}

    public enum ArmState {
        CORAL_STOW(ArmConstants.CORAL_STOW_ANGLE, 0.0),
		ALGAE_STOW(ArmConstants.ALGAE_STOW_ANGLE, 0.0),
        CORAL_HANDOFF(ArmConstants.CORAL_STOW_ANGLE, -1.0),
        CORAL_ALIGN(ArmConstants.CORAL_ALIGN_ANGLE, 0.0),
        CORAL_SCORE(ArmConstants.CORAL_SCORE_ANGLE, 0.0);

		private final Angle angle;
        private final double rollerSpeed;

		ArmState(Angle angle, double rollerSpeed) {
			this.angle = angle;
            this.rollerSpeed = rollerSpeed;
		}
    }

    public class ArmData {

        public ArmState state;

        public Angle position;
        public AngularVelocity velocity;

        public Angle targetPosition;

    }

    private ArmData data = new ArmData();

    private ArmState state = ArmState.CORAL_STOW;

    private final TalonFX pivotMotor = new TalonFX(ArmConstants.PIVOT_MOTOR_ID);
	private final TalonFX rollerMotor = new TalonFX(ArmConstants.ROLLER_MOTOR_ID);

    private final TalonFXSimState pivotMotorSim = pivotMotor.getSimState();
    private final TalonFXSimState rollerMotorSim = rollerMotor.getSimState();

    private final MotionMagicVoltage motionMagic = new MotionMagicVoltage(ArmConstants.START_ANGLE);

    private final SingleJointedArmSim armSim = new SingleJointedArmSim(
        DCMotor.getKrakenX60(1),
        ArmConstants.GEAR_RATIO,
        ArmConstants.MOI,
        ArmConstants.LENGTH.in(Meters),
        ArmConstants.MIN_ANGLE.in(Radians),
        ArmConstants.MAX_ANGLE.in(Radians),
        true,
        ArmConstants.START_ANGLE.in(Radians)
    );

    private Arm() {
        
        setUpPivotMotor();
        setUpRollerMotor();

    }

    private void setUpPivotMotor() {
        
        var talonFXConfigs = new TalonFXConfiguration();

		talonFXConfigs.Slot0 = ArmConstants.PIVOT_PID_CONFIGS;

		talonFXConfigs.MotionMagic.MotionMagicCruiseVelocity = ArmConstants.MM_VELOCITY;
		talonFXConfigs.MotionMagic.MotionMagicAcceleration = ArmConstants.MM_ACCELERATION;

        var limitConfigs = new CurrentLimitsConfigs();

        limitConfigs.StatorCurrentLimit = ArmConstants.PIVOT_STATOR_CURRENT_LIMIT;
		limitConfigs.StatorCurrentLimitEnable = true;

		limitConfigs.SupplyCurrentLimit = ArmConstants.PIVOT_SUPPLY_CURRENT_LIMIT;
		limitConfigs.SupplyCurrentLimitEnable = true;

        var feedbackConfigs = new FeedbackConfigs().withSensorToMechanismRatio(ArmConstants.GEAR_RATIO);

        pivotMotor.getConfigurator().apply(talonFXConfigs);
		pivotMotor.getConfigurator().apply(limitConfigs);
		pivotMotor.getConfigurator().apply(feedbackConfigs);
    }

    private void setUpRollerMotor() {

        var limitConfigs = new CurrentLimitsConfigs();

        limitConfigs.StatorCurrentLimit = ArmConstants.ROLLER_STATOR_CURRENT_LIMIT;
		limitConfigs.StatorCurrentLimitEnable = true;

		limitConfigs.SupplyCurrentLimit = ArmConstants.ROLLER_SUPPLY_CURRENT_LIMIT;
		limitConfigs.SupplyCurrentLimitEnable = true;

        pivotMotor.getConfigurator().apply(limitConfigs);
    }

    public void simulationPeriodic() {
		
		pivotMotorSim.setSupplyVoltage(RobotController.getBatteryVoltage());
		rollerMotorSim.setSupplyVoltage(RobotController.getBatteryVoltage());

		armSim.setInput(pivotMotor.getMotorVoltage().getValueAsDouble());

		armSim.update(0.020);

		pivotMotorSim.setRawRotorPosition(Radians.of(armSim.getAngleRads() * ArmConstants.GEAR_RATIO));
		pivotMotorSim.setRotorVelocity(RadiansPerSecond.of(armSim.getVelocityRadPerSec() * ArmConstants.GEAR_RATIO));

		RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(armSim.getCurrentDrawAmps()));
		RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(rollerMotor.getStatorCurrent().getValue().in(Amps)));
		
        data.position = pivotMotor.getPosition().getValue();
		data.velocity = pivotMotor.getVelocity().getValue();

		data.targetPosition = Rotations.of(pivotMotor.getClosedLoopReference().getValue());

		data.state = state;
	}

    public ArmData getData() {
		return data;
	}

    public Command requestState(ArmState state) {
		this.state = state;
		return this.run(() -> {
            pivotMotor.setControl(motionMagic
			    .withSlot(0)
			    .withPosition(state.angle)
		    );
            rollerMotor.set(state.rollerSpeed);
        });
	}
}
