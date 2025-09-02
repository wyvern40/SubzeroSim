package frc.robot.subsystems.Arm;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
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
        CORAL_STOW(ArmConstants.coralStowSetpoint),
        CORAL_ALIGN(ArmConstants.coralAlignSetpoint),
        CORAL_SCORE(ArmConstants.coralScoreSetpoint);

		private final Angle angle;

		ArmState(Angle angle) {
			this.angle = angle;
		}
    }

    public class ArmData {

        public ArmState state;

        public Angle position;
        public AngularVelocity velocity;

        public Angle targetPosition;
        public AngularVelocity targetVelocity;

    }

    private ArmData data = new ArmData();

    private ArmState state = ArmState.CORAL_STOW;

    private final TalonFX pivotMotor = new TalonFX(ArmConstants.motorID);
	//private final TalonFX rollerMotor = new TalonFX(ArmConstants.ROLLER_MOTOR_ID);

    private final TalonFXSimState pivotMotorSim = pivotMotor.getSimState();
    //private final TalonFXSimState rollerMotorSim = rollerMotor.getSimState();

    private final MotionMagicVoltage motionMagic = new MotionMagicVoltage(ArmConstants.startingAngle);

    private final SingleJointedArmSim armSim = new SingleJointedArmSim(
        DCMotor.getKrakenX60(1),
        ArmConstants.gearRatio,
        ArmConstants.moi,
        ArmConstants.length.in(Meters),
        ArmConstants.minAngle.in(Radians),
        ArmConstants.maxAngle.in(Radians),
        true,
        ArmConstants.startingAngle.in(Radians)
    );

    private Arm() {
        setUpPivotMotor();
    }

    private void applyPIDConfigs() {
        var talonFXConfigs = new TalonFXConfiguration();

        talonFXConfigs.Slot0 = new Slot0Configs()
            .withKP(ArmConstants.kP.get())
            .withKS(ArmConstants.kS.get())
            .withKG(ArmConstants.kG.get())
            .withKV(ArmConstants.kV.get())
            .withKV(ArmConstants.kA.get())
            .withGravityType(GravityTypeValue.Arm_Cosine);

        talonFXConfigs.MotionMagic.MotionMagicCruiseVelocity = ArmConstants.profileMaxVelocity.get();
		talonFXConfigs.MotionMagic.MotionMagicAcceleration = ArmConstants.profileMaxAcceleration.get();

        pivotMotor.getConfigurator().apply(talonFXConfigs);
    }

    private void setUpPivotMotor() {

        applyPIDConfigs();

        var limitConfigs = new CurrentLimitsConfigs();

        limitConfigs.StatorCurrentLimit = ArmConstants.statorCurrentLimit;
		limitConfigs.StatorCurrentLimitEnable = true;

		limitConfigs.SupplyCurrentLimit = ArmConstants.supplyCurrentLimit;
		limitConfigs.SupplyCurrentLimitEnable = true;

        var feedbackConfigs = new FeedbackConfigs().withSensorToMechanismRatio(ArmConstants.gearRatio);

		pivotMotor.getConfigurator().apply(limitConfigs);
		pivotMotor.getConfigurator().apply(feedbackConfigs);
    }

    public void simulationPeriodic() {
		
        if(
            ArmConstants.kP.hasChanged() ||
            ArmConstants.kS.hasChanged() ||
            ArmConstants.kG.hasChanged() ||
            ArmConstants.kV.hasChanged() ||
            ArmConstants.kA.hasChanged() ||
            ArmConstants.profileMaxVelocity.hasChanged() ||
            ArmConstants.profileMaxAcceleration.hasChanged()
        ) {
            applyPIDConfigs();
        }

		pivotMotorSim.setSupplyVoltage(RobotController.getBatteryVoltage());

		armSim.setInput(pivotMotor.getMotorVoltage().getValueAsDouble());

		armSim.update(0.020);

		pivotMotorSim.setRawRotorPosition(Radians.of(armSim.getAngleRads() * ArmConstants.gearRatio));
		pivotMotorSim.setRotorVelocity(RadiansPerSecond.of(armSim.getVelocityRadPerSec() * ArmConstants.gearRatio));

		RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(armSim.getCurrentDrawAmps()));
		
        data.position = pivotMotor.getPosition().getValue();
		data.velocity = pivotMotor.getVelocity().getValue();

		data.targetPosition = Rotations.of(pivotMotor.getClosedLoopReference().getValue());
        data.targetVelocity = RotationsPerSecond.of(pivotMotor.getClosedLoopReferenceSlope().getValue());

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
        });
	}
}
