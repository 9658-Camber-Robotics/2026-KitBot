package frc.robot.subsystems;

import java.util.List;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Ports;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.remote.TalonFXWrapper;
import yams.units.YUnits;

import static edu.wpi.first.units.Units.*;

public class ShooterSubsystem extends SubsystemBase {
    private static final AngularVelocity kVelocityTolerance = RPM.of(100);

    private final SmartMotorControllerConfig smcConfig = new SmartMotorControllerConfig(this)
            .withClosedLoopController(0.5, 2, 0)
            .withFeedforward(new SimpleMotorFeedforward(0, 12.0 / RadiansPerSecond.of(DCMotor.getKrakenX60(1).freeSpeedRadPerSec).in(RotationsPerSecond)))
            .withStatorCurrentLimit(Amps.of(120))
            .withSupplyCurrentLimit(Amps.of(70))
            .withIdleMode(SmartMotorControllerConfig.MotorMode.COAST);
//            .withMomentOfInertia(YUnits.PoundSquareFeet.of(1));

    private final TalonFX leftMotor, middleMotor, rightMotor;
    private final TalonFXWrapper leftMotorSMC, middleMotorSMC, rightMotorSMC;
    private final List<TalonFXWrapper> motors;

    public ShooterSubsystem() {
        leftMotor = new TalonFX(Ports.kShooterLeft, Ports.kRoboRioCANBus);
        middleMotor = new TalonFX(Ports.kShooterMiddle, Ports.kRoboRioCANBus);
        rightMotor = new TalonFX(Ports.kShooterRight, Ports.kRoboRioCANBus);
        leftMotorSMC = new TalonFXWrapper(leftMotor, DCMotor.getKrakenX60(1), smcConfig.clone().withMotorInverted(false));
        middleMotorSMC = new TalonFXWrapper(middleMotor, DCMotor.getKrakenX60(1), smcConfig.clone().withMotorInverted(true));
        rightMotorSMC = new TalonFXWrapper(rightMotor, DCMotor.getKrakenX60(1), smcConfig.clone().withMotorInverted(true));

        motors = List.of(leftMotorSMC, middleMotorSMC, rightMotorSMC);
        for (TalonFXWrapper motor : motors) {
            var cfg = (TalonFXConfiguration) motor.getMotorControllerConfig();
            var cfgrtr = ((TalonFX) motor.getMotorController()).getConfigurator();
            cfgrtr.apply(cfg.withVoltage(new VoltageConfigs()
                    .withPeakReverseVoltage(Volts.of(0))));
        }
    }

    private Command setRPM(AngularVelocity rpm) {
        return run(() -> {
                    for (final TalonFXWrapper motor : motors) {
                        motor.setVelocity(rpm);
                    }
                }
        );
    }

    private Command setPercentOutput(double percentOutput) {
        return run(() -> {
                for (final TalonFXWrapper motor : motors) {
                    motor.setDutyCycle(percentOutput);
                }
            }
        );
    }

    public Command shoot() {
        return run(() -> setPercentOutput(0.5));
    }

    public Command stop() {
        return run(() -> setPercentOutput(0));
    }

    public Command spinUpCommand(AngularVelocity rpm) {
        return runOnce(() -> setRPM(rpm))
                .andThen(Commands.waitUntil(this::isVelocityWithinTolerance));
    }

    public boolean isVelocityWithinTolerance() {
        return motors.stream().allMatch(motor -> {
//            final boolean isInVelocityMode = motor.getAppliedControl().equals(velocityRequest);
            if (motor.getMechanismSetpointVelocity().isEmpty())
                return false;
            final AngularVelocity currentVelocity = motor.getMechanismVelocity();
            final AngularVelocity targetVelocity = motor.getMechanismSetpointVelocity().get();
            return currentVelocity.isNear(targetVelocity, kVelocityTolerance);
        });
    }

    @Override
    public void periodic() {
        for (var motor : motors) {
            motor.updateTelemetry();
        }
    }

    @Override
    public void simulationPeriodic() {
        for (var motor : motors) {
            motor.simIterate();
        }
    }
}
