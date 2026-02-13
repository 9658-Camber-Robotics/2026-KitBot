package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.local.SparkWrapper;

import static edu.wpi.first.units.Units.*;

public class IntakeRollerSubsystem extends SubsystemBase {

    // make SparkMax
    // make SmartMotorControllerConfig
    // make SmartMotorController
    // DO NOT MAKE MECHANIMS

    private final SparkMax intake =  new SparkMax(1, SparkLowLevel.MotorType.kBrushless);
    private final SmartMotorControllerConfig smcConfig = new SmartMotorControllerConfig(this)
            .withClosedLoopController(300, 0, 0)
            .withFeedforward(new SimpleMotorFeedforward(0, 12.0 / RadiansPerSecond.of(DCMotor.getNEO(1).freeSpeedRadPerSec).in(RotationsPerSecond)))
            .withStatorCurrentLimit(Amps.of(120))
            .withSupplyCurrentLimit(Amps.of(70))
            .withIdleMode(SmartMotorControllerConfig.MotorMode.COAST);
    //          .withMomentOfInertia(YUnits.PoundSquareFeet.of(1));

    private final SmartMotorController intakeSMC = new SparkWrapper(intake, DCMotor.getNEO(1), smcConfig.clone());

    public IntakeRollerSubsystem() {}

    @Override
    public void simulationPeriodic() {
        // SMC.simIterate
    }

    @Override
    public void periodic() {
        // SMC.updateTelemetry()
    }

    public Command setIntakeRoller(double speed) {
        return run(() -> intake.set(speed));
    }

    public Command out(double speed) {
        return setIntakeRoller(speed * -1);
    }

    public Command in(double speed) {
        return setIntakeRoller(speed);
    }

    public Command stop() {
        return setIntakeRoller(0);
    }

}