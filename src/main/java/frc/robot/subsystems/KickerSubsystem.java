package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;

import static edu.wpi.first.units.Units.*;

public class KickerSubsystem extends SubsystemBase {

    private SmartMotorControllerConfig smcConfig = new SmartMotorControllerConfig(this)
            .withControlMode(ControlMode.CLOSED_LOOP)
            .withClosedLoopController(1, 0, 0)
            .withSimClosedLoopController(1, 0, 0)
            .withFeedforward(new SimpleMotorFeedforward(0, 0, 0))
            .withSimFeedforward(new SimpleMotorFeedforward(0, 0, 0))
            .withTelemetry("ShooterMotor", TelemetryVerbosity.HIGH)
            .withGearing(new MechanismGearing(GearBox.fromReductionStages(3, 4)))
            .withMotorInverted(false)
            .withIdleMode(MotorMode.COAST)
            .withStatorCurrentLimit(Amps.of(40));

    private SparkMax spark = new SparkMax(4, MotorType.kBrushless);

    private SmartMotorController hopperMotorController = new SparkWrapper(spark, DCMotor.getNEO(1), smcConfig);

    public Command setHopper(double dutycycle) {return run(()-> hopperMotorController.setDutyCycle(dutycycle));}

    public void setHopperDutycycle(double dutycycle) {hopperMotorController.setDutyCycle(dutycycle);}

    public Command indexFuel() {return setHopper(.75);}
    public Command hopperOut() {return setHopper(-.75);}


    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        hopperMotorController.updateTelemetry();
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
        hopperMotorController.simIterate();
    }
}