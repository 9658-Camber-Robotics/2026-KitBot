package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Kicker;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;

import static edu.wpi.first.units.Units.*;

public class KickerSubsystem extends SubsystemBase {

    private SparkMax spark = new SparkMax(4, MotorType.kBrushless);

    private SmartMotorController kickerSMC = new SparkWrapper(spark, Kicker.motor, Kicker.smc.clone().withSubsystem(this));

    private FlyWheel kicker = new FlyWheel(Kicker.config.clone().withSmartMotorController(kickerSMC));

    public void setDutycycle(double dutyCycle)
    {
      kicker.setDutyCycleSetpoint(dutyCycle);
    }

    public void setVelocity(AngularVelocity velocity)
    {
      kicker.setMechanismVelocitySetpoint(velocity);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        kicker.updateTelemetry();
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
        kicker.simIterate();
    }

  public Command setDutycycleCommand(double dutycycle)
  {
    return kicker.set(dutycycle);
  }
}