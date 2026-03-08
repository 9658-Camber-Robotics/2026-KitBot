package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Intake;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.local.SparkWrapper;

public class IntakeRollerSubsystem extends SubsystemBase
{

  private final SparkMax motorController = new SparkMax(1, SparkLowLevel.MotorType.kBrushless);

  private final SmartMotorController intakeSMC = new SparkWrapper(motorController,
                                                                  Intake.motor,
                                                                  Intake.smc.clone().withSubsystem(this));
  private final FlyWheel             intake    = new FlyWheel(Intake.config.clone()
                                                                           .withSmartMotorController(intakeSMC));

  public IntakeRollerSubsystem() {}

  @Override
  public void simulationPeriodic()
  {
    intake.simIterate();
  }

  @Override
  public void periodic()
  {
    intake.simIterate();
  }

  public void setDutyCycle(double dutyCycle)
  {
    intake.setDutyCycleSetpoint(dutyCycle);
  }

  public void setVelocity(AngularVelocity velocity)
  {
    intake.setMechanismVelocitySetpoint(velocity);
  }

  public Command setDutyCycleCommand(double dutycycle)
  {
    return intake.set(dutycycle);
  }

  public Command setVelocityCommand(AngularVelocity of)
  {
    return intake.run(of);
  }
}