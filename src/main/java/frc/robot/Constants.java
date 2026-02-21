// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.units.measure.*;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.gearing.Sprocket;

import static edu.wpi.first.units.Units.*;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static class OperatorConstants {
        public static final int kDriverControllerPort = 0;
    }

    public static class ClimbConstants {

        public static final String ArmPositionKey = "ArmPosition";
        public static final String ArmPKey = "ArmP";

        // The P gain for the PID controller that drives this arm.
        public static final double ClimbKp = 50.0;
        public static final double ClimbKi = 0.0;
        public static final double ClimbKd = 0.0;

        // distance per pulse = (angle per revolution) / (pulses per revolution)
        //  = (2 * PI rads) / (4096 pulses)
        public static final double ArmEncoderDistPerPulse = 2.0 * Math.PI / 4096;

        public static final MechanismGearing ClimbGearBox = new MechanismGearing(GearBox.fromReductionStages(80), Sprocket.fromStages("16:48"));
        public static final Mass ClimbMass = Pounds.of(8); // Kilograms
        public static final Distance ClimbLength = Inches.of(30);
        public static final Angle softMaxAngle = Degrees.of(225);
        public static final Angle softMinAngle = Degrees.of(-45);
        public static final Angle hardMaxAngle = Degrees.of(225);
        public static final Angle hardMinAngle = Degrees.of(-45);
        public static final Angle startingAngle = Degrees.of(225);

        //yippee
        public static final Angle angle1 = Degrees.of(0); //Rotations.of(-0.74);
        public static final Angle angle2 = Degrees.of(15); //Rotations.of(-0.788);
        public static final Angle intakeAngle = Degrees.of(45); //Rotations.of(-0.69);
    }

    public static double maxSpeed;
}
