// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/** Add your docs here. */
public class Constants {
    class Drivetrain {
        public static final int kLeftMasterMotorPort = 0;
        public static final int kLeftFollowerMotorPort = 1;
        public static final int kRightMasterMotorPort = 2;
        public static final int kRightFollowerMotorPort = 3;

        public static final double kDriveRampRate = 0.6;

        //encoder constants
        public static final int kEncoderTicksPerRevolution = 4096;
        public static final int kWheelDiameter = 6; //6 inches
        public static final double kGearRatio = 5.4;
        public static final double kPi = 3.14159265;
        public static final double kMagMultiplier = ((kEncoderTicksPerRevolution*kGearRatio)/(kPi * kWheelDiameter));

    }
}
