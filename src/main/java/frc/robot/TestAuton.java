// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Add your docs here. */
public class TestAuton {
    Drivetrain drivetrain;
    PIDController pidController = new PIDController(1, 0, 0);

    public TestAuton(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
    }

    public void driveStraight(double distanceInMeters) {
        var output = pidController.calculate((drivetrain.getLeftMeters() + drivetrain.getRightMeters()) /2, distanceInMeters);
        SmartDashboard.putNumber("AutonOutput", output);
        drivetrain.setOutput(-output, -output);
    }
}
