// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.system.plant.DCMotor;
import edu.wpi.first.wpilibj.util.Units;
import frc.robot.Drivetrain;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */

  private Drivetrain drivetrain = new Drivetrain();
  private TestAuton testAuto = new TestAuton(drivetrain);

  // Handle robot simulation
  private final DifferentialDrivetrainSim driveSim = new DifferentialDrivetrainSim(
    DCMotor.getCIM(2),
    7.29,
    7.5,
    60,
    Units.inchesToMeters(3),
    0.7112,
    null);

  private Field2d field2d = new Field2d();

  @Override
  public void robotInit() {
    SmartDashboard.putData("Field", field2d);
  }

  @Override
  public void robotPeriodic() {
    drivetrain.updateOdometry();
    SmartDashboard.putNumber("LeftMotor", drivetrain.getLeftMotor());
    SmartDashboard.putNumber("RightMotor", drivetrain.getRightMotor());
    SmartDashboard.putNumber("Robot Y", drivetrain.getOdometry().getPoseMeters().getY());
    field2d.setRobotPose(drivetrain.getOdometry().getPoseMeters());
  }

  @Override
  public void autonomousInit() {
  }

  @Override
  public void autonomousPeriodic() {
    testAuto.driveStraight(1);
  }

  @Override
  public void teleopInit() {
  }

  @Override
  public void teleopPeriodic() {
    drivetrain.drive();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {
    driveSim.setInputs(drivetrain.getLeftMotor() * RobotController.getInputVoltage(),
                       drivetrain.getRightMotor() * RobotController.getInputVoltage());

    driveSim.update(0.02);

    drivetrain.simulationPeriodic(driveSim);

    var navxSim = SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[0]");
    var angle = new SimDouble(SimDeviceDataJNI.getSimValueHandle(navxSim, "Yaw"));
    angle.set(-driveSim.getHeading().getDegrees());
  }
}
