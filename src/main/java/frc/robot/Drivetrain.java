// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonSRXSimCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.simulation.AnalogGyroSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Units;

/** Add your docs here. */
public class Drivetrain {
    private static final WPI_TalonSRX leftMaster = new WPI_TalonSRX(Constants.Drivetrain.kLeftMasterMotorPort);
    private static final WPI_TalonSRX leftFollower = new WPI_TalonSRX(Constants.Drivetrain.kLeftFollowerMotorPort);
    private static final WPI_TalonSRX rightMaster = new WPI_TalonSRX(Constants.Drivetrain.kRightMasterMotorPort);
    private static final WPI_TalonSRX rightFollower = new WPI_TalonSRX(Constants.Drivetrain.kRightFollowerMotorPort);

    private final XboxController drivePad = new XboxController(0);
    private final DifferentialDrive diffyDrive = new DifferentialDrive(leftMaster, rightMaster);

    private final AHRS gyro = new AHRS(SerialPort.Port.kMXP);

    private final DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(gyro.getRotation2d());

    // Object for simulated inputs into Talon.
    TalonSRXSimCollection m_leftDriveSim = leftMaster.getSimCollection();
    TalonSRXSimCollection m_rightDriveSim = rightMaster.getSimCollection();

    // Trajectory following PIDController
    private final PIDController m_leftPIDController = new PIDController(8.5, 0, 0);
    private final PIDController m_rightPIDController = new PIDController(8.5, 0, 0);

    private final SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(1, 3);

    public Drivetrain() {
        leftMaster.configOpenloopRamp(Constants.Drivetrain.kDriveRampRate);
        rightMaster.configOpenloopRamp(Constants.Drivetrain.kDriveRampRate);

        leftFollower.follow(leftMaster);
        rightFollower.follow(rightMaster);

        leftMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
        rightMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);

        if (RobotBase.isReal()) {
            leftMaster.setInverted(true);
            leftMaster.setSensorPhase(true);
        }

        leftMaster.setSelectedSensorPosition(0);
        rightMaster.setSelectedSensorPosition(0);

        gyro.reset();
    }

    public void drive() {
        var forward = -drivePad.getY(Hand.kLeft);
        var turn = drivePad.getX(Hand.kRight);

        SmartDashboard.putNumber("forward", forward);

        diffyDrive.arcadeDrive(turn, forward);
    }

    public void setOutput(double leftPercent, double rightPercent) {
        diffyDrive.tankDrive(-leftPercent, rightPercent);

        diffyDrive.feed();
    }

    /** Sets speeds to the drivetrain motors. */
    public void setSpeeds(double xSpeed, double rot) {
        var speeds = Constants.Drivetrain.kinematics.toWheelSpeeds(new ChassisSpeeds(xSpeed, 0, rot));

        var leftFeedforward = m_feedforward.calculate(speeds.leftMetersPerSecond);
        var rightFeedforward = m_feedforward.calculate(speeds.rightMetersPerSecond);

        SmartDashboard.putNumber("LeftMetersPerSecond", speeds.leftMetersPerSecond);
        var leftVelocity = Units.inchesToMeters(leftMaster.getSelectedSensorVelocity() * Constants.Drivetrain.kMagMultiplier);
        var rightVelocity = Units.inchesToMeters(rightMaster.getSelectedSensorVelocity() * Constants.Drivetrain.kMagMultiplier);

        double leftOutput =
            m_leftPIDController.calculate(leftVelocity, speeds.leftMetersPerSecond);
        double rightOutput =
            m_rightPIDController.calculate(rightVelocity, speeds.rightMetersPerSecond);

        SmartDashboard.putNumber("LeftOut", leftOutput);
        SmartDashboard.putNumber("RightOut", rightOutput);

        leftMaster.setVoltage(leftOutput);
        rightMaster.setVoltage(rightOutput);

        diffyDrive.feed();
    }

    public double getLeftMotor() {
        return leftMaster.get();
    }

    public double getRightMotor() {
        return rightMaster.get();
    }

    public void simulationPeriodic(DifferentialDrivetrainSim driveSim) {
            // Update all of our sensors.
        m_leftDriveSim.setQuadratureRawPosition(
            distanceToNativeUnits(
            driveSim.getLeftPositionMeters()));
        m_leftDriveSim.setQuadratureVelocity(
            velocityToNativeUnits(
            driveSim.getLeftVelocityMetersPerSecond()));
        m_rightDriveSim.setQuadratureRawPosition(
            distanceToNativeUnits(
            driveSim.getRightPositionMeters()));
        m_rightDriveSim.setQuadratureVelocity(
            velocityToNativeUnits(
            driveSim.getRightVelocityMetersPerSecond()));
    }

    public double getLeftMeters() {
        return Units.inchesToMeters(leftMaster.getSelectedSensorPosition() / Constants.Drivetrain.kMagMultiplier);
    }

    public double getRightMeters() {
        return Units.inchesToMeters(rightMaster.getSelectedSensorPosition() / Constants.Drivetrain.kMagMultiplier);
    }

    public void updateOdometry() {
        odometry.update(gyro.getRotation2d(), 
        getLeftMeters(), 
        getRightMeters());
    }

    public DifferentialDriveOdometry getOdometry() {
        return odometry;
    }

    public void resetOdometry(Pose2d pose) {
        setEncoders(0, 0);
        odometry.resetPosition(pose, gyro.getRotation2d());
    }

    public AHRS getGyro() {
        return gyro;
    }

    public void setEncoders(double leftEncoderPos, double rightEncoderPos) {
        leftMaster.setSelectedSensorPosition(leftEncoderPos * Constants.Drivetrain.kMagMultiplier);
        rightMaster.setSelectedSensorPosition(rightEncoderPos * Constants.Drivetrain.kMagMultiplier);
    }

    public void setRate(double leftEncoderRate, double rightEncoderRate) {
        leftMaster.set(ControlMode.Velocity, leftEncoderRate);
        rightMaster.set(ControlMode.Velocity, rightEncoderRate);
    }

    public int distanceToNativeUnits(double positionMeters){
        double wheelRotations = positionMeters/(2 * Math.PI * Units.inchesToMeters(Constants.Drivetrain.kWheelDiameter/2));
        double motorRotations = wheelRotations * Constants.Drivetrain.kGearRatio;
        int sensorCounts = (int)(motorRotations * Constants.Drivetrain.kEncoderTicksPerRevolution);
        return sensorCounts;
    }
    
    public int velocityToNativeUnits(double velocityMetersPerSecond){
        double wheelRotationsPerSecond = velocityMetersPerSecond/(2 * Math.PI * Units.inchesToMeters(Constants.Drivetrain.kWheelDiameter/2));
        double motorRotationsPerSecond = wheelRotationsPerSecond * Constants.Drivetrain.kGearRatio;
        double motorRotationsPer100ms = motorRotationsPerSecond / 10;
        int sensorCountsPer100ms = (int)(motorRotationsPer100ms * Constants.Drivetrain.kEncoderTicksPerRevolution);
        return sensorCountsPer100ms;
    }
}
