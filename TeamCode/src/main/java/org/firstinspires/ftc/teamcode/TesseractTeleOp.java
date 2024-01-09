package org.firstinspires.ftc.teamcode;

import java.lang.*;
import java.util.Arrays;
import java.util.Collections;
import java.util.Vector;


import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;
import org.apache.commons.math3.linear.FieldVectorPreservingVisitor;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.config.TesseractConfig;

@TeleOp(name="Tesseract")
public class TesseractTeleOp extends OpMode {
    public double lerp(double start, double target, double alpha) {
        double output = start + (target - start) * alpha;
        return output;
    }

    // Motors
    public DcMotor motorFL;
    public DcMotor motorFR;
    public DcMotor motorBL;
    public DcMotor motorBR;
    public DcMotor craneL;
    public DcMotor craneR;
    public DcMotor Arm;
    public Servo clawL;
    public Servo clawR;
    double servoOpenPos = 0;
    double servoClosePos = 1;
    double powerFL = 0;
    double powerBL = 0;
    double powerFR = 0;
    double powerBR = 0;
    public IMU imu;
    public IMU.Parameters imuParams;
    public DcMotor motorTest;
    public YawPitchRollAngles robotOrientation;
    public double easeTime = TesseractConfig.easeAmount; // Alpha for Joystick Lerp
    public boolean easingEnabled = true; // Disable if causing problems
    public boolean angleDrivingEnabled = true; // Disable if causing problems
    public double craneTopLimit = 1.0;
    public double craneBottomLimit = 0.0;
    public double craneHeight = 0.0;
    // setup joystick variables
    Vector2D LJoyPos = Vector2D.ZERO;
    double RJoyX = 0.0;
    double LJoyAngle = 0.0;
    double LJoyMagnitude = 0.0;
    double currentRJX = RJoyX;
    double craneSpeed = 1;
    double armSpeed = 2;
    Vector2D currentLJPos = Vector2D.ZERO;

    private void driveWithAngle(double yaw) {
        double anglePosX = Math.sin(LJoyAngle - yaw) * LJoyMagnitude;
        double anglePosY = Math.cos(LJoyAngle - yaw) * LJoyMagnitude;
        double MaxPower = Math.max(Math.abs(anglePosX) + Math.abs(anglePosY) + Math.abs(currentRJX), 1);
        powerFL = (-anglePosY + anglePosX + currentRJX) / MaxPower;
        powerBL = (-anglePosY - anglePosX + currentRJX) / MaxPower;
        powerFR = (-anglePosY - anglePosX - currentRJX) / MaxPower;
        powerBR = (-anglePosY + anglePosX - currentRJX) / MaxPower;
        telemetry.addData("test: ", yaw);
    }

    @Override
    public void init() {
        // imu = hardwareMap.get(IMU.class, "imu");
        motorFR = hardwareMap.get(DcMotor.class, "FR");
        motorFL = hardwareMap.get(DcMotor.class, "FL");
        motorBR = hardwareMap.get(DcMotor.class, "BR");
        motorBL = hardwareMap.get(DcMotor.class, "BL");
        craneL = hardwareMap.get(DcMotor.class, "LCrane");
        craneR = hardwareMap.get(DcMotor.class, "RCrane");
        motorFL.setDirection(DcMotor.Direction.REVERSE);
        motorFR.setDirection(DcMotor.Direction.FORWARD);
        motorBL.setDirection(DcMotor.Direction.FORWARD);
        motorBR.setDirection(DcMotor.Direction.FORWARD);
        Arm = hardwareMap.get(DcMotor.class, "Arm");
        clawL = hardwareMap.get(Servo.class, "ClawL");
        clawR = hardwareMap.get(Servo.class, "ClawR");
        // clawR = hardwareMap.get(Servo.class, "ClawR");

        imu = hardwareMap.get(IMU.class, "imu");
        imuParams = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                )
        );
        imu.initialize(imuParams);
        imu.resetYaw();
    }

    @Override
    public void loop() {
        // Gamepad Variables
        RJoyX = gamepad1.right_stick_x;
        LJoyPos = new Vector2D(gamepad1.left_stick_x, gamepad1.left_stick_y);
        currentLJPos = new Vector2D(lerp(currentLJPos.getX(), LJoyPos.getX(), easeTime), lerp(currentLJPos.getY(), LJoyPos.getY(), easeTime));
        currentRJX = lerp(currentRJX, RJoyX, easeTime);
        LJoyPos = new Vector2D(currentLJPos.getY(), currentLJPos.getX());
        LJoyMagnitude = LJoyPos.getNorm();
        LJoyAngle = Math.atan2(currentLJPos.getX(), currentLJPos.getY());

        // Make sure LJoyPos isn't at (0, 0), throwing an error

        if (LJoyMagnitude > 0) {
            // LJoyAngle = Vector2D.angle(Vector2D.ZERO, LJoyPos);
        } else {
            LJoyAngle = 0;
        }

        // IMU Variables
        robotOrientation = imu.getRobotYawPitchRollAngles();
        double yaw = robotOrientation.getYaw(AngleUnit.RADIANS);
        double pitch = robotOrientation.getPitch(AngleUnit.RADIANS);
        double roll = robotOrientation.getRoll(AngleUnit.RADIANS);

        if (easingEnabled) {
            double MaxPower = Math.max(Math.abs(currentLJPos.getY()) + Math.abs(currentLJPos.getX()) + Math.abs(currentRJX), 1);
            powerFL = (-currentLJPos.getY() + currentLJPos.getX() + currentRJX) / MaxPower;
            powerBL = (-currentLJPos.getY() - currentLJPos.getX() + currentRJX) / MaxPower;
            powerFR = (-currentLJPos.getY() - currentLJPos.getX() - currentRJX) / MaxPower;
            powerBR = (-currentLJPos.getY() + currentLJPos.getX() - currentRJX) / MaxPower;
        }
        else {
            // Motor Power Equations
            double MaxPower = Math.max(Math.abs(LJoyPos.getY()) + Math.abs(LJoyPos.getX()) + Math.abs(RJoyX), 1);
            powerFL = (-LJoyPos.getY() + LJoyPos.getX() + RJoyX) / MaxPower;
            powerBL = (-LJoyPos.getY() - LJoyPos.getX() + RJoyX) / MaxPower;
            powerFR = (-LJoyPos.getY() - LJoyPos.getX() - RJoyX) / MaxPower;
            powerBR = (-LJoyPos.getY() + LJoyPos.getX() - RJoyX) / MaxPower;
        }

        // Angle Dependent Movement (Using Gyroscope)

        if (angleDrivingEnabled) {
            driveWithAngle(yaw);
        }

        // Recalibrate orientation
        if (gamepad1.dpad_left) {
            imu.resetYaw();
        }

        // Set all motors' power
        motorFL.setPower(powerFL);
        motorFR.setPower(powerFR); // Reversed Motor
        motorBL.setPower(powerBL);
        motorBR.setPower(powerBR); // Reversed Motor

        if (craneHeight < craneTopLimit) {
            craneL.setPower(gamepad1.left_trigger / craneSpeed);
            craneR.setPower(gamepad1.left_trigger / craneSpeed);
        }

        if (craneHeight > craneBottomLimit) {
            craneL.setPower(-gamepad1.right_trigger / craneSpeed);
            craneR.setPower(-gamepad1.right_trigger / craneSpeed);
        }

        if (gamepad1.dpad_up) {
            Arm.setPower(1 / armSpeed);
        } else if (gamepad1.dpad_down) {
            Arm.setPower(-1 / armSpeed);
        } else {
            Arm.setPower(0);
        }

        if (gamepad1.a) {
            clawL.setPosition(servoOpenPos+0.7);
            clawR.setPosition(servoClosePos);
        }

        if (gamepad1.b) {
            clawL.setPosition(servoClosePos);
            clawR.setPosition(servoOpenPos);
        }

        telemetry.addData("Front Motors: ","FL: %.3f, FR: %.3f",powerFL, powerFR);
        telemetry.addData("Rear Motors", "RL: %.3f, RR: %.3f", powerBL, powerBR);
        telemetry.addData("Vector", LJoyPos);
        telemetry.addData("Magnitude", LJoyMagnitude);
        telemetry.addData("Left Crane", "Raw: %.3f", gamepad1.right_trigger - gamepad1.left_trigger);
        telemetry.addData("Gyro: ", "Yaw: %.3f, Pitch: %.3f, Roll: %.3f", yaw, pitch, roll);
    }
}