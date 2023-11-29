package org.firstinspires.ftc.teamcode;

import java.lang.*;
import java.util.Arrays;
import java.util.Collections;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gyroscope;

import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;

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
    public DcMotor leftCraneMotor;
    public DcMotor rightCraneMotor;
    public Gyroscope imu;
    public DcMotor motorTest;
    public double easeTime = 0.01; // Alpha for Joystick Lerp
    public boolean easingEnabled = true; // Disable if causes problems
    // setup joystick variables
    double LJoyX = 0;
    double LJoyY = 0;
    double RJoyX = 0;
    double currentLJX = LJoyX;
    double currentLJY = LJoyY;
    double currentRJX = RJoyX;

    @Override
    public void init() {
        //imu = hardwareMap.get(Gyroscope.class, "imu");
        motorFR = hardwareMap.get(DcMotor.class, "FR");
        motorFL = hardwareMap.get(DcMotor.class, "FL");
        motorBR = hardwareMap.get(DcMotor.class, "BR");
        motorBL = hardwareMap.get(DcMotor.class, "BL");
        motorFL.setDirection(DcMotor.Direction.REVERSE);
        motorFR.setDirection(DcMotor.Direction.FORWARD);
        motorBL.setDirection(DcMotor.Direction.FORWARD);
        motorBR.setDirection(DcMotor.Direction.FORWARD);
    }

    @Override
    public void loop() {
        // Gamepad Variables
        LJoyX = gamepad1.left_stick_x;
        LJoyY = gamepad1.left_stick_y;
        RJoyX = gamepad1.right_stick_x;
        currentLJY = lerp(currentLJY, LJoyY, easeTime);
        currentLJX = lerp(currentLJX, LJoyX, easeTime);
        currentRJX = lerp(currentRJX, RJoyX, easeTime);

        double powerFL = 0;
        double powerBL = 0;
        double powerFR = 0;
        double powerBR = 0;

        if (easingEnabled) {
            double MaxPower = Math.max(Math.abs(currentLJY) + Math.abs(currentLJX) + Math.abs(currentRJX), 1);
            powerFL = (-currentLJY + currentLJX + currentRJX) / MaxPower;
            powerBL = (-currentLJY - currentLJX + currentRJX) / MaxPower;
            powerFR = (-currentLJY - currentLJX - currentRJX) / MaxPower;
            powerBR = (-currentLJY + currentLJX - currentRJX) / MaxPower;
        }
        else {
            // Motor Power Equations
            double MaxPower = Math.max(Math.abs(LJoyY) + Math.abs(LJoyX) + Math.abs(RJoyX), 1);
            powerFL = (-LJoyY + LJoyX + RJoyX) / MaxPower;
            powerBL = (-LJoyY - LJoyX + RJoyX) / MaxPower;
            powerFR = (-LJoyY - LJoyX - RJoyX) / MaxPower;
            powerBR = (-LJoyY + LJoyX - RJoyX) / MaxPower;
        }

        // Directional Movement Equations (not used right now)
        Vector2D LJoyVector = new Vector2D(LJoyX, LJoyY); // Convert Pos to Vector2D
        double LJoyM = LJoyVector.getNorm(); // Magnitude
        double LJoyA = 0;

        if (!(LJoyM == 0)) {
            LJoyA = Vector2D.angle(new Vector2D(0, 1) , LJoyVector); // Angle in Radians
        } else {
            LJoyA = 0;
        }

        motorFL.setPower(powerFL);
        motorFR.setPower(powerFR); // Reversed Motor
        motorBL.setPower(powerBL);
        motorBR.setPower(powerBR); // Reversed Motor

        telemetry.addData("Front Motors: ","FL: %.3f, FR: %.3f",LJoyY, LJoyY);
        telemetry.addData("Rear Motors", "RL: %.3f, RR: %.3f", LJoyY, LJoyY);
        telemetry.addData("Left Joystick", "Raw: %.3f, Mod: %.3f", LJoyY, currentLJY);
    }
}