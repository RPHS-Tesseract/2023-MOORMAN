package org.firstinspires.ftc.teamcode;

import java.lang.*;
import java.util.Arrays;
import java.util.Collections;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gyroscope;

import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;

@TeleOp(name="Tesseract")
public class TesseractTeleOp extends OpMode {
    // Motors
    public DcMotor frontLeftMotor;
    public DcMotor frontRightMotor;
    public DcMotor rearLeftMotor;
    public DcMotor rearRightMotor;
    public DcMotor leftCraneMotor;
    public DcMotor rightCraneMotor;
    public Gyroscope imu;
    public DcMotor motorTest;

    @Override
    public void init() {
        imu = hardwareMap.get(Gyroscope.class, "imu");
    }

    @Override
    public void loop() {
        // Gamepad Variables
        double LJoyX = gamepad1.left_stick_x;
        double LJoyY = gamepad1.left_stick_y;
        double RJoyX = gamepad1.right_stick_x;
        // Motor Power Equations
        double MaxPower = Math.max(Math.abs(LJoyY) + Math.abs(LJoyX) + Math.abs(RJoyX), 1);
        double FLPower = (LJoyY + LJoyX + RJoyX) / MaxPower;
        double BLPower = (LJoyY - LJoyX + RJoyX) / MaxPower;
        double FRPower = (LJoyY - LJoyX - RJoyX) / MaxPower;
        double BRPower = (LJoyY + LJoyX - RJoyX) / MaxPower;
        // Directional Movement Equations (not used right now)
        Vector2D LJoyVector = new Vector2D(LJoyX, LJoyY); // Convert Pos to Vector2D
        double LJoyM = LJoyVector.getNorm(); // Magnitude
        double LJoyA = 0;

        if (!(LJoyM == 0)) {
            LJoyA = Vector2D.angle(new Vector2D(0, 1) , LJoyVector); // Angle in Radians
        } else {
            LJoyA = 0;
        }

        frontLeftMotor.setPower(LJoyY); // Reversed Motor
        frontRightMotor.setPower(LJoyY);
        rearLeftMotor.setPower(LJoyY); // Reversed Motor
        rearRightMotor.setPower(LJoyY);

        telemetry.addData("Front Motors: ","FL: %.3f, FR: %.3f",LJoyY, LJoyY);
        telemetry.addData("Rear Motors", "RL: %.3f, RR: %.3f", LJoyY, LJoyY);
    }
}