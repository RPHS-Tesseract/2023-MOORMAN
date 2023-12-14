package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.config.TesseractConfig;

@TeleOp(name="Testing")
public class TestingTeleOp extends OpMode {
    // Motors
    public DcMotor craneL;
    public DcMotor craneR;
    public DcMotor wheelFL;
    public DcMotor wheelFR;
    public DcMotor wheelBR;
    public DcMotor wheelBL;
    public double powerFL;
    public double powerFR;
    public double powerBL;
    public double powerBR;
    public double leftJoyX = gamepad1.left_stick_x;
    public double leftJoyY = gamepad1.left_stick_y;
    public double rightJoyX = gamepad1.right_stick_x;
    public double denominator = 0.0;

    @Override
    public void init() {
        craneL = hardwareMap.get(DcMotor.class, "LCrane");
        wheelFL = hardwareMap.get(DcMotor.class, "FL");
        wheelFR = hardwareMap.get(DcMotor.class, "FR");
        wheelBR = hardwareMap.get(DcMotor.class, "BR");
        wheelBL = hardwareMap.get(DcMotor.class, "BL");
        wheelBL.setDirection(DcMotor.Direction.REVERSE);
        wheelBR.setDirection(DcMotor.Direction.REVERSE);
    }

    @Override
    public void loop() {
        denominator = Math.max(Math.abs(leftJoyX) + Math.abs(leftJoyY) + Math.abs(rightJoyX),1);
        powerFL = (leftJoyY + leftJoyX + rightJoyX) / denominator;
        powerBL = (leftJoyY - rightJoyX + rightJoyX) / denominator;
        powerFR = (leftJoyY - leftJoyX - rightJoyX) / denominator;
        powerBR = (leftJoyY + leftJoyX - rightJoyX) / denominator;

        wheelFL.setPower(powerFL);
        wheelFR.setPower(powerFR);
        wheelBR.setPower(powerBR);
        wheelBL.setPower(powerBL);
        //craneL.setPower(gamepad1.left_stick_y);
    }
}