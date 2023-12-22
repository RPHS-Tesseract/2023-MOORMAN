package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.config.TesseractConfig;
import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;

@TeleOp(name="Testing")
public class TestingTeleOp extends OpMode {
    // Motors
    public DcMotor wheelFL;
    public DcMotor wheelFR;
    public DcMotor wheelBR;
    public DcMotor wheelBL;
    public DcMotor rCrane;
    public DcMotor lCrane;
    public double powerFL;
    public double powerFR;
    public double powerBL;
    public double powerBR;
    Vector2D lJoyPos;
    Vector2D oldLeftJoyPos;
    double oldRightJoyX = 0;
    double oldTriggerR = 0;
    double oldTriggerL = 0;
    public double denominator = 0.0;
    public double easeTime = TesseractConfig.easeAmount;
    double lerp(double start, double end, double t){
        return start * (1-t) + end * t;
    }

    @Override
    public void init() {
        wheelFL = hardwareMap.get(DcMotor.class, "FL");
        wheelFR = hardwareMap.get(DcMotor.class, "FR");
        wheelBR = hardwareMap.get(DcMotor.class, "BR");
        wheelBL = hardwareMap.get(DcMotor.class, "BL");
        rCrane = hardwareMap.get(DcMotor.class, "RCrane");
        lCrane = hardwareMap.get(DcMotor.class, "LCrane");
        wheelFL.setDirection(DcMotor.Direction.REVERSE);
        wheelFR.setDirection(DcMotor.Direction.FORWARD);
        wheelBL.setDirection(DcMotor.Direction.FORWARD);
        wheelBR.setDirection(DcMotor.Direction.FORWARD);
    }

    @Override
    public void loop() {
        double leftJoyX = gamepad1.left_stick_x;
        double leftJoyY = gamepad1.left_stick_y;
        double rightJoyX = gamepad1.right_stick_x;
        double triggerR = gamepad1.right_trigger;
        double triggerL = gamepad1.left_trigger;

        denominator = Math.max(Math.abs(leftJoyX) + Math.abs(leftJoyY) + Math.abs(rightJoyX),1);
        powerFL = (-leftJoyY + leftJoyX + rightJoyX) / denominator;
        powerBL = (-leftJoyY - leftJoyX + rightJoyX) / denominator;
        powerFR = (-leftJoyY - leftJoyX - rightJoyX) / denominator;
        powerBR = (-leftJoyY + leftJoyX - rightJoyX) / denominator;

        lCrane.setPower(triggerR- triggerL);
        rCrane.setPower(triggerR - triggerL);


        wheelFL.setPower(powerFL);
        wheelFR.setPower(powerFR);
        wheelBR.setPower(powerBR);
        wheelBL.setPower(powerBL);

        //oldLeftJoyY = lerp(oldLeftJoyY, leftJoyY, easeTime);
        oldRightJoyX = lerp(oldRightJoyX, rightJoyX, easeTime);
        //oldLeftJoyX = lerp(oldLeftJoyX, leftJoyX, easeTime);
        oldTriggerR = lerp(oldTriggerR, triggerR , easeTime);
        oldTriggerL = lerp(oldTriggerL, triggerL, easeTime);
    }
}