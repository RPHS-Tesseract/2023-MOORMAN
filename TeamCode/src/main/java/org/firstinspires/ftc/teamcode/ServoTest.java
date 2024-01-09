package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

@TeleOp(name="ServoTest")
public class ServoTest extends OpMode {
    Servo servo;
    Servo servo1;

    @Override
    public void init(){
        servo1 = hardwareMap.get(Servo.class, "ClawL");
        servo = hardwareMap.get(Servo.class, "ClawR");
    }

    @Override
    public void loop(){
        servo.setPosition(gamepad1.left_stick_x);
        servo1.setPosition(gamepad1.left_stick_x);
    }
}

