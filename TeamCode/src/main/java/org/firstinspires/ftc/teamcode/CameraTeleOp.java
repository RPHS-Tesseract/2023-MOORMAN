package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

@TeleOp(name="CameraTeleOp")
public class CameraTeleOp extends OpMode {
    WebcamName Webcam;

    @Override
    public void init(){
        Webcam = hardwareMap.get(WebcamName.class, "Webcam 1");
    }

    @Override
    public void loop(){

    }
}

