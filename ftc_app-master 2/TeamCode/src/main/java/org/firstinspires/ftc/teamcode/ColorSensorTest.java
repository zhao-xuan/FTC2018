package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by 14187 on 12/6/2017.
 */


public class ColorSensorTest extends LinearOpMode {

    public ColorSensor color_sensor;
    public int r = color_sensor.red();
    public int g = color_sensor.green();
    public int b = color_sensor.blue();
    public int a = color_sensor.alpha();

    @Override
    public void runOpMode() {
        telemetry.addData("Test Mode state");
        telemetry.update();
    }

    //Tom is horsing around again!!!

}