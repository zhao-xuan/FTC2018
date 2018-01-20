/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="blueEasy2", group="Linear Opmode")
//@Disabled
public class blueOneArm extends LinearOpMode {

    // Declare OpMode members.

    private ElapsedTime runtime = new ElapsedTime();
    public DcMotor leftDrivef;
    public DcMotor rightDrivef;
    public DcMotor leftDriveb;
    public DcMotor rightDriveb;
    public DcMotor forklift;
    public Servo sideArm;
    public CRServo handLeft;
    public CRServo handRight;
    public ModernRoboticsI2cColorSensor colorSensor = null;



    //NormalizedColorSensor colorSensor;
    //View relativeLayout;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftDrivef  = hardwareMap.get(DcMotor.class, "left_drivef");
        rightDrivef = hardwareMap.get(DcMotor.class, "right_drivef");
        leftDriveb  = hardwareMap.get(DcMotor.class, "left_driveb");
        rightDriveb = hardwareMap.get(DcMotor.class, "right_driveb");
        forklift = hardwareMap. get(DcMotor.class, "forklift");
        sideArm = hardwareMap.get(Servo.class, "sideArm");
        handLeft = hardwareMap.get(CRServo.class, "handLeft");
        handRight = hardwareMap.get(CRServo.class, "handRight");

        colorSensor = hardwareMap.get(ModernRoboticsI2cColorSensor.class, "colorSensor");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftDrivef.setDirection(DcMotor.Direction.REVERSE);
        rightDrivef.setDirection(DcMotor.Direction.FORWARD);
        leftDriveb.setDirection(DcMotor.Direction.REVERSE);
        rightDriveb.setDirection(DcMotor.Direction.FORWARD);


        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        sideArm.setPosition(0.3);
        handLeft.setPower(0.6);
        handRight.setPower(-0.6);
        up(1);
        sleep(700);
        stopForklift();


        while (opModeIsActive()){
            colorSensor.enableLed(true);
            int color_num = colorSensor.readUnsignedByte(ModernRoboticsI2cColorSensor.Register.COLOR_NUMBER);
            telemetry.addData("Status", "Run Time: " + color_num );
            telemetry.update();
            if (color_num >= 1 && color_num <= 4){
                driveStraight(-1); //we are blue team, recognize the ball is blue, go back
                sleep(200);
                stopDriving();
                sleep(800);
                sideArm.setPosition(1);
                sleep(250);
                driveStraight(-1);
                sleep(550);
                stopDriving();
                sleep(1000);
                driveHorizontal(1);
                sleep(775);
                stopDriving();
                driveStraight(-1);
                sleep(195);
                stopDriving();
                //turn(-1);
                //sleep(50);
                //stopDriving();
                //up(-1);
                //sleep(650);
                //stopForklift();
                //sleep(500);
                //handLeft.setPosition(0.6);
                //handRight.setPosition(0.2);
                //driveStraight(1);
                //sleep(100);
                //stopDriving();
            }
            else if (color_num >= 9 && color_num <= 12){
                driveStraight(1); //we are blue team, recognize the ball is red, go forward
                sleep(70);
                stopDriving();
                sleep(300);
                sideArm.setPosition(1);
                sleep(300);
                driveStraight(-1);
                sleep(285);
                stopDriving();
                sleep(2000);
                driveStraight(-1);
                sleep(605);
                stopDriving();
                sleep(1000);
                driveHorizontal(1);
                sleep(750);
                stopDriving();
                driveStraight(-1);
                sleep(250);
                stopDriving();
                turn(1);
                sleep(40);
                stopDriving();

            }


            telemetry.addData("Status", "Run Time: " + color_num);
            telemetry.update();

        }


    }

    public void driveStraight (double power){
        leftDrivef.setPower(power);
        rightDrivef.setPower(power);
        leftDriveb.setPower(power);
        rightDriveb.setPower(power);
    }

    public void stopDriving (){
        driveStraight (0);
    }


    public void turn (double power) {
        //1 turn left, -1 turn right

        leftDrivef.setPower(-power);
        rightDrivef.setPower(power);
        leftDriveb.setPower(-power);
        rightDriveb.setPower(power);
    }
    public void driveHorizontal  (double power){
        //1 to left, -1 to right
        leftDrivef.setPower(-power);
        rightDrivef.setPower(power);
        leftDriveb.setPower(power);
        rightDriveb.setPower(-power);
    }
    public void up (double power)
    {forklift.setPower(power);}

    public void stopForklift()
    {up (0);}


}
