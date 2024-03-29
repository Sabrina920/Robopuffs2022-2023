package org.firstinspires.ftc.teamcode;

import static com.sun.tools.doclint.HtmlTag.I;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.hardware.bosch.BNO055IMU;

@TeleOp(name="TheleOp", group="Iterative Opmode")


public class TheoTeleop extends LinearOpMode {

    
   // public CRServo rh.scissorServoCR;


    @Override
    public void runOpMode() throws InterruptedException {
       RobotHardware rh = new RobotHardware();
        rh.initialize(hardwareMap);
        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y; // Remember, this is reversed!
            double x = gamepad1.left_stick_x * 1.0; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;
            rh.drive(x, y, rx);

            telemetry.addData("servo pos:", rh.scissorServo.getPosition());

            if(gamepad2.x){
                rh.scissorServo.setPosition(0.7);
            }
            else if(gamepad2.y){
                rh.scissorServo.setPosition(0.4);
            }
            if(gamepad2.dpad_up){
                rh.scissorServo.setPosition(rh.scissorServo.getPosition()+.05);
            }
            else if (gamepad2.dpad_down){
                rh.scissorServo.setPosition(rh.scissorServo.getPosition()-.05);
            }
            else{
                rh.scissorServo.setPosition(rh.scissorServo.getPosition());
            }
            if (gamepad2.b){
                rh.servoopen = true;
            }
            else if (gamepad2.a){
                rh.servoopen = false;

            }
            if (rh.servoopen){
                rh.gripperServo.setPosition(rh.openposition);
            }
            else {
                rh.gripperServo.setPosition(rh.closeposition);
            }



 }

            telemetry.update();
        }
    }


