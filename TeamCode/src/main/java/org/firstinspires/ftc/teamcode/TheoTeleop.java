package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.robot.Robot;


@TeleOp(name="TeleOpppp", group="Iterative Opmode")


public class TheoTeleop extends OpMode{
    public RobotHardware rh = new RobotHardware (hardwareMap);

    @Override
    public void init() {
        rh.initialize();
    }
    @Override
    public void loop () {
        double y = -gamepad1.left_stick_y; // Remember, this is reversed!
        double x = gamepad1.left_stick_x * 1.0; // Counteract imperfect strafing
        double rx = gamepad1.right_stick_x;
        rh.drive(x, y, rx);
        rh.servo.setPower(gamepad2.left_stick_y);

        if (gamepad2.b){
            rh.servoopen = true;
        }
        else if (gamepad2.a){
            rh.servoopen = false;

        }
        if (rh.servoopen){
            rh.leftgripperServo.setPosition(rh.leftopenposition);
            rh.rightgripperServo.setPosition(rh.rightopenposition);
        }
        else {
            rh.leftgripperServo.setPosition(rh.leftcloseposition);
            rh.rightgripperServo.setPosition(rh.rightcloseposition);
        }

    }
}
