package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.robotcontroller.external.samples.RobotHardware;
@TeleOp(name="DriveControl", group="TeleOp")

 public class DriveControl extends OpMode {

    // define robot
    RobotHardware robot = new RobotHardware();


    //run once on init()
    @Override
    public void init() {
        robot.init(hardwareMap);
        telemetry.addData("STATUS", "Initialized");
    }

    // loop on init()
    @Override
    public void init_loop() {
    }

    // code that runs once on start()
    @Override
    public void start() {
    }

    // loops on start(), main opMode
    @Override
    public void loop() {
        //defines the variables used to calculate motor power
        double Y = gamepad1.right_stick_y;
        double X = gamepad1.right_stick_x;
        double R = gamepad1.left_stick_x;


        //Defines the power for each Dcmotors and Servos
        robot.frontLeftMotor.setPower(Y + X - R);
        robot.backLeftMotor.setPower(Y - X - R);
        robot.frontRightMotor.setPower(-Y + X - R);
        robot.backRightMotor.setPower(-Y - X - R);

    }

    }



