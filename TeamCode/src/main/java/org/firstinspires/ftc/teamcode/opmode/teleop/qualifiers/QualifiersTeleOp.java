package org.firstinspires.ftc.teamcode.opmode.teleop.qualifiers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp
public class QualifiersTeleOp extends LinearOpMode {

    public DcMotor [] motors;
    public double[] wheelPowers;
    public Servo [] servos;

    @Override
    public void runOpMode() throws InterruptedException{
        // define motors //
        DcMotor leftFront = hardwareMap.dcMotor.get("leftFront");
        DcMotor rightFont = hardwareMap.dcMotor.get("rightFront");
        DcMotor leftBack = hardwareMap.dcMotor.get("leftBack");
        DcMotor rightBack = hardwareMap.dcMotor.get("rightBack");
        DcMotor intake = hardwareMap.dcMotor.get("intake");
        DcMotor leftSlide = hardwareMap.dcMotor.get("leftSlide");
        DcMotor rightSlide = hardwareMap.dcMotor.get("rightSlide");

        // set motor directions //
        leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFont.setDirection(DcMotorSimple.Direction.FORWARD);
        leftBack.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBack.setDirection(DcMotorSimple.Direction.FORWARD);
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        leftSlide.setDirection(DcMotorSimple.Direction.FORWARD);
        rightSlide.setDirection(DcMotorSimple.Direction.FORWARD);

        // list of all motors //
        motors = new DcMotor[]{leftFront, rightFont, leftBack, rightBack, intake, leftSlide, rightSlide};


        // reset motor and run with encoder //
        for (DcMotor motor : motors){
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        Servo outake = hardwareMap.servo.get("outake");

        servos = new Servo[]{outake};



        IMU imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(parameters);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()){
            wheelPowers = getWheelPowers(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
            for (int i = 0; i < 5; i++){
                motors[i].setPower(wheelPowers[i]);
            }

        }
    }

    public double[] getWheelPowers(double x, double y, double rx, double botHeading){
        double rotX = x * Math.cos(botHeading) - y * Math.sin(botHeading);
        double rotY = x * Math.sin(botHeading) + y * Math.cos(botHeading);
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double leftFrontPower = (rotY + rotX + rx) / denominator;
        double leftBackPower = (rotY - rotX + rx) / denominator;
        double rightFrontPower = (rotY - rotX - rx) / denominator;
        double rightBackPower = (rotY + rotX - rx) / denominator;
        double[] wheelPowers = new double[]{leftFrontPower, leftBackPower, rightFrontPower, rightBackPower};

        return (wheelPowers);
    }





}
