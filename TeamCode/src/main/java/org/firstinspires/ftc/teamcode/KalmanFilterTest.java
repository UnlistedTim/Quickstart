package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.Arrays;
import java.util.List;



@TeleOp
@Config
public class KalmanFilterTest extends LinearOpMode {




    public DcMotorEx turretSpin;


    Pose2D pose;

    public GoBildaPinpointDriver Pinpoint;


    boolean limeValid = false;

    int id = 1;


    double Tx = 0;

    double Ty = 0;


    private DcMotorEx leftFront, rightFront, leftBack, rightBack;

    private double dx, dy, dtheta, lastX, lastY, lastTheta;


    private Limelight3A Limelight;


    @Override

    public void runOpMode() {
//        Intake = hardwareMap.get(DcMotorEx.class, "Intake");


        leftFront = hardwareMap.get(DcMotorEx.class,"leftFront");
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");

        Limelight = hardwareMap.get(Limelight3A.class, "Limelight");
//        turretSpin = hardwareMap.get(DcMotorEx.class, "turretSpin");

        Pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "Pinpoint");

        configurePinpoint();

        Kalman1D kfX;
        Kalman1D kfY;
        Kalman1D kfTheta;

        kfX = new Kalman1D(0,0.02,2.0);
        kfY = new Kalman1D(0,0.02,2.0);
        kfTheta = new Kalman1D(0,0.01,1.0);




        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());//todo




        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        Limelight.pipelineSwitch(3);

        double targetGoalX = 140.86;
        double targetGoalY = 140.86;

        double startX = (93.83-7.5);
        double startY = 11.17;
        double startHeading=1.5*Math.PI;



        waitForStart();

        Pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, startX, startY, AngleUnit.RADIANS, startHeading));
//
        Limelight.start();


        while (opModeIsActive()) { //Main While loop

            Pinpoint.update();
            pose = Pinpoint.getPosition();
            dx = pose.getX(DistanceUnit.INCH) - lastX;
            dy = pose.getY(DistanceUnit.INCH) - lastY;
            dtheta = pose.getHeading(AngleUnit.RADIANS) - lastTheta;

            kfX.predict(dx);
            kfY.predict(dx);
            kfTheta.predict(dtheta);


            LLResult result = Limelight.getLatestResult();

            limeValid = result.isValid();

            if (limeValid){



            }


            lastX = pose.getX(DistanceUnit.INCH);
            lastY = pose.getY(DistanceUnit.INCH);
            lastTheta = pose.getHeading(AngleUnit.RADIANS);






        }
    }

    public class Kalman1D {
        private double estimate;
        private double covariance;
        private final double Q;
        private final double R;

        public Kalman1D(double initialEstimate, double Q, double R) {
            this.estimate = initialEstimate;
            this.covariance = 1;
            this.Q = Q;
            this.R = R;
        }

        public void predict(double delta) {
            estimate += delta;
            covariance += Q;
        }

        public void update(double measurement) {
            double K = covariance / (covariance + R);
            estimate += K * (measurement - estimate);
            covariance *= (1 - K);
        }

        public double getEstimate() {
            return estimate;
        }
    }



    public void configurePinpoint(){
        /*
         *  Set the odometry pod positions relative to the point that you want the position to be measured from.
         *
         *  The X pod offset refers to how far sideways from the tracking point the X (forward) odometry pod is.
         *  Left of the center is a positive number, right of center is a negative number.
         *
         *  The Y pod offset refers to how far forwards from the tracking point the Y (strafe) odometry pod is.
         *  Forward of center is a positive number, backwards is a negative number.
         */
        Pinpoint.setOffsets(3.15, -4.9, DistanceUnit.INCH); //these are tuned for 3110-0002-0001 Product Insight #1

        /*
         * Set the kind of pods used by your robot. If you're using goBILDA odometry pods, select either
         * the goBILDA_SWINGARM_POD, or the goBILDA_4_BAR_POD.
         * If you're using another kind of odometry pod, uncomment setEncoderResolution and input the
         * number of ticks per unit of your odometry pod.  For example:
         *     pinpoint.setEncoderResolution(13.26291192, DistanceUnit.MM);
         */
        Pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

        /*
         * Set the direction that each of the two odometry pods count. The X (forward) pod should
         * increase when you move the robot forward. And the Y (strafe) pod should increase when
         * you move the robot to the left.
         */
        Pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD);

        /*
         * Before running the robot, recalibrate the IMU. This needs to happen when the robot is stationary
         * The IMU will automatically calibrate when first powered on, but recalibrating before running
         * the robot is a good idea to ensure that the calibration is "good".
         * resetPosAndIMU will reset the position to 0,0,0 and also recalibrate the IMU.
         * This is recommended before you run your autonomous, as a bad initial calibration can cause
         * an incorrect starting value for x, y, and heading.
         */
        Pinpoint.resetPosAndIMU();
    }

    public double calcDist(double x0, double y0, double x1, double y1){
        return Math.sqrt (Math.pow(x1 - x0, 2) + Math.pow(y1 - y0, 2));

    }





}











