
package org.firstinspires.ftc.teamcode;



import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;
import com.arcrobotics.ftclib.util.InterpLUT;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.List;

public class bass {


    Limelight3A limelight;
    LLResult result = limelight.getLatestResult();
    boolean limeValid = result.isValid();
    double Tx=0,Ty=0;
    boolean limeLocked=false;
    double turretPower=0;
    PIDController turretPID;
    double Tx_offset=0;
    int turretCwlim=-250;
    int turretCcwlim=250;



    public void turretturn(boolean outtake , int target, int turretPos){

//        int turretPos=turretSpin.getCurrentPosition();
        int id;

        if(limeValid) {
            limeLocked = true;
            Tx = result.getTx();
            Ty = result.getTy();
        }

        if (outtake)  {

            if(limeValid) {
                turretPower = turretPID.calculate(Tx, Tx_offset);
                if (turretPos > turretCcwlim- 10 &&  turretPower  > 0) {
                    turretPower= -0.3;
                    target = 0;
                  limeLocked = false;
                }
                if (turretPos < (turretCwlim + 10) && turretPower< 0) {
                    turretPower= 0.3;
                    target = 0;
                    limeLocked = false;
                }


                return;
            }

             if (limeLocked) {
                turretPower=0;
                Tx=10;
                return;
            }



        }


        // non limalinment, contrl by encoder/angle

        //      if(Math.abs(turretPos-target)<12) {pid_power=0; if (target!=0) turettarget=0;}// if reach the limit, turn to the middle

        turretPower = turretPID.calculate( turretPos*0.35,  target*0.35);








    }

}

//package org.firstinspires.ftc.teamcode;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
//import com.arcrobotics.ftclib.controller.PIDController;
//import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;
//import com.arcrobotics.ftclib.util.InterpLUT;
//
//import com.qualcomm.hardware.limelightvision.LLResult;
//import com.qualcomm.hardware.limelightvision.LLResultTypes;
//import com.qualcomm.hardware.limelightvision.Limelight3A;
//import com.qualcomm.hardware.lynx.LynxModule;
//import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.IMU;
//import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.robotcore.external.Telemetry;
//
//import java.util.List;
//
//public class bass {
//
//
//
//    public void limeAutoAimer(boolean lim , int target, int turretPos, Limelight3A limelight){
//
////        int turretPos=turretSpin.getCurrentPosition();
//        int id;
//
//        if (lim)  {
//
//            LLResult result = limelight.getLatestResult();
//            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
//            for (LLResultTypes.FiducialResult fiducial : fiducials) {
//                id = fiducial.getFiducialId(); // The ID number of the fiducial
//            }
//
//            limeValid = result.isValid();
//
//            if (limeValid && ((id == target_id)) ) {
//
//                //telemetry.addLine("detected");
//                //   flag[autoaim]=true;
//                // flag[limedetect]=true;
//                Tx = result.getTx();
//                Ty = result.getTy();
//                if (red && Ty < 10 && Ty> -10.5) Tx_offset = -1.5;
//
//                if (!red && Ty < 10 && Ty> -10.5) Tx_offset = 1.5;
//
//                else if (Ty < -10.5 && red) Tx_offset = -3;
//
//                else if (Ty < -10.5 && !red) Tx_offset = 2.5;
//                flag[detected]=true;
//
//                //turettarget=turretPos;
//                pid_power = turretPID.calculate(Tx, Tx_offset);
//                if(turretPos>turretcounterclock-10&& pid_power>0)  {pid_power=-0.2;turettarget=0;flag[detected]=false;}
//                if(turretPos<(turrretclock+10)&& pid_power<0)  {pid_power=0.2;turettarget=0;flag[detected]=false;}
//                turretSpin.setPower(pid_power);
//
//                return;
//
//
//            }
//            else if (flag[detected]) {
//                turretSpin.setPower(0);
//                Tx=10;
//                return;
//            }
//
//
//
//        }
//        // non limalinment, contrl by encoder/angle
//
//        //      if(Math.abs(turretPos-target)<12) {pid_power=0; if (target!=0) turettarget=0;}// if reach the limit, turn to the middle
//
//        double turnpower = turretPID.calculate( turretPos*0.35,  target*0.35);
//
//        pid_power = Range.clip(turnpower, -MAX_TURN_POWER, MAX_TURN_POWER);
//
//        if(turretPos>turretcounterclock-10&& pid_power>0)  {pid_power=-0.2;turettarget=0;}
//        if(turretPos<(turrretclock+10)&& pid_power<0)  {pid_power=0.2;turettarget=0;}
//
//        turretSpin.setPower(pid_power);
//
//
//    }
//
//}

