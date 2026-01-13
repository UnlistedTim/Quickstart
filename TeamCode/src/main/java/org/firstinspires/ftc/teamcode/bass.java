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
