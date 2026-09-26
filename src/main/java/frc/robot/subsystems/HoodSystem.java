package frc.robot.subsystems;

import frc.robot.Constants.LinearServoConstants;
import com.revrobotics.ResetMode;
//import com.revrobotics.config.BaseConfig;
//import com.revrobotics.servohub.config.ServoHubParameter;
import com.revrobotics.servohub.ServoChannel;
import com.revrobotics.servohub.ServoChannel.ChannelId;
import com.revrobotics.servohub.ServoHub;
import com.revrobotics.servohub.config.ServoHubConfig;
import com.revrobotics.servohub.config.ServoChannelConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class HoodSystem extends SubsystemBase {
    ServoHubConfig config = new ServoHubConfig();
    private final ServoHub m_servoHub;
    private final ServoChannel linearServoLeft;
    private final ServoChannel linearServoRight;
    double m_angle = 0;
    

    public HoodSystem() {
        m_servoHub = new ServoHub(LinearServoConstants.kServoHubId);
        m_servoHub.setBankPulsePeriod(ServoHub.Bank.kBank0_2, LinearServoConstants.kPulsePeriod);
        
        linearServoLeft = m_servoHub.getServoChannel(ChannelId.kChannelId1);
        linearServoRight = m_servoHub.getServoChannel(ChannelId.kChannelId2);


        config
            .channel0.pulseRange(LinearServoConstants.kMinPulseWidth, LinearServoConstants.kMidPulseWidth, LinearServoConstants.kMaxPulseWidth)
            .disableBehavior(ServoChannelConfig.BehaviorWhenDisabled.kSupplyPower);

        // Persist parameters and reset any not explicitly set above to
        // their defaults.
        m_servoHub.configure(config, ResetMode.kResetSafeParameters);
        retractServo();
    }

    private static int angleToServoValue(double angle)
    {
        // Desired mapping:
        //  - 70 degrees -> fully retracted -> use kMinExtensionPulseWidth
        //  - 25 degrees -> fully extended  -> use kMaxExtensionPulseWidth
        // We'll interpolate linearly between these two angles.

        // Define the angle endpoints from HoodConstants (these match previous motor-based mapping)
        final double angleRetracted = frc.robot.Constants.HoodConstants.kRetractedAngle; // 70.0
        final double angleExtended = frc.robot.Constants.HoodConstants.kHoodAngleB;     // 25.0

        // If angle is outside the [extended, retracted] range, clamp it
        double clamped = Math.max(Math.min(angle, angleRetracted), angleExtended);

        // Compute interpolation factor t in [0,1]
        // t = 0 -> at extended angle (25 deg) -> full extension (max pulse)
        // t = 1 -> at retracted angle (70 deg) -> full retraction (min pulse)
        double t = (clamped - angleExtended) / (angleRetracted - angleExtended);

        // Interpolate pulse width between max extension (at t=0) and min extension (at t=1)
        double pulseAtT = LinearServoConstants.kMaxExtensionPulseWidth * (1.0 - t)
                        + LinearServoConstants.kMinExtensionPulseWidth * t;

        // Final safety clamp to overall pulse bounds
        double value = Math.max(LinearServoConstants.kMinPulseWidth,
                        Math.min(LinearServoConstants.kMaxPulseWidth, pulseAtT));

        return (int)Math.round(value);
    }

    public void setAngle(double angle)
    {   
        if(m_angle == angle)
            return;
        m_angle = angle;
        setPosition(angleToServoValue(angle));
    }
    
    /**
     * Return the last commanded hood angle (degrees).
     */
    public double getCommandedAngleDeg() {
        return m_angle;
    }


    public void setPosition(int pulseWidth) {
        // Convert position (0.0 to 1.0) to pulse width (500 to 2500 microseconds)
        linearServoLeft.setPowered(true);
        linearServoLeft.setEnabled(true);
        linearServoLeft.setPulseWidth(pulseWidth);

        linearServoRight.setPowered(true);
        linearServoRight.setEnabled(true);
        linearServoRight.setPulseWidth(pulseWidth);
    }

    public void extendServo() {
        linearServoLeft.setPowered(true);
        linearServoLeft.setEnabled(true);
        linearServoLeft.setPulseWidth(LinearServoConstants.kMaxExtensionPulseWidth);

        linearServoRight.setPowered(true);
        linearServoRight.setEnabled(true);
        linearServoRight.setPulseWidth(LinearServoConstants.kMaxExtensionPulseWidth);
    }

    public void retractServo() {
        linearServoLeft.setPowered(true);
        linearServoLeft.setEnabled(true);
        linearServoLeft.setPulseWidth(LinearServoConstants.kMinExtensionPulseWidth);

        linearServoRight.setPowered(true);
        linearServoRight.setEnabled(true);
        linearServoRight.setPulseWidth(LinearServoConstants.kMinExtensionPulseWidth);
    }

}
    
