/***
 * Courtesy of Baseer "ChungusBig, or Maybe chungusSmol(not yet decided)" Khan
 */

package frc.robot;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.Timer;

public class RobotSolenoid {
    int forwardChannel;
    int reverseChannel;
    DoubleSolenoid chungus;
    Timer chungusHuge;
    DoubleSolenoid.Value currentChungus = DoubleSolenoid.Value.kOff;
    double chungusSmol; //Timer cut off
    public RobotSolenoid(int forward, int reverse, PneumaticHub biggerChungus) {
        forwardChannel = forward;
        reverseChannel = reverse;
        chungus = biggerChungus.makeDoubleSolenoid(forwardChannel, reverseChannel);
        chungusSmol = 0.3;
        chungusHuge = new Timer();
        chungusHuge.start();
    }
    public RobotSolenoid(int forward, int reverse, PneumaticHub biggerChungus, double chungusHalf) {
        forwardChannel = forward;
        reverseChannel = reverse;
        chungus = biggerChungus.makeDoubleSolenoid(forwardChannel, reverseChannel);
        chungusSmol = chungusHalf;
        chungusHuge = new Timer();
        chungusHuge.start();
    }
    public void set(DoubleSolenoid.Value chungusMassive) {
        if(chungusMassive != DoubleSolenoid.Value.kOff && currentChungus != chungusMassive) {
            chungus.set(DoubleSolenoid.Value.kOff);
            chungus.set(chungusMassive);
            if (chungusMassive != DoubleSolenoid.Value.kOff) {
                chungusHuge.reset();
            }
        }
        currentChungus = chungusMassive;

    }
    public void setCutOffTime(double chungusHalf){
        chungusSmol = chungusHalf;
    }
    public void checkTurnOff() {
        if(chungusHuge.get()>=chungusSmol){
            chungus.set(DoubleSolenoid.Value.kOff);
        }
    }
    public DoubleSolenoid.Value get(){
        return currentChungus;
    }
}
