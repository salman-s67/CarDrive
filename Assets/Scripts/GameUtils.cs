using UnityEngine;

public static class GameUtils
{
    public static float ConvertHPToKw(float hp)
    {
        return hp * GameConsts.hpToKw;
    }

    public static float ConvertRadPerSecToRPM(float radPerSec)
    {
        return (radPerSec * 60f / (2f * Mathf.PI));
    }

    public static float ConvertRadPerSecToRPMForHP()
    {
        // convert rad/s to RPM

        // omega (rad/s) * (1 rev/2*pi rad) * (60s/1 min) => rev/min = omega (RPM) * 2pi / 60

        // P = Fv = FR*omega (rad/s) = torque * omega (rad/s) = torque * omega * 2pi/60 (RPM)
        // HP = P/746 = (torque * omega * 2pi/60)/746 = torque * omega (RPM) / 7127

        return (2 * Mathf.PI / 60f) / GameConsts.hpToKw;
    }

    public static float ConvertVelocityToKmhFromMs(float velocityMs)
    {
        // convert velocity from m/s to km/h

        return velocityMs * GameConsts.msToKmph;
    }

    public static float ConvertVelocityToMsFromKmh(float velocityKmh)
    {
        // convert velocity from km/h to m/s

        return velocityKmh * GameConsts.kmphToMs;
    }

    public static float RemapValue(float oldRangeMin, float oldRangeMax, float newRangeMin, float newRangeMax, float valToRemap)
    {
        // map oldRange from 0-1
        // OldRange = (OldMax - OldMin)
        // NewRange = (NewMax - NewMin)
        // NewValue = (((OldValue - OldMin) * NewRange) / OldRange) + NewMin

        float oldRange = oldRangeMax - oldRangeMin;
        float newRange = newRangeMax - newRangeMin;
        float newValue = (((valToRemap - oldRangeMin) * newRange) / oldRange) + newRangeMin;

        return newValue;
    }
}
