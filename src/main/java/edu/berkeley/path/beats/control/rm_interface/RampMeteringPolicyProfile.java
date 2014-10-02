package edu.berkeley.path.beats.control.rm_interface;

import edu.berkeley.path.beats.simulator.Link;

import java.util.LinkedList;
import java.util.List;

/**
 * Created with IntelliJ IDEA.
 * User: jdr
 * Date: 10/25/13
 * Time: 3:00 PM
 * To change this template use File | Settings | File Templates.
 */
public class RampMeteringPolicyProfile {
    public Link sensorLink;
    public List<Double> rampMeteringPolicy; // vehicles / (unit time), each element per simulation timestep

    public RampMeteringPolicyProfile() {
        rampMeteringPolicy = new LinkedList<Double>();
    }

    @Override
    public String toString() {
        String str = sensorLink.getId() + ": ";
        for (Double d : rampMeteringPolicy)
            str += d.toString() + ",";
        return str;
    }
}