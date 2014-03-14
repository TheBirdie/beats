package edu.berkeley.path.beats.simulator;

/**
 * Created by gomes on 3/12/14.
 */
public abstract class LinkBehavior implements LinkBehaviorInterface {

    protected Link myLink;
    protected Scenario myScenario;
    protected double [] spaceSupply;        // [veh]	numEnsemble
    protected double [][] outflowDemand;    // [veh] 	numEnsemble x numVehTypes

    public LinkBehavior(Link link){
        this.myLink = link;
        this.myScenario = myLink.myScenario;
    }

    /////////////////////////////////////////////////////////////////////
    // protected API
    /////////////////////////////////////////////////////////////////////

    protected void reset(double [] initial_density) {
        int n1 = myScenario.getNumEnsemble();
        int n2 = myScenario.getNumVehicleTypes();
        outflowDemand 	= BeatsMath.zeros(n1,n2);
        spaceSupply 	= BeatsMath.zeros(n1);
        reset_density();
        set_density_in_veh(initial_density);
    }

    public boolean set_density_in_veh(double [] d){
        for(int e=0;e<myLink.myScenario.getNumEnsemble();e++)
            if(!set_density_in_veh(e,d))
                return false;
        return true;
    }

    public double getTotalDensityInVPMeter(int e) {
        return getTotalDensityInVeh(e)/myLink._length;
    }

    public double[] get_out_demand_in_veh(int e) {
        return outflowDemand[e];
    }

    public double get_space_supply_in_veh(int e) {
        return spaceSupply[e];
    }

    public double[] getDensityInVeh(int e) {
        int nVT = myLink.myScenario.getNumVehicleTypes();
        double [] d = new double[nVT];
        for(int v=0;v<nVT;v++)
            d[v] = get_density_in_veh(e, v);
        return d;

    }

    public double getTotalDensityInVeh(int e) {
        return BeatsMath.sum(getDensityInVeh(e));
    }

    public double computeTotalDelayInVeh(int e){
        double val=0d;
        for(int v=0;v<myLink.myScenario.getNumVehicleTypes();v++)
            val += compute_delay_in_veh(e, v);
        return val;
    }

}
