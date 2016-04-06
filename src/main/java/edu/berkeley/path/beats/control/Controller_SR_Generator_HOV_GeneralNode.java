package edu.berkeley.path.beats.control;

import edu.berkeley.path.beats.simulator.Clock;
import edu.berkeley.path.beats.simulator.Controller;
import edu.berkeley.path.beats.simulator.Link;
import edu.berkeley.path.beats.simulator.Scenario;
import edu.berkeley.path.beats.simulator.nodeBeahavior.Node_FlowSolver;
import edu.berkeley.path.beats.simulator.utils.BeatsMath;

/**
 * Created by matt on 2/1/16.
 */
public class Controller_SR_Generator_HOV_GeneralNode extends Controller_SR_Generator_new {

	protected boolean[] variable_vtype;

	/////////////////////////////////////////////////////////////////////
	// Construction
	/////////////////////////////////////////////////////////////////////

	public Controller_SR_Generator_HOV_GeneralNode(Scenario myScenario, edu.berkeley.path.beats.jaxb.Controller c) {
		super(myScenario,c);
		variable_vtype = new boolean[myScenario.get.numVehicleTypes()];
		for(int v=0;v<myScenario.get.numVehicleTypes();v++) {
			if( myScenario.get.vehicleTypeNames()[v].compareToIgnoreCase("hov") == 0 ||
					myScenario.get.vehicleTypeNames()[v].compareToIgnoreCase("sov") == 0 ||
					myScenario.get.vehicleTypeNames()[v].compareToIgnoreCase("lov") == 0) {
				variable_vtype[v] = true;
			}
			else {
				variable_vtype[v] = false;
			}
		}
	}

	@Override
	protected NodeData createNodeData(Link link, String demandString, double knob, double dpdt, double start_time) {
		return new NodeData(this, link, demandString, knob, dpdt, start_time, myScenario);
	}

	class NodeData extends Controller_SR_Generator_new.NodeData {

		int gp_in, hov_in, offramp_out;

		public NodeData(Controller parent, Link profileLink, String demandStr, Double knob, Double dpdt, Double start_time, Scenario scenario) {
			super(parent, profileLink, demandStr, knob, dpdt, start_time, scenario);

			for (int i = 0; i < is_feed.size(); i++) {
				if (is_feed.get(i) && myNode.getInput_link()[i].isHov())
					hov_in = i;
				else if (is_feed.get(i) && myNode.getInput_link()[i].isFreeway())
					gp_in = i;
			}
			offramp_out = myNode.getOutputLinkIndex(profileLink.getId());
  		}

  		@Override
  		public void update(Clock clock) {
			int i,j;
			int e = 0;

			double bHigh, bLow, bTest;
			Node_FlowSolver.IOFlow flow;

			// make node sample its split ratio
			myNode.sample_split_ratio_profile();

			if(measured_flow_profile_veh.sample(false,clock))
				measured_flow_veh = measured_flow_profile_veh.getCurrentSample()*knob; // hat{f}_222^in

			if(BeatsMath.equals(measured_flow_veh, 0d)){
				beta = 0d;
				return;
			}

			// find demands from GP and HOV lanes
			double S[] = new double[myNode.getnIn()];
			for (i = 0; i < myNode.getnIn(); i++) {
				S[i] = myNode.getInput_link()[i].get_total_out_demand_in_veh(e);
			}

			if( BeatsMath.sum(S) <= measured_flow_veh ) { // insufficient demand
				beta = 1d;
				return;
			}

			bHigh = 1d;
			bLow = measured_flow_veh / BeatsMath.sum(S);

			// begin bisection method
			Double[][][] splitratio = myNode.getSplitRatio().clone();
			bTest = bLow;
			boolean solved = false;
			while (!solved) {
				for (int c=0; c<myScenario.get.numVehicleTypes(); c++)
				{
					if(variable_vtype[c]) {
						splitratio[hov_in][offramp_out][c] = bTest;// beta we are searching for
						splitratio[gp_in][offramp_out][c] = bTest;

						// remaining split
						for (i = 0; i < myNode.getnIn(); i++) {
							for (j = 0; j < myNode.getnOut(); j++) {
								if (!((i == hov_in || i == gp_in) && j == offramp_out)) {
									splitratio[i][j][c] = (1 - bTest) * myNode.getSplitRatio(i, j, c);
								}
							}
						}
					}
				}
				// invoke the SRsolver to fill in any undefined parts
				Double[][][] splitratio_filled = myNode.node_behavior.sr_solver.computeAppliedSplitRatio(splitratio, e);

				flow = myNode.node_behavior.flow_solver.computeLinkFlows(splitratio_filled, e);
				if (BeatsMath.equals( BeatsMath.sum(flow.getOut(offramp_out)),measured_flow_veh, measured_flow_veh*.01 )) {
					beta = bTest;
					solved = true;
				}
				else if (BeatsMath.equals(bLow, bHigh, (bLow+bHigh)/2 * .001)) {
					beta = (bLow+bHigh)/2;
					solved = true;
				}
				else if (BeatsMath.sum(flow.getOut(offramp_out)) - measured_flow_veh < 0) {
					bLow = (bLow + bHigh) / 2;
					bTest = (bLow + bHigh) / 2;
				}
				else {
					bHigh = (bLow + bHigh) / 2;
					bTest = (bLow + bHigh) / 2;
				}
			}
		}

		@Override
		public void deploy(double current_time_in_seconds){
			int i,j,c;
			double current_split;
			for(i=0;i<myNode.nIn;i++){
				Link inlink = myNode.input_link[i];
				if( inlink.isHov() || inlink.isFreeway()){
					for(j=0;j<myNode.nOut;j++){
						Link outlink = myNode.output_link[j];

						if(j==offramp_out) { // the offramp gets split ratio of beta
							for(c=0;c<myScenario.get.numVehicleTypes();c++) {
								if(variable_vtype[c]) {
									cms.set_split(inlink.getId(), outlink.getId(), myScenario.get.vehicleTypeIdForIndex(c), beta);
								}
							}
						}
						else {   // not measured scaled to 1-beta
							for(c=0;c<myScenario.get.numVehicleTypes();c++) { //
								if(variable_vtype[c]) {
									current_split = myNode.getSplitRatio(i, j, c);
									cms.set_split(inlink.getId(), outlink.getId(), myScenario.get.vehicleTypeIdForIndex(c),
											current_split * (1d - beta)); // split ratios differ across commodities
								}
							}
						}
					}
				}
			}
			cms.deploy(current_time_in_seconds);
		}
	}

}