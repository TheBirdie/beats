package edu.berkeley.path.beats.simulator;

import java.util.ArrayList;
import java.util.List;

/**
 * Created by gomes on 10/26/14.
 */
public class ScenarioUpdaterACTM extends ScenarioUpdaterAbstract {

    private List<Link> onramp_links;
    private List<Link> not_onramp_links;
    private List<FwyNode> fwy_nodes;

    public ScenarioUpdaterACTM(Scenario scenario){
        super(scenario,null,"A");
        System.out.println("ERROR!!! ACTM link and node behaviors not implemented");
    }

    @Override
    public void populate() {
        super.populate();
        // for actm, collect references to onramp links
        onramp_links = new ArrayList<Link>();
        not_onramp_links = new ArrayList<Link>();
        for (edu.berkeley.path.beats.jaxb.Link link : scenario.getNetworkSet().getNetwork().get(0).getLinkList().getLink())  {
            Link bLink = (Link)link;
            if(bLink.isOnramp())
                onramp_links.add(bLink);
            else
                not_onramp_links.add(bLink);
        }
    }

    @Override
    public void update() throws BeatsException {

        // CHECK NUMENSEMBLE == 1
        int e=0;
        double xi = 0.1;
        double gamma = 0.1;

        update_profiles();

        update_sensors_control_events();

        // compute demand and total supply for all links
        for(edu.berkeley.path.beats.jaxb.Network network : scenario.getNetworkSet().getNetwork())
            for(edu.berkeley.path.beats.jaxb.Link link : network.getLinkList().getLink()) {
                Link bLink = (Link)link;
                bLink.updateOutflowDemand();
                bLink.link_behavior.update_total_space_supply();
            }

        // mainline allocation for onramps only: xi*(njam-n)
        for(FwyNode fwy_node : fwy_nodes )
            if(fwy_node.onramp!=null)
                ((LinkBehaviorACTM) fwy_node.onramp.link_behavior).update_available_space_supply_for_onramp();

        // onramp flow = min(onramp demand,mainline supply for onramp)
        for (FwyNode fwy_node : fwy_nodes){
            if(fwy_node.onramp!=null){
                Node_FlowSolver_ACTM flow_solver = (Node_FlowSolver_ACTM) fwy_node.node.node_behavior.flow_solver;
                double[] demand = fwy_node.onramp.get_out_demand_in_veh(e);
                double supply = ((LinkBehaviorACTM) fwy_node.onramp.link_behavior).available_space_supply_for_onramp[e];
                double total_demand = BeatsMath.sum(demand);
                double ratio = Math.min(1d,supply/total_demand);
                fwy_node.r = Math.min(total_demand,supply);
                fwy_node.onramp.setOutflow(e, BeatsMath.times(demand,ratio) );
            }
        }

        // available supply for mainline = min( w*(totalsupply - gamma*r ) , F )
        for(FwyNode fwy_node : fwy_nodes){
            Link link = fwy_node.dn_ml;
            if(link==null)
                continue;
            FundamentalDiagram FD = link.currentFD(e);
            double r = fwy_node.onramp==null ? 0d : fwy_node.r;
            double w = FD.getWNormalized();
            double F = FD._getCapacityInVeh();
            link.link_behavior.available_space_supply[e] = Math.min( w*(link.get_total_space_supply_in_veh(e) - gamma*r) , F );
        }

        // mainline flow
        for(FwyNode fwy_node : fwy_nodes){

            Link link = fwy_node.dn_ml;
            if(link==null)
                continue;

            Node node = fwy_node.node;

            // update split ratio matrix
            Double3DMatrix[] splitratio_selected = node.select_and_perturb_split_ratio();

            // compute applied split ratio matrix
            Double3DMatrix splitratio_applied = node.node_behavior.sr_solver.computeAppliedSplitRatio(splitratio_selected[e],e);

            // compute node flows ..........................................
            Node_FlowSolver.IOFlow IOflow = node.node_behavior.flow_solver.computeLinkFlows(splitratio_applied,e);

            if(IOflow==null)
                return;

            // assign flow to input links ..................................
            for(int i=0;i< node.nIn;i++)
                node.input_link[i].setOutflow(e,IOflow.getIn(i));

            // assign flow to output links .................................
            for (int j=0;j< node.nOut;j++)
                node.output_link[j].setInflow(e,IOflow.getOut(j));
        }

        // update density
        for(edu.berkeley.path.beats.jaxb.Network network : scenario.getNetworkSet().getNetwork())
            update_density((Network) network);

        update_cumalitives_clock();
    }

    @Override
    protected LinkBehavior create_link_behavior(Link link) {
        return new LinkBehaviorACTM(link);
    }

    @Override
    protected Node_FlowSolver create_node_flow_solver(Node node) {
        return new Node_FlowSolver_ACTM(node);
    }


    public class FwyNode {
        Link dn_ml;
        Node node;
        Link onramp;
        double r;
    }

}
