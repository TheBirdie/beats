package edu.berkeley.path.beats.control;

import edu.berkeley.path.beats.control.rm_interface.RampMeteringControlSet;
import edu.berkeley.path.beats.control.rm_interface.RampMeteringPolicyMaker;
import edu.berkeley.path.beats.control.rm_interface.RampMeteringPolicySet;
import edu.berkeley.path.beats.jaxb.FundamentalDiagramSet;
import edu.berkeley.path.beats.simulator.*;

import java.util.Properties;

public class PolicyMaker_Alinea implements RampMeteringPolicyMaker {

    @Override
    public RampMeteringPolicySet givePolicy(Network net, FundamentalDiagramSet fd, DemandSet demand, SplitRatioSet splitRatios, InitialDensitySet ics, RampMeteringControlSet control, Double dt, Properties props) {

//        for(edu.berkeley.path.beats.jaxb.Link jaxbL : net.getListOfLinks()){
//
//            Link L = (Link) jaxbL;
//
//            if(L.isSource()){
//                Link upML = getUpstreamMainlineDensity(net,L)
//
//
//            }
//
//        }
        return null;
    }

//    private Link getIncomingMainlineLinkForNode(Network net,Node node){
//        Link [] in_links = ;
//        for(Link link : node.getInput_link())
//            if(link.getLinkType().equals())
//
//    }
//
//    private Node getEndNodeForLinkId(Network net,Link link){
//        return net.link.getEnd_node();
//    }
//
//    private double getDensityForLinkId(InitialDensitySet ics,long link_id){
//        return BeatsMath.sum(ics.getDensityForLinkIdInVeh(link_id));
//    }
}
