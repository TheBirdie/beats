package edu.berkeley.path.beats.util;

import java.math.BigDecimal;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map.Entry;

import org.apache.commons.math3.util.Pair;

import edu.berkeley.path.beats.simulator.Controller;
import edu.berkeley.path.beats.simulator.ControllerSet;
import edu.berkeley.path.beats.simulator.Link;
import edu.berkeley.path.beats.simulator.Network;
import edu.berkeley.path.beats.simulator.Scenario;
import edu.berkeley.path.beats.actuator.ActuatorRampMeter;
import edu.berkeley.path.beats.jaxb.Demand;
import edu.berkeley.path.beats.jaxb.DemandProfile;
import edu.berkeley.path.beats.jaxb.DemandSet;
import edu.berkeley.path.beats.link.Type;

public class NetworkObservator {
    Network _net;
    Scenario _scenario;
    ArrayList<Link> _highwayLinks;
    ArrayList<ActuatorRampMeter> _actuators;
    HashMap<Link, Pair<Integer, Integer>> _linkToIdx;
    double _totalLength;
    double _discretSpace;
    double _densityMean;
    double _densityVar;

    double[] _observationDensity;
    double[] _observationOnramps;
    double[] _observationOfframps;

    public NetworkObservator(Scenario scenario, Network net, double discretizationSpace, double mean, double var) {
        this._net = net;
        this._scenario = scenario;
        this._discretSpace = discretizationSpace;
        this._densityMean = mean;
        this._densityVar = var;
        ComputeHighwayLinks();
        GenerateLinkToIdxMapping();
        GenerateActuatorsArray();
    }

    Link GetAnyFreewayLink() {
        List<edu.berkeley.path.beats.jaxb.Link> links = _net.getLinkList().getLink();
        for (edu.berkeley.path.beats.jaxb.Link link_: links) {
            Link link = (Link)link_;
            if (link.link_type.compareTo(Type.freeway) == 0)
                return link;
        }
        return null;
    }
    public void CollectLocalIntegralOutflow(double[] localDensity) {
    	int onramp_idx = 0;
    	for (Link link: _highwayLinks) {
    		Link[] output_links = link.getEnd_node().output_link;
    		Link[] input_links = link.getBegin_node().input_link;
    		// Next onramp idx if we are at an onramp
    		for (Link input_link: input_links)
    			if (input_link.link_type.compareTo(Type.onramp) == 0)
    				++onramp_idx;
    		double outflow_here = 0;
    		for (Link output_link: output_links)
    			if (output_link.link_type.compareTo(Type.offramp) == 0 || output_link.issink)
            		for (int j = 0; j < output_link.outflow.length; ++j)
            			for (int k = 0; k < output_link.outflow[j].length; ++k)
            				outflow_here += output_link.outflow[j][k];
    		if (onramp_idx < localDensity.length) // Skip after last onramp
    			localDensity[onramp_idx] += outflow_here;
			if (onramp_idx > 0) // Skip before first onramp
				localDensity[onramp_idx-1] += outflow_here;
    	}
    }
    void ComputeHighwayLinks() {
        Link fwy_any_link = GetAnyFreewayLink();
        Link cur_link = fwy_any_link;
        assert(cur_link.getBegin_node().input_link.length == 0);

        _highwayLinks = new ArrayList<Link>();
        _highwayLinks.add(cur_link);
        _totalLength = cur_link._length;
        while (true) {
            Link[] output_links = cur_link.getEnd_node().output_link;
            cur_link = null;
            for (Link l: output_links) {
                if (l.link_type.compareTo(Type.freeway) == 0)
                    cur_link = l;
            }
            if (cur_link == null)
                return;
            _highwayLinks.add(cur_link);
            _totalLength += cur_link._length;
        }
    }
    void GenerateLinkToIdxMapping() {
        _linkToIdx = new HashMap<Link, Pair<Integer, Integer>>();
        int cur_discr_cell = 0;
        double dist_modification = 0.;
        for (Link link: _highwayLinks) {
            double link_length = link._length + dist_modification;
            int index_first = cur_discr_cell;
            while (link_length > 0) {
                link_length -= _discretSpace;
                cur_discr_cell++;
            }
            dist_modification = link_length;
            _linkToIdx.put(link, new Pair<Integer, Integer>(index_first, cur_discr_cell));
        }
    }
    void GenerateActuatorsArray() {
        _actuators = new ArrayList<ActuatorRampMeter>();
        ControllerSet controllerSet = _scenario.controllerset;
        ArrayList<Controller> controllers = controllerSet.get_Controllers();
        for (Controller c: controllers) {
            int num = c.getNumActuators();
            for (int i = 0; i < num; ++i)
                _actuators.add((ActuatorRampMeter) c.getNthActuator(i));
        }
    }

    static boolean LinkHasOfframp(Link link) {
        return link.getEnd_node().nOut > 1;
    }
    static double LinkOnrampQueue(Link link) {
        for (Link in_link: link.getBegin_node().input_link) {
            if (in_link.link_type.compareTo(Type.onramp) == 0)
                return in_link.getDensityInVeh(0, 0);
        }
        return 0.;
    }
    public void ComputeObservation(double[][] observation) {
        /*
        A [l, 4] array for all the HWY length with l = length of the highway
            Dim0: density
            Dim1: 0/1 presence of offramp
            Dim2: on-ramp queue
        */

        for (Entry<Link, Pair<Integer, Integer>> entry : _linkToIdx.entrySet()) {
            Link link = entry.getKey();
            Pair<Integer, Integer> indexes = entry.getValue();
            boolean isFirst = true;
            
            for (int cell = indexes.getFirst(); cell < indexes.getSecond(); ++cell) {
                observation[cell][0] = (link.getDensityInVeh(0, 0) / link._length - _densityMean) / _densityVar;
                observation[cell][1] = LinkHasOfframp(link) ? 1.0 : 0.0;
                if (isFirst)
                {
                    double queue = LinkOnrampQueue(link);
                    observation[cell][2] = Math.log(1. + queue);
                }
                else
                    observation[cell][2] = 0.;
                isFirst = false;
            }
        }
    }
    public void ComputeFreewaySpeeds(double[] speeds) {
        /*
        A [l, 4] array for all the HWY length with l = length of the highway
            Dim0: density
            Dim1: 0/1 presence of offramp
            Dim2: on-ramp queue
        */

        for (Entry<Link, Pair<Integer, Integer>> entry : _linkToIdx.entrySet()) {
            Link link = entry.getKey();
            Pair<Integer, Integer> indexes = entry.getValue();

            for (int cell = indexes.getFirst(); cell < indexes.getSecond(); ++cell) {
                speeds[cell] = link.computeSpeedInMPS(0);
            }
        }
    }
    public void SetActuatorValues(double[] actions) {
        for (int i = 0; i < _actuators.size(); ++i) {
            actions[i] = Double.max(actions[i], 0d);
            _actuators.get(i).setMeteringRateInVPH(actions[i] * 100);
        }
    }
    public double ComputeIntegralDensity() {
    	double integralDensity = 0;
    	int numVehTypes = _scenario.get.numVehicleTypes();
        for(int i=0;i<_net.getLinkList().getLink().size();i++){
            Link link = (Link) _net.getLinkList().getLink().get(i);
            Double [] linkdensity = link.getDensityInVeh(0);
            if(linkdensity != null)
                for(int j=0;j<numVehTypes;j++)
                	integralDensity += linkdensity[j];
        }
        return integralDensity;
    }
    public double SetAllDemandToMaximum() {
    	double sumAllDemand = 0.0;
    	for(int i=0;i<_net.getLinkList().getLink().size();i++){
            Link link = (Link) _net.getLinkList().getLink().get(i);
            edu.berkeley.path.beats.simulator.DemandProfile demandProfile = link.getDemandProfile();
            if (demandProfile != null) {
	            List<Demand> demandsList = demandProfile.getDemand();
	            for (Demand d: demandsList) {
	                double max = 0;
	                int count = 0;
	            	// <demand vehicle_type_id="1">1158.000,1158.000,1158.0...
	            	Data1D data1d = new Data1D(d.getContent(),",");		
	        		if (!data1d.isEmpty()) {
	        			StringBuilder sb = new StringBuilder();
	        			for (BigDecimal val : data1d.getData()) {
	        				max = Math.max(max, val.doubleValue());
	        				++count;
	        			}
	        			for (int j = 0; j < count; ++j)
	        			{
	        				if (0 < sb.length()) sb.append(',');
	        				sb.append(max);
	        			}
	        			d.setContent(sb.toString());
	        		}
	        		sumAllDemand += max;
	            }
            }
    	}
    	return sumAllDemand;
    }
}
