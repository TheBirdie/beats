package edu.berkeley.path.beats.util;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map.Entry;

import org.apache.commons.math3.util.Pair;

import edu.berkeley.path.beats.simulator.Link;
import edu.berkeley.path.beats.simulator.Network;
import edu.berkeley.path.beats.link.Type;

public class NetworkObservator {
	Network _net;
	ArrayList<Link> _highwayLinks;
	HashMap<Link, Pair<Integer, Integer>> _linkToIdx;
	double _totalLength;
	double _discretSpace;

	double[] _observationDensity;
	double[] _observationOnramps;
	double[] _observationOfframps;

	public NetworkObservator(Network net, double discretizationSpace) {
		this._net = net;
		this._discretSpace = discretizationSpace;
		ComputeHighwayLinks();
		GenerateLinkToIdxMapping();
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
	void ComputeObservation(double[][] observation) {
        /*
        A [l, 4] array for all the HWY length with l = length of the highway
        	Dim0: density
        	Dim1: 0/1 presence of offramp
        	Dim2: on-ramp queue
        */

        int discretization_cells = (int) (1 + _totalLength / _discretSpace);

        for (Entry<Link, Pair<Integer, Integer>> entry : _linkToIdx.entrySet()) {
        	Link link = entry.getKey();
        	Pair<Integer, Integer> indexes = entry.getValue();
            boolean isFirst = true;
            
            for (int cell = indexes.getFirst(); cell < indexes.getSecond(); ++cell) {
            	observation[cell][0] = link.getDensityInVeh(0, 0) / link._length;
            	observation[cell][1] = LinkHasOfframp(link) ? 1.0 : 0.0;
                if (isFirst)
                	observation[cell][2] = Math.log(1. + LinkOnrampQueue(link));
                else
                	observation[cell][2] = 0.;
                isFirst = false;
            }
        }
	}
}
