package edu.berkeley.path.beats.simulator.output;

import java.io.FileOutputStream;
import java.io.IOException;
import java.io.OutputStreamWriter;
import java.io.Writer;
import java.util.Properties;

import edu.berkeley.path.beats.simulator.utils.BeatsException;
import edu.berkeley.path.beats.simulator.Link;
import edu.berkeley.path.beats.simulator.LinkCumulativeData;
import edu.berkeley.path.beats.simulator.Network;
import edu.berkeley.path.beats.simulator.Scenario;

public class OutputWriterTSV extends OutputWriterBase {

	protected static String delim = "\t";
	private String prefix;

	public OutputWriterTSV(Scenario scenario, Properties props,double outDt,int outsteps,double outStart){
		super(scenario,outDt,outsteps,outStart);
		if (null != props) 
			prefix = props.getProperty("prefix");
		if (null == prefix) 
			prefix = "output";
		requestLinkCumulatives();
	}

	@Override
	public void open(int run_id) throws BeatsException {
	}
	
	@Override
	public void close(){
	}

	@Override
	public void recordstate(double time, boolean exportflows, int outsteps) throws BeatsException {
		
		if(scenario==null)
			return;

		try {

			double density;
			double flow;
			double speed;
			
			String timestr = String.format("%.0f", scenario.get.currentTimeInSeconds()) ;
			Writer out = new OutputStreamWriter(new FileOutputStream(prefix+"_"+timestr+".txt"));
		
			for(edu.berkeley.path.beats.jaxb.Network network : scenario.getNetworkSet().getNetwork()){				
				
				if( ((Network)network).isEmpty() )
					continue;
				
				for(edu.berkeley.path.beats.jaxb.Link jlink : network.getLinkList().getLink()){
					Link link = (Link) jlink;
					LinkCumulativeData link_cum_data = getCumulatives(link);
					if(exportflows){
						density = link_cum_data.getMeanTotalDensityInVeh(0)/link.getLengthInMeters();
						flow = link_cum_data.getCumulativeTotalOutputFlowInVeh(0)/outDt;
						speed = link.computeSpeedInMPS(0);
					} 
					else{
						density = link.getTotalDensityInVeh(0)/link.getLengthInMeters();
						flow = Double.NaN;
						speed = Double.NaN;
					}

					out.write(String.format("%d\t%f\t%f\t%f\n",link.getId(),speed,density,flow));
				}
			}
			
			out.close();

			
		} catch (IOException e) {
			throw new BeatsException(e);
		}
	}


}
