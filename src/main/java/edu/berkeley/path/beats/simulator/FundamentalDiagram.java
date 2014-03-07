/**
 * Copyright (c) 2012, Regents of the University of California
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 *   Redistributions of source code must retain the above copyright notice,
 *   this list of conditions and the following disclaimer.
 *   Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 **/

package edu.berkeley.path.beats.simulator;

public final class FundamentalDiagram extends edu.berkeley.path.beats.jaxb.FundamentalDiagram{

	// does not change ....................................
	private Link myLink;
	private double _vf;                	// [-]
	private double _w;                	// [-]
	private double std_dev_capacity;	// [veh]
	
	// does change ........................................
	private double lanes;
	private double _densityJam;     	// [veh] 
	private double _capacity;   		// [veh] 
	private double _capacityDrop;     	// [veh] 

    private double conv_spd;
    private double conv_dty;

	/////////////////////////////////////////////////////////////////////
	// construction 
	/////////////////////////////////////////////////////////////////////

	public FundamentalDiagram(){}
	
	public FundamentalDiagram(Link myLink){
		this.myLink       = myLink;
		this.lanes 		  = myLink==null ? Double.NaN : myLink.get_Lanes();
		_densityJam 	  = Double.NaN;  
	    _capacity  		  = Double.NaN;
		_capacityDrop 	  = Double.NaN; 
	    _vf 			  = Double.NaN; 
	    _w 				  = Double.NaN; 
	    std_dev_capacity  = Double.NaN;
	}
	
	// fundamental diagram created from jaxb objects must have all values filled in. 
	public FundamentalDiagram(Link myLink,edu.berkeley.path.beats.jaxb.FundamentalDiagram jaxbfd){
		this.myLink       = myLink;
		this.lanes 		  = myLink==null ? Double.NaN : myLink.get_Lanes();
		_densityJam 	  = Double.NaN;  
	    _capacity  		  = Double.NaN;
		_capacityDrop 	  = Double.NaN; 
	    _vf 			  = Double.NaN; 
	    _w 				  = Double.NaN; 
	    std_dev_capacity  = Double.NaN;

        this.setOrder(jaxbfd.getOrder());
        this.setCapacity(jaxbfd.getCapacity());
        this.setCapacityDrop(jaxbfd.getCapacityDrop());
        this.setStdDevCapacity(jaxbfd.getStdDevCapacity());
        this.setFreeFlowSpeed(jaxbfd.getFreeFlowSpeed());
        this.setCongestionSpeed(jaxbfd.getCongestionSpeed());
        this.setCriticalSpeed(jaxbfd.getCriticalSpeed());
        this.setStdDevFreeFlowSpeed(jaxbfd.getStdDevFreeFlowSpeed());
        this.setStdDevCongestionSpeed(jaxbfd.getStdDevCongestionSpeed());
        this.setJamDensity(jaxbfd.getJamDensity());
        this.setTransitionDensity(jaxbfd.getTransitionDensity());

        if(myLink==null)
	    	return;
	    
	    // procedure for determining fd parameters.
	    // jaxbfd may contain up to 4 of: densityJam, capacity, vf, and w. 
	    // if any of these is missing, assume the fd is triangular. 
	    // if more than one is missing, use default values up to 3 values, then define the fourth with triangle. 
	    // The order of default application is capacity, v, w, densityjam.
	    
	    // record jaxbfd values and undefined parameters
	    int nummissing = 0;
	    boolean missing_capacity, missing_vf, missing_w, missing_densityJam;
	    double simDtInSeconds = myLink.getMyNetwork().getMyScenario().getSimdtinseconds();

        conv_spd = simDtInSeconds / myLink.getLengthInMeters();
        conv_dty = lanes * myLink.getLengthInMeters();
        double conv_flw = conv_spd*conv_dty;

		if(!Double.isNaN(jaxbfd.getCapacity())){
			_capacity = jaxbfd.getCapacity() * conv_flw;
			missing_capacity = false;
		} 
		else{
			nummissing++;
			missing_capacity = true;
		}
	    
		if(!Double.isNaN(jaxbfd.getFreeFlowSpeed())){
			_vf = jaxbfd.getFreeFlowSpeed() * conv_spd;
			missing_vf = false;
		} 
		else{
			nummissing++;
			missing_vf = true;
		}
	
		if(!Double.isNaN(jaxbfd.getCongestionSpeed())){
			_w = jaxbfd.getCongestionSpeed() * conv_spd;
			missing_w = false;
		} 
		else{
			nummissing++;
			missing_w = true;
		}
	
		if(jaxbfd.getJamDensity()!=null){
			_densityJam = jaxbfd.getJamDensity().doubleValue() * conv_dty;
			missing_densityJam = false;
		} 
		else{
			nummissing++;
			missing_densityJam = true;
		}

		// in order, check for missing values and fill in until we are able to make triangle
		if(missing_capacity && nummissing>1){
			_capacity = Defaults.capacity * conv_flw;
			nummissing--;
		}

		if(missing_vf && nummissing>1){
			_vf = Defaults.vf * conv_spd;
			nummissing--;
		}

		if(missing_w && nummissing>1){
			_w = Defaults.w * conv_spd;
			nummissing--;
		}

		if(missing_densityJam && nummissing>1){
			_densityJam = Defaults.densityJam * conv_dty;
			nummissing--;
		}
	    
	    // now there should be no more than one missing value
		if(nummissing>1)
			System.out.println("BIG MISTAKE!!!!");
	    
		// if there is one missing, compute it with triangular assumption
		
		if(nummissing==1){
			if(missing_capacity){
				_capacity = _densityJam / (1.0/_vf + 1.0/_w);
                setCapacity(_capacity/conv_flw);
            }
			if(missing_vf){
				_vf = 1.0 / ( _densityJam/_capacity - 1.0/_w );
                setFreeFlowSpeed(_vf/conv_spd);
            }
			if(missing_w){
				_w = 1.0 / ( _densityJam/_capacity - 1.0/_vf );
                setCongestionSpeed(_w/conv_spd);
            }
			if(missing_densityJam){
				_densityJam = _capacity*(1.0/_vf + 1.0/_w);
                setJamDensity(_densityJam/conv_dty);
            }
		}
		
		// set capacity drop and capacity uncertainty
		if(jaxbfd.getStdDevCapacity()!=null)
			std_dev_capacity = jaxbfd.getStdDevCapacity().doubleValue() * conv_flw;
		else
			std_dev_capacity = 0.0;

		if(!Double.isNaN(jaxbfd.getCapacityDrop()))
			_capacityDrop = jaxbfd.getCapacityDrop() * conv_flw;
		else
			_capacityDrop = Defaults.capacityDrop * conv_flw;
	}
	
	// fundamental diagrams created from other fundamental diagrams copy all values. 
	public FundamentalDiagram(Link myLink,edu.berkeley.path.beats.simulator.FundamentalDiagram fd){
		if(myLink==null)
			return;
		this.myLink = myLink;
		this.lanes = myLink.get_Lanes();
		_densityJam 	  = Double.NaN;  
	    _capacity         = Double.NaN;
		_capacityDrop 	  = Double.NaN; 
	    _vf 			  = Double.NaN; 
	    _w 				  = Double.NaN; 
	    std_dev_capacity  = Double.NaN;

	    this.copyfrom(fd);		// copy and normalize
	}
	
	/////////////////////////////////////////////////////////////////////
	// protected interface
	/////////////////////////////////////////////////////////////////////

	protected void set_capacity(double _capacity) {
		this._capacity = _capacity;
	}	
	
	// we do not have to worry about getters returning NaN:
	// they are only called for fundamental diagrams belonging
	// to links, these are initialized with default values, and 
	// copyfrom only replaces with non-nan values.

	protected double _getDensityJamInVeh() {
		return _densityJam;
	}

	protected double _getCapacityInVeh() {
		return _capacity;
	}

	protected double _getCapacityDropInVeh() {
		return _capacityDrop;
	}

	protected double getVfNormalized() {
        return myLink.is_queue_link ? 1.0 : _vf;
	}

	protected double getWNormalized() {
        return myLink.is_queue_link ? 1.0 : _w;
	}

	protected double getDensityCriticalInVeh() {
		return _capacity/_vf;
	}

	protected void setLanes(double newlanes){
		if(newlanes<=0)
			return;
		if(BeatsMath.equals(newlanes,lanes))
			return;
		double alpha = newlanes/lanes;
		_densityJam 	  *= alpha; 
		_capacity  		  *= alpha; 
		_capacityDrop 	  *= alpha; 
		lanes = newlanes;
	}
	
	/////////////////////////////////////////////////////////////////////
	// reset / validate
	/////////////////////////////////////////////////////////////////////

	// assign default values parameters and normalize
 	protected void settoDefault(){
		if(myLink==null)
			return;
		double simDtInSeconds = myLink.getMyNetwork().getMyScenario().getSimdtinseconds();
		double lengthInMeters = myLink.getLengthInMeters();
		_densityJam 	  = Defaults.densityJam		* lanes * lengthInMeters;
		_capacity  		  = Defaults.capacity		* lanes * simDtInSeconds;
		_capacityDrop 	  = Defaults.capacityDrop	* lanes * simDtInSeconds;
		_vf = Defaults.vf * simDtInSeconds / lengthInMeters;
		_w  = Defaults.w  * simDtInSeconds / lengthInMeters;
	}

 	// copy per lane parameters from jaxb and normalize
	protected void copyfrom(edu.berkeley.path.beats.jaxb.FundamentalDiagram fd){

		if(fd==null)
			return;
		if(myLink==null)
			return;
		
		double value;
		double simDtInSeconds = myLink.getMyNetwork().getMyScenario().getSimdtinseconds();

		if(fd.getJamDensity()!=null){
			value = fd.getJamDensity().doubleValue();		// [veh/meter/lane]
			_densityJam = value * lanes * myLink.getLengthInMeters();
		} 

		if(!Double.isNaN(fd.getCapacity())){
			value = fd.getCapacity();			// [veh/second/lane]
			_capacity = value * lanes * simDtInSeconds;
		} 
		
		if(fd.getStdDevCapacity()!=null){
			value = fd.getStdDevCapacity().doubleValue();	// [veh/second/lane]
			std_dev_capacity = value * lanes * simDtInSeconds;
		}
		
		if(!Double.isNaN(fd.getCapacityDrop())){
			value = fd.getCapacityDrop();		// [veh/second/lane]
			_capacityDrop = value * lanes * simDtInSeconds;
		} 
		
		if(!Double.isNaN(fd.getFreeFlowSpeed())){
			value = fd.getFreeFlowSpeed();		// [meters/second]
			_vf = value * simDtInSeconds / myLink.getLengthInMeters();
		}

		if(!Double.isNaN(fd.getCongestionSpeed())){
			value = fd.getCongestionSpeed();		// [meters/second]
			_w = value * simDtInSeconds / myLink.getLengthInMeters();
		}

	}

 	// clone a fd
	public void copyfrom(edu.berkeley.path.beats.simulator.FundamentalDiagram that){
		if(that==null)
			return;
		this.myLink = that.myLink;
		this._capacity = that._capacity;
		this._capacityDrop = that._capacityDrop;
		this._densityJam = that._densityJam;
		this._vf = that._vf;
		this._w = that._w;
		this.lanes = that.lanes;
		this.std_dev_capacity = that.std_dev_capacity;

        this.setOrder(that.getOrder());
        this.setCapacity(that.getCapacity());
        this.setCapacityDrop(that.getCapacityDrop());
        this.setStdDevCapacity(that.getStdDevCapacity());
        this.setFreeFlowSpeed(that.getFreeFlowSpeed());
        this.setCongestionSpeed(that.getCongestionSpeed());
        this.setCriticalSpeed(that.getCriticalSpeed());
        this.setStdDevFreeFlowSpeed(that.getStdDevFreeFlowSpeed());
        this.setStdDevCongestionSpeed(that.getStdDevCongestionSpeed());
        this.setJamDensity(that.getJamDensity());
        this.setTransitionDensity(that.getTransitionDensity());

    }
	
	protected void reset(Scenario.UncertaintyType uncertaintyModel){
		if(myLink==null)
			return;
		// set lanes back to original value
		setLanes(myLink.get_Lanes());
	}
	
	// produce a sample fundamental diagram with this one as expected value.
	protected FundamentalDiagram perturb(){
		if(myLink==null)
			return null;
		// make a copy of this fundamental diagram
		FundamentalDiagram samp = new FundamentalDiagram(myLink,this);
		
		// perturb it
		if(!Double.isNaN(std_dev_capacity) && std_dev_capacity>0){
			switch(myLink.getMyNetwork().getMyScenario().getUncertaintyModel()){
			case uniform:
				samp._capacity += BeatsMath.sampleZeroMeanUniform(std_dev_capacity);
				break;

			case gaussian:
				samp._capacity += BeatsMath.sampleZeroMeanUniform(std_dev_capacity);
				break;
			}			
		}
		
		// adjustments to sampled fd:
		
		// non-negativity
		samp._capacity = Math.max(samp._capacity,0.0);
		
		// density_critical no greater than dens_crit_congestion
        double dens_critical = samp._capacity / samp._vf;
		double dens_crit_congestion = samp._densityJam-samp._capacity/samp._w;	// [veh]
		if(BeatsMath.greaterthan(dens_critical,dens_crit_congestion)){
//			samp.density_critical = dens_crit_congestion;
			samp._capacity = samp._vf * dens_critical;
		}

		return samp;
	}
	
	protected void validate(){
				
		if(myLink==null)
			return;
				
		if(_vf<0 || _w<0 || _densityJam<0 || _capacity<0 || _capacityDrop<0)
			BeatsErrorLog.addError("Negative fundamental diagram parameters for link ID=" + myLink.getId());

        double dens_critical = _capacity / _vf;
        double dens_crit_congestion = _densityJam-_capacity/_w;	// [veh]
		if(BeatsMath.greaterthan(dens_critical,dens_crit_congestion))
			BeatsErrorLog.addError("Maximum allowable critical density for link " + myLink.getId() + " is " + dens_crit_congestion + "(current="+dens_critical+")");
		
		if(_vf>1 && !myLink.isSource())
			BeatsErrorLog.addError("CFL condition violated, FD for link " + myLink.getId() + " has vf=" + _vf);

		if(_w>1 && !myLink.isSource())
			BeatsErrorLog.addError("CFL condition violated, FD for link " + myLink.getId() + " has w=" + _w);
	}

}
