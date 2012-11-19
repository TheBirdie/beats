package edu.berkeley.path.beats.processor;


import java.math.BigDecimal;
import java.sql.Timestamp;
import java.util.ArrayList;
import java.util.List;

import org.apache.torque.TorqueException;
import org.apache.torque.map.ColumnMap;
import com.workingdogs.village.DataSetException;
import com.workingdogs.village.Record;


import edu.berkeley.path.beats.om.LinkDataDetailedPeer;
import edu.berkeley.path.beats.om.LinkPerformanceDetailedPeer;

public class LinkPerformanceDetailed extends edu.berkeley.path.beats.om.LinkPerformanceDetailed {

	/**
	 * 
	 */
	private static final long serialVersionUID = 4961351269765575170L;

	

	/**
	 * calculates and saves performance measures
	 * @param originalData
	 * @param linkLength
	 * @param previousTs
	 * @return current time stamp
	 * @throws Exception 
	 */
	public long savePerformanceData(int recordNumber, List data, double linkLength, long previousTs) throws Exception {
		
		LinkDataDetailed obj = new LinkDataDetailed();
		
		long timeDelta =0;
		double timeDeltaInHours =0.0;
				
			LinkDataDetailedPeer.populateObject((Record)data.get(recordNumber), 1, obj);
			
			setPrimaryKey(obj.getPrimaryKey());
			
			
			if ( obj.getOutFlow() != null )
				setVmt(obj.getOutFlow().multiply(BigDecimal.valueOf(linkLength)));
			

			
			if ( previousTs > 0 ) timeDelta = getTs().getTime() - previousTs;
			timeDeltaInHours = timeDelta / 1000.0/60.0/60.0;
			
			if ( obj.getDensity() != null )
				setVht(obj.getDensity().multiply(BigDecimal.valueOf(timeDeltaInHours))); 
			
			
			if ( obj.getSpeed() != null && getVht() !=null && getVmt() !=null )
				setDelay(PerformanceData.delay(getVht(), getVmt(), obj.getSpeed()));

			
			setNew(true);
			save();	
	
		return getTs().getTime();
	}
	
	/**
	 * populates aggregated object, sets calculated data and insert
	 * @param table
	 * @param aggregation ID
	 * @param time
	 * @param originalData
	 * @param aggregatedData
	 * @return number of rows processed
	 */
    
    public  int saveAggregated(String table, long aggregationId, Long time, List originalData, List aggregatedData )  {
			
		Timestamp ts = new Timestamp(time);
		
		ArrayList<String> colList = getColumnsForAggreagtion(); 

		try {
			
			// Use the originalData record to populate non-aggregated values of the this row.
			LinkPerformanceDetailedPeer.populateObject((Record)originalData.get(0), 1, this);
			
			//Populate aggregated data
				
			for (int i=0; i < colList.size(); i++ ) {
				
				setByPeerName(table+"."+colList.get(i), (BigDecimal)(((Record)aggregatedData.get(0)).getValue(i+1).asBigDecimal()));
				
			}		
			
			setTs(ts);
			setAggTypeId(aggregationId);
			
			setNew(true);
			save();
			
			
		} catch (Exception e) {

			e.printStackTrace();
			return 0;
		}

		return 1;
		
	}
    
    /**
     * returns list of primary keys with values except time stamp and aggregation
     * @return string
     * @throws TorqueException
     * @throws DataSetException 
     */
    public String getListOfKeys(Record rec) throws TorqueException, DataSetException {
    	
    	String str = new String("");
    	int n=1;
    	
    	ColumnMap[] columns = getTableMap().getColumns();
    	
    	for (int i=0; i< columns.length; i++) {

    		if ( columns[i].isPrimaryKey() ) {
    			
    			if ( columns[i].getColumnName().equals("ts") || columns[i].getColumnName().equals("agg_type_id") ) {
    				// do not include time stamp or aggregation
    			}
    			else  {
    				    			
    				// include key name and value
    				
    				str  += " AND " + columns[i].getColumnName() + "=" + rec.getValue(n++).asString() ;	
    			}

    		}
    	}
    		
    	return str;
    }
   
    
    public static void removeNulls(edu.berkeley.path.beats.om.LinkPerformanceDetailed obj) {
    	
    	
    	BigDecimal zero = new BigDecimal(0);
    	

    	try {
			for (int i=0; i< obj.getTableMap().getColumns().length; i++) {

				if ( obj.getByPosition(i) == null ) {
					
					try {
						obj.setByPosition(i, zero);
					} catch (TorqueException e) {
						// TODO Auto-generated catch block
						e.printStackTrace();
					} catch (IllegalArgumentException e) {
						// TODO Auto-generated catch block
						e.printStackTrace();
					}
				}
			}
		} catch (TorqueException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
    		   	
	
    }
    
    
    /**
     * Creates a list of columns that should be aggregated
     * @return List of strings
     * @throws TorqueException
     * @throws DataSetException
     */
   public ArrayList<String> getColumnsForAggreagtion()  {
    	
	   ArrayList<String> colList = new ArrayList<String>();    	
    	
    	ColumnMap[] columns;
		try {
			columns = getTableMap().getColumns();
		
    	
    	for (int i=0; i< columns.length; i++) {

    		if ( columns[i].isPrimaryKey() ) {
    			
    		} else {
    			
    			if ( columns[i].getTorqueType().equals("DECIMAL") ) {
    				
    				colList.add(columns[i].getColumnName());
    				
    			}
    		}
    	}
    		
    	
		} catch (TorqueException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		
    	return colList;
		
    }
   
 
    
    /**
     * returns list of primary keys with values except time stamp and aggregation
     * @return string
     * @throws TorqueException
     * @throws DataSetException 
     */
    public String getLinkIdAndNetwokId(Record rec) throws TorqueException, DataSetException {
    	
    	String str = new String("");   	
    	
   	
    	
    	int n=1;
    	
    	ColumnMap[] columns = getTableMap().getColumns();
    	
    	for (int i=0; i< columns.length; i++) {

    		if ( columns[i].isPrimaryKey() ) {
    			
    			if ( columns[i].getColumnName().equals("ts") || columns[i].getColumnName().equals("agg_type_id") ) {
    				// do not include time stamp
    			}
    			else  {
    				
    			
    				// include key name and value
    				if (columns[i].getColumnName().equals("link_id")) {
    					str  += " AND id=\'" + rec.getValue(n).asString() + "\'";	
    				} else 
    				if (columns[i].getColumnName().equals("network_id")) {
    					str  += " AND network_id=\'" + rec.getValue(n).asString() + "\'";	
    				}
    					
    				n++;	
    					
    			}

    		}
    	}
  
    	
    	return str;
    }
    	
    	
    	   /**
         * returns list of primary keys except time stamp and aggregation
         * @return string
         * @throws TorqueException
         */
        public String getListOfKeys() throws TorqueException {
        	
        	String str = new String("");
        	
        	ColumnMap[] columns = getTableMap().getColumns();
        	
        	for (int i=0; i< columns.length; i++) {

        		if ( columns[i].isPrimaryKey() ) {
        			
        			if ( columns[i].getColumnName().equals("ts") || columns[i].getColumnName().equals("agg_type_id") ) {
        				// do not include time stamp or aggregation
        			}
        			else  {
        				// include key name
        				if (str.length() > 1 ) str += ", ";
        				
        				str  += columns[i].getColumnName();	
        			}

        		}
        		
        	}
        	  	
    	  	
    	
    	return str;
    }
        


	/**
	 * returns column number for given name
	 * @param name
	 * @return
	 */
     public int getColumnNumber(String name) {    	
     	
     	ColumnMap[] columns;
		try {			
				columns = getTableMap().getColumns();
			    	
		     	for (int i=0; i< columns.length; i++) {

		 			
		 			if ( columns[i].getColumnName().equals(name)  ) return i+1;	   		
	     	}
	     	
			} catch (TorqueException e) {
				
				return 0;
			}
     	    	 	
		return 0;
     }
       
}