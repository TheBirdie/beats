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

/****************************************************************************/
/************        Author: Alexey Goder alexey@goder.com  *****************/
/************                    Dec 10, 2012               *****************/
/****************************************************************************/


package edu.berkeley.path.beats.processor;

import java.awt.BasicStroke;
import java.awt.Color;
import java.awt.Container;
import java.awt.Dimension;
import java.awt.GridBagLayout;
import java.awt.Rectangle;
import java.awt.Shape;
import java.awt.Stroke;
import java.awt.geom.Ellipse2D;
import java.awt.geom.Rectangle2D;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.math.BigDecimal;
import java.sql.Timestamp;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Date;

import org.apache.torque.TorqueException;
import org.apache.torque.util.BasePeer;

import com.workingdogs.village.DataSetException;
import com.workingdogs.village.Record;
import com.itextpdf.text.BadElementException;
import com.itextpdf.text.Document;
import com.itextpdf.text.DocumentException;
import com.itextpdf.text.Element;
import com.itextpdf.text.ExceptionConverter;
import com.itextpdf.text.Font;
import com.itextpdf.text.Image;
import com.itextpdf.text.Paragraph;
import com.itextpdf.text.Phrase;
import com.itextpdf.text.pdf.ColumnText;
import com.itextpdf.text.pdf.PdfPCell;
import com.itextpdf.text.pdf.PdfPTable;
import com.itextpdf.text.pdf.PdfPageEventHelper;
import com.itextpdf.text.pdf.PdfTemplate;
import com.itextpdf.text.pdf.PdfWriter;

import edu.berkeley.path.beats.db.OutputToCSV;

import org.jfree.chart.ChartFactory;
import org.jfree.chart.ChartUtilities;
import org.jfree.chart.JFreeChart;
import org.jfree.chart.plot.DefaultDrawingSupplier;
import org.jfree.chart.plot.PlotOrientation;
import org.jfree.chart.plot.XYPlot;
import org.jfree.chart.renderer.xy.StandardXYItemRenderer;
import org.jfree.chart.renderer.xy.XYItemRenderer;
import org.jfree.chart.renderer.xy.XYLineAndShapeRenderer;
import org.jfree.data.xy.XYSeries;
import org.jfree.data.xy.XYSeriesCollection;
import org.jfree.data.xy.XYZDataset;

import org.apache.poi.hslf.record.TextHeaderAtom;
import org.apache.poi.hslf.usermodel.SlideShow;
import org.apache.poi.hslf.usermodel.RichTextRun;
import org.apache.poi.hslf.model.*;
import org.apache.commons.io.FileUtils;

public class PdfReport extends AggregateData {

	Document document;
	
	private static Font headFont = new Font(Font.FontFamily.TIMES_ROMAN, 24,
			Font.BOLD);
	private static Font catFont = new Font(Font.FontFamily.TIMES_ROMAN, 18,
			Font.BOLD);
	private static Font normalFont = new Font(Font.FontFamily.TIMES_ROMAN, 12,
			Font.NORMAL);
	private static Font normalBold = new Font(Font.FontFamily.TIMES_ROMAN, 12,
			Font.BOLD);

	private static Font subFont = new Font(Font.FontFamily.TIMES_ROMAN, 9,
			Font.NORMAL);
	private static Font tableFont = new Font(Font.FontFamily.TIMES_ROMAN, 8,
			Font.NORMAL);
	private static Font smallFont = new Font(Font.FontFamily.TIMES_ROMAN, 9,
			Font.NORMAL);
	private static Font smallBold = new Font(Font.FontFamily.TIMES_ROMAN, 10,
			Font.BOLD);
	
	private static String slideFont = new String("Calibri");
	
	/**
     * Inner class to add a table as header.
     */
    class TableHeader extends PdfPageEventHelper {
        /** The header text. */
        String header;
        /** The template with the total number of pages. */
        PdfTemplate total;
 
        /**
         * Allows us to change the content of the header.
         * @param header The new header String
         */
        public void setHeader(String header) {
            this.header = header;
        }
 
        /**
         * Creates the PdfTemplate that will hold the total number of pages.
         * @see com.itextpdf.text.pdf.PdfPageEventHelper#onOpenDocument(
         *      com.itextpdf.text.pdf.PdfWriter, com.itextpdf.text.Document)
         */
        public void onOpenDocument(PdfWriter writer, Document document) {
            total = writer.getDirectContent().createTemplate(30, 16);
        }
 
        /**
         * Adds a header to every page
         * @see com.itextpdf.text.pdf.PdfPageEventHelper#onEndPage(
         *      com.itextpdf.text.pdf.PdfWriter, com.itextpdf.text.Document)
         */
        public void onEndPage(PdfWriter writer, Document document) {
        	
        	if (writer.getPageNumber() < 2 ) return;
        	
            PdfPTable table = new PdfPTable(3);
            try {
                table.setWidths(new int[]{24, 24, 2});
                table.setTotalWidth(527);
                table.setLockedWidth(true);
                //table.getDefaultCell().setFixedHeight(12);
                
                table.getDefaultCell().setBorder(0);
                table.addCell(header);
                table.getDefaultCell().setHorizontalAlignment(Element.ALIGN_RIGHT);
                table.getDefaultCell().setVerticalAlignment(Element.ALIGN_BOTTOM);
                table.addCell(new Phrase("Page "+writer.getPageNumber()+" of",smallFont));
                //table.addCell(String.format("Page %d of", writer.getPageNumber()));
                PdfPCell cell = new PdfPCell(Image.getInstance(total));
                
                cell.setBorder(0);
                cell.setVerticalAlignment(Element.ALIGN_MIDDLE);
                table.addCell(cell);
                table.writeSelectedRows(0, -1, 48, 824, writer.getDirectContent());
            }
            catch(DocumentException de) {
                throw new ExceptionConverter(de);
            }
        }
 
        /**
         * Fills out the total number of pages before the document is closed.
         * @see com.itextpdf.text.pdf.PdfPageEventHelper#onCloseDocument(
         *      com.itextpdf.text.pdf.PdfWriter, com.itextpdf.text.Document)
         */
        public void onCloseDocument(PdfWriter writer, Document document) {
            ColumnText.showTextAligned(total, Element.ALIGN_LEFT,
                    new Phrase(String.valueOf(writer.getPageNumber() - 1)),
                    2, 2, 0);
        }
    }
 
    /**
     * Form the slide title
     * @param section
     * @param table
     * @return
     */
    public String getSlideTitle(String section, String table) {
    	
    	String str = new String();
    	
    	str = section;
    	
    	if ( table.indexOf("performance") >=0 ) str += " Performance";
    	if (table.indexOf("detailed") >=0 ) str += " Detailed";
    	else 
    		if (table.indexOf("total") >=0 ) str += " Total";  			
    	
    	return str;
    }
    
    
	public void outputPdf(String table) {
		
		AggregateData.reportToStandard("Report request: " + "report_request.xml" );
		
		ReportRequest rr = new  ReportRequest();
		rr.readXMLFile("report_request.xml");

		// slide show
		
		SlideShow ppt = new SlideShow();
		rr.setPpt(ppt);

		try {
		
			document = new Document();
			//PdfWriter.getInstance(document, new FileOutputStream(table+".pdf"));
			
			PdfWriter writer = PdfWriter.getInstance(document, new FileOutputStream("traffic_report.pdf"));
	        
			
			TableHeader event = new TableHeader();
	        writer.setPageEvent(event);
	        
			document.open();
			addMetaData(document);
			addTitlePage(document, rr);		
			
			rr.setMultiplier(1.0);
			
			// Network section
			if ( rr.getNetworkPerformance() ) {
				if ( rr.getDetailed() )
					addNetworkSection(document, "link_performance_detailed", rr);
				else
					addNetworkSection(document, "link_performance_total", rr);
			}
			
			
			// Route section addContourPlots
			if ( rr.getRouteData() ) {
				if ( rr.getDetailed() )
					addContourPlots(document, "link_data_detailed", rr);
				else
					addContourPlots(document, "link_data_total", rr);
			}
			
			if ( rr.getRoutePerformance() ) {
				if ( rr.getDetailed() ) {
					addContourPlots(document, "link_performance_detailed", rr);
					addRouteSection(document, "link_performance_detailed", rr);
				}
				else {
					addContourPlots(document, "link_performance_total", rr);
					addRouteSection(document, "link_performance_total", rr);
					addRoutePerfomanceSection(document, "route_performance_total", rr);
				}
			}
			
			// Link section
			if ( rr.getLinkData() ) {
				if ( rr.getDetailed() )
					addLinkSection(document, "link_data_detailed", rr);
				else
					addLinkSection(document, "link_data_total", rr);
			}
			
			if ( rr.getLinkPerformance() ) {
				if ( rr.getDetailed() )
					addLinkSection(document, "link_performance_detailed", rr);
				else
					addLinkSection(document, "link_performance_total", rr);
			}
			
			// Onramp section addOnrampSection
			if ( rr.getOnRampData() ) {
				if ( rr.getDetailed() )
					addOnrampSection(document, "link_data_detailed", rr);
				else
					addOnrampSection(document, "link_data_total", rr);
			}
			
			if ( rr.getOnRampPerformance() ) {
				if ( rr.getDetailed() )
					addOnrampSection(document, "link_performance_detailed", rr);
				else
					addOnrampSection(document, "link_performance_total", rr);
			}
			

			document.close();
		} catch (Exception e) {
			e.printStackTrace();
		}
		
		//save changes in a file
	    FileOutputStream out;
		try {
			out = new FileOutputStream("traffic.ppt");
		} catch (FileNotFoundException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
			return;
		}
	    try {
	    	
			ppt.write(out);
			
			out.close();
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		
	    reportToStandard("DONE");
	}

	// iText allows to add metadata to the PDF which can be viewed in your Adobe
	// Reader
	// under File -> Properties
	private static void addMetaData(Document document) {
		document.addTitle("INTEGRATED CORRIDOR MANAGEMENT PROJECT");
		document.addSubject("Java Generated");
		document.addCreator("Alexey Goder");
	}
	
	/**
	 * return center aligned paragraph
	 * @param s
	 * @param f
	 * @return
	 */
	public static Paragraph addCenter(String s, Font f) {
		Paragraph p = new Paragraph(s,f);
		p.setAlignment(Element.ALIGN_CENTER);
		return p;
	}
	
	/**
	 * return left aligned paragraph
	 * @param s
	 * @param f
	 * @return
	 */
	public static Paragraph addLeft(String s, Font f, ReportRequest rr) {
		Paragraph p = new Paragraph(s,f);
		p.setAlignment(Element.ALIGN_LEFT);
		
		String temp = s.replaceAll("\\s+", "");
		if (temp.indexOf(" ") == 0) temp = temp.substring(1);
		rr.addKeys( temp );
		
		return p;
	}

	private static void addTitlePage(Document document, ReportRequest rr) throws DocumentException {
		
		Paragraph preface = new Paragraph();
		
		// We add one empty line
		addEmptyLine(preface, 1);
		
		// Lets write a big header
		preface.add(addCenter("UC Berkeley", headFont));
		addEmptyLine(preface, 8);

		preface.add(addCenter("INTEGRATED CORRIDOR MANAGEMENT PROJECT", catFont));

		addEmptyLine(preface, 1);
		addEmptyLine(preface, 3);
		
		//Date
		
		java.util.Date today = new java.util.Date();	
		preface.add(addCenter("Report generated: " + new java.sql.Timestamp(today.getTime()), normalBold));
		
		addEmptyLine(preface, 3);	
		preface.add(addCenter("This Report Includes:", normalBold));
		addEmptyLine(preface, 1);
		preface.add(addCenter(dataIncluded("Network", rr.getDetailed(), false, rr.getNetworkPerformance()), smallBold));
		preface.add(addCenter(dataIncluded("Route", rr.getDetailed(), rr.getRouteData(), rr.getRoutePerformance()), smallBold));
		preface.add(addCenter(dataIncluded("Link", rr.getDetailed(), rr.getLinkData(), rr.getLinkPerformance()), smallBold));
		preface.add(addCenter(dataIncluded("Onramp", rr.getDetailed(), rr.getOnRampData(), rr.getOnRampPerformance()), smallBold));
		
		addEmptyLine(preface, 8);

		if ( rr.getUnits().equals("US") ) 
			preface.add(addCenter("Units: US", normalBold));
		else
			preface.add(addCenter("Units: Metric", normalBold));
		
		addEmptyLine(preface, 3);
		document.add(preface);
		
		// Create title slide
		Slide slide = rr.getPpt().createSlide();

        TextBox box1 = new TextBox();
        TextRun tr1 = box1.getTextRun();
        tr1.setRunType(TextHeaderAtom.CENTER_TITLE_TYPE);
        tr1.setText("INTEGRATED CORRIDOR MANAGEMENT PROJECT");
        box1.getTextRun().getRichTextRuns()[0].setFontName(slideFont);
        box1.setAnchor(new Rectangle(54, 78, 612, 115));
        slide.addShape(box1);

        TextBox box2 = new TextBox();
        TextRun tr2 = box2.getTextRun();
        tr2.setRunType(TextHeaderAtom.CENTRE_BODY_TYPE);
        
        String temp = (new java.sql.Timestamp(today.getTime())).toString();
        String date = temp.substring(0,temp.indexOf(' '));
        String time = temp.substring(temp.indexOf(" ")+1,temp.indexOf('.') );      		
        
        tr2.setText("Date: " + date + "  Time: " + time );
        box2.getTextRun().getRichTextRuns()[0].setFontSize(14);
        box2.getTextRun().getRichTextRuns()[0].setFontName(slideFont);
        box2.setAnchor(new Rectangle(108, 204, 504, 138));
        slide.addShape(box2);

        TextBox box3 = new TextBox();
        TextRun tr3 = box3.getTextRun();
        tr3.getRichTextRuns()[0].setFontSize(32);
        box3.setHorizontalAlignment(TextBox.AlignCenter);
        tr3.setText("UC Berkley");
        box3.getTextRun().getRichTextRuns()[0].setFontName(slideFont);
        box3.setAnchor(new Rectangle(206, 348, 310, 84));
        slide.addShape(box3);
        
        
	    HeadersFooters hdd = rr.getPpt().getSlideHeadersFooters();
        hdd.setSlideNumberVisible(true);
	
		// Start a new page
		document.newPage();
	}
	
	private static String dataIncluded(String type, Boolean det, Boolean data, Boolean per) {
		
		String s = new String();
		
		s = "";
		
		if ( !data && !per ) return s;
		
		if ( det ) 

			s += "Detailed ";
		
		s += (type + " ");
		
		if ( data && per )  {

			s += "Data and Performance";
		}
		else if ( data )
			s += "Data";
		else if ( per )
			s += "Performance";
		
		return s;
	}


/**
 * Add time series to the document
 * Networks
Data
Nothing
Performance
Plots and tables of time series of the following performance measures for each scenario network:
•	VMT – sum of VMT from network links (total and detailed);
•	VHT – sum of VHT from network links (total and detailed);
•	Delay – sum of delays from network links (total and detailed);
•	Productivity Loss – sum of Productivity Loss from network links (only total).

 * @param document
 * @param table
 * @param rr
 * @throws DocumentException
 * @throws TorqueException
 * @throws DataSetException
 * @throws IOException
 */

	private void addNetworkSection(Document document, String table, ReportRequest rr) throws DocumentException, TorqueException, DataSetException, IOException {
		
		String query;
		
		reportToStandard("NETWORK SECTION Contour plots for table: " +table);
		
		rr.setTitle(getSlideTitle("Network", table));
		
		// Get a list of keys 
		
		query =  getScenarioAndRunSelection(getAggregationSelection("select distinct " + getListOfKeys(table, "link_id") + " from "  + table, rr.getAggregation()) ,rr.getContent() );
		reportToStandard("Key query: " + query);
		
		ArrayList<String> listOfColumnNames;
		
		@SuppressWarnings("rawtypes")
		java.util.List listOfKeys = BasePeer.executeQuery(query);
		
		reportToStandard("Unique key combinations: " + listOfKeys.size());

		for (int i=0; i < listOfKeys.size(); i++ ) {
			
			Paragraph keys = new Paragraph();
			
			// Get starting time stamp			
			query =  setKeys(getAggregationSelection("SELECT MIN(ts) FROM " + table,rr.getAggregation()), table, (Record)listOfKeys.get(i), "link_id" );				
			long start = rr.getStartTimeInMilliseconds();					
			start += ((Record)BasePeer.executeQuery(query).get(0)).getValue(1).asTimestamp().getTime();
			
			// Get max time stamp for this report
			long stop = start +  rr.getDurationInMilliseconds();
			
			addKeyValuesToDocument(keys, table, (Record)listOfKeys.get(i), rr, "NETWORK", null);		
			
			listOfColumnNames = getAggregationColumns(table);
			
			// Form main select
			String columns = "ts, " + listToString( listOfColumnNames, "SUM");
			query =  setKeys(getAggregationSelection("SELECT " + columns + " FROM " + table,rr.getAggregation()), table, (Record)listOfKeys.get(i), "link_id" );				
		    query = setTimeInterval(query, start-1, stop);
		    query += " GROUP BY ts ORDER BY ts ASC";
		    
			reportToStandard("Query: " + query);
			
			@SuppressWarnings("rawtypes")
			java.util.List data = BasePeer.executeQuery(query);
			//AggregateData.reportToStandard("Size " + data.size() );
		

			// Add generated chart and table
			if (table == "link_data_total") {
				return;
		
			}
			else if (table == "link_data_detailed") {
				return;
			}
			else if (table == "link_performance_total") {
				createTimeSeriesChart(keys, new ArrayList<String>(Arrays.asList("vht")), listOfColumnNames, data, rr);
				createTimeSeriesChart(keys, new ArrayList<String>(Arrays.asList("vmt")), listOfColumnNames, data, rr);
				createTimeSeriesChart(keys, new ArrayList<String>(Arrays.asList("productivity_loss")), listOfColumnNames, data, rr);
				createTimeSeriesChart(keys, new ArrayList<String>(Arrays.asList("delay")), listOfColumnNames, data, rr);
				
				createTables(keys, new ArrayList<String>(Arrays.asList("vht", "vmt", "productivity_loss", "delay")), listOfColumnNames, data, rr);
			}
			else if (table == "link_performance_detailed") {
				createTimeSeriesChart(keys, new ArrayList<String>(Arrays.asList("vht")), listOfColumnNames, data, rr);
				createTimeSeriesChart(keys, new ArrayList<String>(Arrays.asList("vmt")), listOfColumnNames, data, rr);				
				createTimeSeriesChart(keys, new ArrayList<String>(Arrays.asList("delay")), listOfColumnNames, data, rr);
				
				createTables(keys, new ArrayList<String>(Arrays.asList("vht", "vmt", "delay")), listOfColumnNames, data, rr);

			}
			
			document.newPage();

		}
		
	}
	
	/**
	 * Add contour charts to the document
	 * @param document
	 * @param table
	 * @param rr
	 * @throws DocumentException
	 * @throws TorqueException
	 * @throws DataSetException
	 * @throws IOException
	 */
		@SuppressWarnings("unchecked")
		private void addRouteSection(Document document, String table, ReportRequest rr) throws DocumentException, TorqueException, DataSetException, IOException {
			
			String query;
			
			reportToStandard("ROUTE SECTION Time series for table: " +table);
			rr.setTitle(getSlideTitle("Route", table));
			
			// Get a list of routes 
			
			query =  getScenarioAndRunSelection(getAggregationSelection("SELECT DISTINCT link_id FROM " + table, rr.getAggregation()) ,rr.getContent() );
			String routeQuery = "SELECT DISTINCT route_id FROM route_links WHERE link_id IN (" + query + ")";
						
			reportToStandard("Key query: " + routeQuery);
			
			@SuppressWarnings("rawtypes")
			java.util.List listOfRoutes = BasePeer.executeQuery(routeQuery);
			
			int numberOfRoutes = listOfRoutes.size();			
			reportToStandard("Number of routes: " + numberOfRoutes);
			
			if ( numberOfRoutes < 1 ) numberOfRoutes = 1;

			ArrayList<String> listOfColumnNames;

			java.util.List<Record> linkData;
			
			for (int i=0; i < numberOfRoutes; i++ ) {
				
				Paragraph keys = new Paragraph();
				
				// Get a list of unique link ID in the right order
				if ( listOfRoutes.size() > 0 ) {
					
					String linkQuery = "SELECT link_id FROM route_links WHERE route_id=" + ((Record)listOfRoutes.get(i)).getValue(1).asString() + " ORDER BY link_order ASC";
					reportToStandard("Link query: " + linkQuery);
					
					linkData = BasePeer.executeQuery(linkQuery);
					
					reportToStandard("Route="+((Record)listOfRoutes.get(i)).getValue(1).asString() + " Links="+linkData.size());
					
				} else {
					
					String linkQuery = getScenarioAndRunSelection(getAggregationSelection("SELECT DISTINCT link_id FROM " + table, rr.getAggregation()) ,rr.getContent() );
					reportToStandard("Link query: " + linkQuery);
					
					linkData = BasePeer.executeQuery(linkQuery);
				}				

				if (linkData.size() == 0 ) continue;
				
				LengthToPixels conv = new LengthToPixels(linkData.size());
				if ( rr.getUnits().equals("US") )
					conv.setScale(toUS("length"));
				else
					conv.setScale(toMetric("length"));
			 
				for (int j=0; j<linkData.size(); j++) {
					conv.addElement( linkData.get(j).getValue(1).asLong() );
				}
				
				conv.setBoundaries();
				
				// Get unique keys for the current route
				query =  getScenarioAndRunSelection(getAggregationSelection("select distinct " + getListOfKeys(table, "link_id") + " from "  + table, rr.getAggregation()) ,rr.getContent() );
				query += " AND link_id IN (" + getLinks(linkData) + ")";
				
				reportToStandard("Key query: " + query);
				
				@SuppressWarnings("rawtypes")
				java.util.List listOfKeys = BasePeer.executeQuery(query);
				
				reportToStandard("Unique key combinations: " + listOfKeys.size());
				
				for ( int j=0; j<listOfKeys.size(); j++) {
				
					// Get starting time stamp	
					
					query =  setKeys(getAggregationSelection("SELECT MIN(ts) FROM " + table,rr.getAggregation()), table, (Record)listOfKeys.get(j), "link_id" );				
					query += " AND link_id IN (" + getLinks(linkData) + ")";
					
					reportToStandard("Timwstamp query: " + query);
					
					long start = rr.getStartTimeInMilliseconds();					
					start += ((Record)BasePeer.executeQuery(query).get(0)).getValue(1).asTimestamp().getTime();
					
					// Get max time stamp for this report
					long stop = start +  rr.getDurationInMilliseconds();
										
					String val;
					if ( listOfRoutes.size() > 0 )					
						val = ((Record)listOfRoutes.get(i)).getValue(1).asString();
			
					else
						val="0";
					
					addKeyValuesToDocument(keys, table, (Record)listOfKeys.get(j), rr, "ROUTE", val);
						
					listOfColumnNames = getAggregationColumns(table);
						
					// Form main select
					String columns = "ts, "  + listToString( listOfColumnNames, "SUM");
					query =  setKeys(getAggregationSelection("SELECT " + columns + " FROM " + table,rr.getAggregation()), table, (Record)listOfKeys.get(j), "link_id" );							
					query = setTimeInterval(query, start-1, stop);
					query += " AND link_id IN (" + getLinks(linkData) + ")";
					query += "  GROUP BY ts ORDER BY ts ASC";
					    
					reportToStandard("Query: " + query);
					
					@SuppressWarnings("unchecked")
					java.util.List<Record> data = BasePeer.executeQuery(query);
					//AggregateData.reportToStandard("Size " + data.size() );
	
					
					// Add generated chart and table
					if (table == "link_data_total") {
										
					}
					else if (table == "link_data_detailed") {
						
					}
					else if (table == "link_performance_total") {
						createTimeSeriesChart(keys, new ArrayList<String>(Arrays.asList("vht")), listOfColumnNames, data, rr);
						createTimeSeriesChart(keys, new ArrayList<String>(Arrays.asList("vmt")), listOfColumnNames, data, rr);
						createTimeSeriesChart(keys, new ArrayList<String>(Arrays.asList("productivity_loss")), listOfColumnNames, data, rr);
						createTimeSeriesChart(keys, new ArrayList<String>(Arrays.asList("delay")), listOfColumnNames, data, rr);
						
						createTables(keys, new ArrayList<String>(Arrays.asList("vht", "vmt", "productivity_loss", "delay")), listOfColumnNames, data, rr);

					}
					else if (table == "link_performance_detailed") {
						createTimeSeriesChart(keys, new ArrayList<String>(Arrays.asList("vht")), listOfColumnNames, data, rr);
						createTimeSeriesChart(keys, new ArrayList<String>(Arrays.asList("vmt")), listOfColumnNames, data, rr);
						createTimeSeriesChart(keys, new ArrayList<String>(Arrays.asList("productivity_loss")), listOfColumnNames, data, rr);
						createTimeSeriesChart(keys, new ArrayList<String>(Arrays.asList("delay")), listOfColumnNames, data, rr);
						
						createTables(keys, new ArrayList<String>(Arrays.asList("vht", "vmt", "productivity_loss", "delay")), listOfColumnNames, data, rr);

					}
								
					document.newPage();
				
				}
			}
			
		}
		
		/**
		 * Route performance section
		 * @param document
		 * @param table
		 * @param rr
		 * @throws DocumentException
		 * @throws TorqueException
		 * @throws DataSetException
		 * @throws IOException
		 */
		private void addRoutePerfomanceSection(Document document, String table, ReportRequest rr) throws DocumentException, TorqueException, DataSetException, IOException {
			
			String query;
			
			reportToStandard("ROUTE PERFORMANCE SECTION Contour plots for table: " +table);
			
			// Get a list of keys 
			
			query =  getScenarioAndRunSelection(getAggregationSelection("select distinct " + getListOfKeys(table) + " from "  + table, rr.getAggregation()) ,rr.getContent() );
			reportToStandard("Key query: " + query);
			
			ArrayList<String> listOfColumnNames;
			
			@SuppressWarnings("rawtypes")
			java.util.List listOfKeys = BasePeer.executeQuery(query);
			
			reportToStandard("Unique key combinations: " + listOfKeys.size());

			for (int i=0; i < listOfKeys.size(); i++ ) {
				
				Paragraph keys = new Paragraph();
				
				// Get starting time stamp			
				query =  setKeys(getAggregationSelection("SELECT MIN(ts) FROM " + table,rr.getAggregation()), table, (Record)listOfKeys.get(i) );				
				long start = rr.getStartTimeInMilliseconds();					
				start += ((Record)BasePeer.executeQuery(query).get(0)).getValue(1).asTimestamp().getTime();
				
				// Get max time stamp for this report
				long stop = start +  rr.getDurationInMilliseconds();
				
				addKeyValuesToDocument(keys, table, (Record)listOfKeys.get(i), rr, "ROUTE", getKeyValue(setKeys("", table, (Record)listOfKeys.get(i)), "route_id"));		
				
				listOfColumnNames = getAggregationColumns(table);
				
				// Form main select
				String columns = "ts, " + listToString( listOfColumnNames, "SUM");
				query =  setKeys(getAggregationSelection("SELECT " + columns + " FROM " + table,rr.getAggregation()), table, (Record)listOfKeys.get(i));				
			    query = setTimeInterval(query, start-1, stop);
			    query += " GROUP BY ts ORDER BY ts ASC";
			    
				reportToStandard("Query: " + query);
				
				@SuppressWarnings("rawtypes")
				java.util.List data = BasePeer.executeQuery(query);
				//AggregateData.reportToStandard("Size " + data.size() );
			
				rr.setMultiplier(1.0/60.0);
				// Add generated chart and table
				if (table == "route_performance_total") {
					
					createTimeSeriesChart(keys, new ArrayList<String>(Arrays.asList("travel_time")), listOfColumnNames, data, rr);
					
					createTables(keys, new ArrayList<String>(Arrays.asList("travel_time")), listOfColumnNames, data, rr);	
				}

				
				rr.setMultiplier(1.0);
			}
			
		}
		
	
	/**
	 * Add contour charts to the document
	 * @param document
	 * @param table
	 * @param rr
	 * @throws DocumentException
	 * @throws TorqueException
	 * @throws DataSetException
	 * @throws IOException
	 */
		@SuppressWarnings("unchecked")
		private void addContourPlots(Document document, String table, ReportRequest rr) throws DocumentException, TorqueException, DataSetException, IOException {
			
			String query;
			
			reportToStandard("ROUTE SECTION Contour plots for table: " +table);
			rr.setTitle(getSlideTitle("Route", table));
			
			// Get a list of keys 
			
			query =  getScenarioAndRunSelection(getAggregationSelection("SELECT DISTINCT link_id FROM " + table, rr.getAggregation()) ,rr.getContent() );
			String routeQuery = "SELECT DISTINCT route_id FROM route_links WHERE link_id IN (" + query + ")";
						
			reportToStandard("Key query: " + routeQuery);
			
			@SuppressWarnings("rawtypes")
			java.util.List listOfRoutes = BasePeer.executeQuery(routeQuery);
			
			int numberOfRoutes = listOfRoutes.size();			
			reportToStandard("Number of routes: " + numberOfRoutes);
			
			if ( numberOfRoutes < 1 ) numberOfRoutes = 1;

			ArrayList<String> listOfColumnNames;

			java.util.List<Record> linkData;
			
			for (int i=0; i < numberOfRoutes; i++ ) {
				
				
				// Get a list of unique link ID in the right order
				if ( listOfRoutes.size() > 0 ) {
					
					String linkQuery = "SELECT link_id FROM route_links WHERE route_id=" + ((Record)listOfRoutes.get(i)).getValue(1).asString() + " ORDER BY link_order ASC";
					reportToStandard("Link query: " + linkQuery);
					
					linkData = BasePeer.executeQuery(linkQuery);
					
					reportToStandard("Route="+((Record)listOfRoutes.get(i)).getValue(1).asString() + " Links="+linkData.size());
					
				} else {
					
					String linkQuery = getScenarioAndRunSelection(getAggregationSelection("SELECT DISTINCT link_id FROM " + table, rr.getAggregation()) ,rr.getContent() );
					reportToStandard("Link query: " + linkQuery);
					
					linkData = BasePeer.executeQuery(linkQuery);
				}				

				if (linkData.size() == 0 ) continue;
				
				LengthToPixels conv = new LengthToPixels(linkData.size());
				if ( rr.getUnits().equals("US") )
					conv.setScale(toUS("length"));
				else
					conv.setScale(toMetric("length"));
			 
				for (int j=0; j<linkData.size(); j++) {
					conv.addElement( linkData.get(j).getValue(1).asLong() );
				}
				
				conv.setBoundaries();
				
				// Get unique keys for the current route
				query =  getScenarioAndRunSelection(getAggregationSelection("select distinct " + getListOfKeys(table, "link_id") + " from "  + table, rr.getAggregation()) ,rr.getContent() );
				query += " AND link_id IN (" + getLinks(linkData) + ")";
				
				reportToStandard("Key query: " + query);
				
				@SuppressWarnings("rawtypes")
				java.util.List listOfKeys = BasePeer.executeQuery(query);
				
				reportToStandard("Unique key combinations: " + listOfKeys.size());
				
				for ( int j=0; j<listOfKeys.size(); j++) {
					
					Paragraph keys = new Paragraph();
				
					// Get starting time stamp	
					
					query =  setKeys(getAggregationSelection("SELECT MIN(ts) FROM " + table,rr.getAggregation()), table, (Record)listOfKeys.get(j), "link_id" );				
					query += " AND link_id IN (" + getLinks(linkData) + ")";
					
					reportToStandard("Timwstamp query: " + query);
					
					long start = rr.getStartTimeInMilliseconds();					
					start += ((Record)BasePeer.executeQuery(query).get(0)).getValue(1).asTimestamp().getTime();
					
					// Get max time stamp for this report
					long stop = start +  rr.getDurationInMilliseconds();
										
					String val;
					if ( listOfRoutes.size() > 0 )					
						val = ((Record)listOfRoutes.get(i)).getValue(1).asString();
			
					else
						val="0";
					
					addKeyValuesToDocument(keys, table, (Record)listOfKeys.get(j), rr, "ROUTE", val);
						
					listOfColumnNames = getAggregationColumns(table);
						
					// Form main select
					String columns = "ts, " + "link_id, " + listToString( listOfColumnNames, "SUM");
					query =  setKeys(getAggregationSelection("SELECT " + columns + " FROM " + table,rr.getAggregation()), table, (Record)listOfKeys.get(j), "link_id" );							
					query = setTimeInterval(query, start-1, stop);
					query += " AND link_id IN (" + getLinks(linkData) + ")";
					query += "  GROUP BY link_id, ts ORDER BY link_id ASC, ts ASC";
					
					query = query.replace(", SUM(free_flow_speed)", " ");
					
					reportToStandard("Query: " + query);
					
					@SuppressWarnings("unchecked")
					java.util.List<Record> data = BasePeer.executeQuery(query);
					//AggregateData.reportToStandard("Size " + data.size() );
	
					reportToStandard("Data size: " + data.size() );
					
					// Add generated chart and table
					if (table == "link_data_total") {
						createContourCharts(keys, new ArrayList<String>(Arrays.asList("in_flow", "out_flow", "speed","density")), listOfColumnNames, data, rr, conv);
				
					}
					else if (table == "link_data_detailed") {
						createContourCharts(keys, new ArrayList<String>(Arrays.asList("in_flow", "out_flow", "density")), listOfColumnNames, data, rr, conv);
					}
					else if (table == "link_performance_total") {
						createContourCharts(keys, new ArrayList<String>(Arrays.asList("vht","vmt","delay", "productivity_loss")), listOfColumnNames, data, rr, conv);
					}
					else if (table == "link_performance_detailed") {
						createContourCharts(keys, new ArrayList<String>(Arrays.asList("vht","vmt","delay")), listOfColumnNames, data, rr, conv);
					}
								
					document.newPage();
				
				}
			}
			
		}
		
	/**
	 * Get list of links for the quiry
	 * @param linkData
	 * @return
	 */
	public static String getLinks(java.util.List<Record> linkData) {
		
		String links = new String();
		links = "";
		
		for ( int i=0; i<linkData.size(); i++ )  {
			try {
				
				if ( i > 0 ) links += ",";
				links += ( linkData.get(i).getValue(1).asString());
				
			} catch (DataSetException e) {
				
				e.printStackTrace();
				return links;
			}
		}
		
		return links;
	}
	
/**
 * Add tables and time series charts to document
 * @param document
 * @param table
 * @param rr
 * @throws DocumentException
 * @throws TorqueException
 * @throws DataSetException
 * @throws IOException
 */
	private void addLinkSection(Document document, String table, ReportRequest rr) throws DocumentException, TorqueException, DataSetException, IOException {
		
		String query;
		
		reportToStandard("LINK SECTION for table: " +table);
		rr.setTitle(getSlideTitle("Link", table));
		
		// Get a list of keys 
		
		query =  getScenarioAndRunSelection(getAggregationSelection("select distinct " + getListOfKeys(table) + " from "  + table, rr.getAggregation()) ,rr.getContent() );

		ArrayList<String> listOfColumnNames;
		
		@SuppressWarnings("rawtypes")
		java.util.List listOfKeys = BasePeer.executeQuery(query);
		
		reportToStandard("Unique key combinations: " + listOfKeys.size());

		for (int i=0; i < listOfKeys.size(); i++ ) {
			
			Paragraph keys = new Paragraph();			
			
			// Get starting time stamp			
			query =  setKeys(getAggregationSelection("SELECT MIN(ts) FROM " + table,rr.getAggregation()), table, (Record)listOfKeys.get(i) );				
			long start = rr.getStartTimeInMilliseconds();					
			start += ((Record)BasePeer.executeQuery(query).get(0)).getValue(1).asTimestamp().getTime();
			
			// Get max time stamp for this report
			long stop = start +  rr.getDurationInMilliseconds();
			
			addKeyValuesToDocument(keys, table, (Record)listOfKeys.get(i), rr, "LINK", null);	
		
			//String link = getKeyValue(setKeys("", table, (Record)listOfKeys.get(i)), "link_id");
			
			listOfColumnNames = getAggregationColumns(table);
			
			// Form main select
			String columns = "ts, " + listToString( listOfColumnNames);
			query =  setKeys(getAggregationSelection("SELECT " + columns + " FROM " + table,rr.getAggregation()), table, (Record)listOfKeys.get(i) );				
		    query = setTimeInterval(query, start-1, stop);
		    query += " ORDER BY ts ASC";
		    
		    query = query.replace(", free_flow_speed"," ");
		    
			reportToStandard("Query: " + query);
			
			@SuppressWarnings("unchecked")
			java.util.List<Record> data = BasePeer.executeQuery(query);
			//AggregateData.reportToStandard("Size " + data.size() );
			
			// Add generated chart and table
			if (table == "link_data_total") {
				createTimeSeriesChart(keys, new ArrayList<String>(Arrays.asList("in_flow", "out_flow", "capasity")), listOfColumnNames, data, rr);
				createTimeSeriesChart(keys, new ArrayList<String>(Arrays.asList("speed", "free_flow_speed")), listOfColumnNames, data, rr);
				createTimeSeriesChart(keys, new ArrayList<String>(Arrays.asList("density")), listOfColumnNames, data, rr);	
				
				createTables(keys, new ArrayList<String>(Arrays.asList("in_flow", "out_flow", "capasity", "speed", "free_flow_speed", "density")), listOfColumnNames, data, rr);	
			}
			else if (table == "link_data_detailed") {
				createTimeSeriesChart(keys, new ArrayList<String>(Arrays.asList("in_flow", "out_flow")), listOfColumnNames, data, rr);
				createTimeSeriesChart(keys, new ArrayList<String>(Arrays.asList("speed")), listOfColumnNames, data, rr);
				createTimeSeriesChart(keys, new ArrayList<String>(Arrays.asList("density")), listOfColumnNames, data, rr);
				
				createTables(keys, new ArrayList<String>(Arrays.asList("in_flow", "out_flow", "speed", "density")), listOfColumnNames, data, rr);	
			}
			else if (table == "link_performance_total") {
				
				// This special piece of code is needed for the minimal travel time only
				String lengthQuery = "SELECT length FROM links WHERE length>0";
				lengthQuery += " AND id="+getKeyValue(setKeys("", table, (Record)listOfKeys.get(i)), "link_id");
				lengthQuery += " AND network_id="+getKeyValue(setKeys("", table, (Record)listOfKeys.get(i)), "network_id");
				reportToStandard("Length query: " + lengthQuery);	
				double length = ((Record) BasePeer.executeQuery(lengthQuery).get(0)).getValue(1).asDouble();
				rr.setLinkLength(length);
				
				createTimeSeriesChart(keys, new ArrayList<String>(Arrays.asList("vht")), listOfColumnNames, data, rr);
				createTimeSeriesChart(keys, new ArrayList<String>(Arrays.asList("vmt")), listOfColumnNames, data, rr);
				createTimeSeriesChart(keys, new ArrayList<String>(Arrays.asList("productivity_loss")), listOfColumnNames, data, rr);
				createTimeSeriesChart(keys, new ArrayList<String>(Arrays.asList("travel_time", "minimal_time")), listOfColumnNames, data, rr);
				createTimeSeriesChart(keys, new ArrayList<String>(Arrays.asList("delay")), listOfColumnNames, data, rr);
//				createTimeSeriesChart(keys, new ArrayList<String>(Arrays.asList("los")), listOfColumnNames, data, rr);
//				createTimeSeriesChart(keys, new ArrayList<String>(Arrays.asList("vc_ratio")), listOfColumnNames, data, rr);
				
				createTables(keys, new ArrayList<String>(Arrays.asList("vht", "vmt", "productivity_loss", "travel_time", "minimal_time", "delay")), listOfColumnNames, data, rr);	

			
			}
			else if (table == "link_performance_detailed") {
				createTimeSeriesChart(keys, new ArrayList<String>(Arrays.asList("vht")), listOfColumnNames, data, rr);
				createTimeSeriesChart(keys, new ArrayList<String>(Arrays.asList("vmt")), listOfColumnNames, data, rr);
				createTimeSeriesChart(keys, new ArrayList<String>(Arrays.asList("delay")), listOfColumnNames, data, rr);	
				
				createTables(keys, new ArrayList<String>(Arrays.asList("vht", "vmt", "delay")), listOfColumnNames, data, rr);	
			}
					

		}
		
	}
	
	/**
	 * Add tables and time series charts to document for Onramp Section
	 * @param document
	 * @param table
	 * @param rr
	 * @throws DocumentException
	 * @throws TorqueException
	 * @throws DataSetException
	 * @throws IOException
	 */
		private void addOnrampSection(Document document, String table, ReportRequest rr) throws DocumentException, TorqueException, DataSetException, IOException {
			
			String query;
			
			reportToStandard("ONRAMP SECTION for table: " + table);
			rr.setTitle(getSlideTitle("Onramp", table));
			
			// Get a list of keys 
			
			@SuppressWarnings("unchecked")
			java.util.List<Record> linkType = BasePeer.executeQuery("SELECT id FROM link_types WHERE description=\'onramp\'");
			String onrampType = linkType.get(0).getValue(1).asString();
			
			query =  getScenarioAndRunSelection(getAggregationSelection("select distinct " 
					+ getListOfKeys(table) 
					+ " from "  
					+ table, rr.getAggregation()) ,rr.getContent() );

			query += " AND link_id IN (SELECT link_id FROM link_type_det WHERE link_type_id=" + onrampType + ")";
			ArrayList<String> listOfColumnNames;
			reportToStandard("Link query: " + query);
			@SuppressWarnings("rawtypes")
			
			java.util.List listOfKeys = BasePeer.executeQuery(query);

			for (int i=0; i < listOfKeys.size(); i++ ) {
				
				Paragraph keys = new Paragraph();
							
				// Get starting time stamp			
				query =  setKeys(getAggregationSelection("SELECT MIN(ts) FROM " + table,rr.getAggregation()), table, (Record)listOfKeys.get(i) );				
				long start = rr.getStartTimeInMilliseconds();					
				start += ((Record)BasePeer.executeQuery(query).get(0)).getValue(1).asTimestamp().getTime();
				
				// Get max time stamp for this report
				long stop = start +  rr.getDurationInMilliseconds();
				
				addKeyValuesToDocument(keys, table, (Record)listOfKeys.get(i), rr, "ONRAMP", null);	
				
				listOfColumnNames = getAggregationColumns(table);
				
				// Form main select
				String columns = "ts, " + listToString( listOfColumnNames);
				query =  setKeys(getAggregationSelection("SELECT " + columns + " FROM " + table,rr.getAggregation()), table, (Record)listOfKeys.get(i) );				
			    query = setTimeInterval(query, start-1, stop);
			    query += " ORDER BY ts ASC";
			    query = query.replace(", free_flow_speed"," ");
			    
				reportToStandard("Query: " + query);
				
				@SuppressWarnings("unchecked")
				java.util.List<Record> data = BasePeer.executeQuery(query);
				//AggregateData.reportToStandard("Size " + data.size() );
				
				// Add generated chart and table
				if (table == "link_data_total") {
					listOfColumnNames.set(listOfColumnNames.indexOf("density"), "queue size");
					createTimeSeriesChart(keys, new ArrayList<String>(Arrays.asList("queue size")), listOfColumnNames, data, rr);
					
					createTables(keys, new ArrayList<String>(Arrays.asList("queue size")), listOfColumnNames, data, rr);	
				}
				else if (table == "link_data_detailed") {
					listOfColumnNames.set(listOfColumnNames.indexOf("density"), "queue size");
					createTimeSeriesChart(keys, new ArrayList<String>(Arrays.asList("queue size")), listOfColumnNames, data, rr);				
				
					createTables(keys, new ArrayList<String>(Arrays.asList("queue size")), listOfColumnNames, data, rr);
				}
				else if (table == "link_performance_total") {
					createTimeSeriesChart(keys, new ArrayList<String>(Arrays.asList("vht")), listOfColumnNames, data, rr);
				
					createTables(keys, new ArrayList<String>(Arrays.asList("vht")), listOfColumnNames, data, rr);
				}
				else if (table == "link_performance_detailed") {
					createTimeSeriesChart(keys, new ArrayList<String>(Arrays.asList("vht")), listOfColumnNames, data, rr);
					
					createTables(keys, new ArrayList<String>(Arrays.asList("vht")), listOfColumnNames, data, rr);
				}
							

			}
			
		}
		

		/**
		 * Extract parameter value from the given string of parameters
		 * @param s
		 * @param key
		 * @return
		 */
		public static String getKeyValue(String s, String key) {
			
			int pos = s.indexOf(key);
			String res = new String();
			
			if (pos >=0 ) {
				res = s.substring(pos+key.length());
				pos = res.indexOf("=");
				if (pos >=0) res = res.substring(pos+1);
				pos = res.indexOf(" ");
				if ( pos >= 0 )
					res = res.substring(0, pos);
				return res;
				
			} else 
				return null;
		}
		
		
	/**
	 * remove ANDs from the key string
	 * @param s
	 * @return formatted string
	 */
	public static String formatKeys(String s) {
		

		if ( s.indexOf("WHERE") >= 0 ) {
			
			s = s.replace("WHERE", "");
		}
		
		if ( s.indexOf("AND ") >= 0 ) {
			
			return s.replace("AND", "\n");
		}
			
		return s;
	}
	
	/**
	 * Get scenario name from AppRunId
	 * @param table
	 * @param rec
	 * @param section
	 * @return
	 */
	public static String getScenarioName(String table, Record rec, String section) {
		
		String scenarioId;
		String name = new String();
		
		String scenarioQuery = "SELECT scenario_id FROM simulation_runs WHERE id=";
		if ( section == "ROUTE" || section == "NETWORK" )
			scenarioQuery += getKeyValue(setKeys("", table, rec, "link_id"), "app_run_id");
		else
			scenarioQuery += getKeyValue(setKeys("", table, rec), "app_run_id");
		
		try {
			@SuppressWarnings("unchecked")
			java.util.List<Record>  scenarioData =  BasePeer.executeQuery(scenarioQuery);
			
			if ( scenarioData.size()==0 ) return "Undefined Scenario";
			
			try {
				
				scenarioId = scenarioData.get(0).getValue(1).asString();
				String nameQuery = "SELECT name, description FROM scenarios WHERE id=" + scenarioId;
				
				@SuppressWarnings("unchecked")
				java.util.List<Record> nameData = BasePeer.executeQuery(nameQuery);
				
				if ( nameData.size() > 0 ) {
					
					name = ((Record)nameData.get(0)).getValue(1).asString();
					
					if  ( name == null || name.indexOf("null") >= 0 ) {						
						
							name = ((Record)nameData.get(0)).getValue(2).asString();
							
							if  ( name == null || name.indexOf("null") >= 0 ) 	name = scenarioId;

					}
					
				} else 
					name = "Undefined Scenario";
				
			} catch (DataSetException e) {
				
				e.printStackTrace();
				return "Undefined Scenario";
			}
			
		} catch (TorqueException e) {
			
			e.printStackTrace();
			return "Undefined Scenario";
		}
		
		return name;
	}
	
	/**
	 * Add values of each key to the document 
	 * @param keys
	 * @param table
	 * @param rec
	 * @param rr
	 */
	public static void addKeyValuesToDocument(Paragraph keys, String table, Record rec, ReportRequest rr, String section, String routeValue) {
		
		keys.add(addLeft(section + " SECTION                         TABLE: " + table.toUpperCase(), subFont, rr));
		rr.setKeys(null);
				
		if ( section == "NETWORK") {
			
			keys.add( addLeft(" Scenario    = " + getScenarioName(table, rec, section) + "\n", subFont, rr) );
			keys.add( addLeft(" Run #        = " + getKeyName("app_run_id", getKeyValue(setKeys("", table, rec, "link_id"), "app_run_id"))+"\n", subFont, rr) );
			keys.add( addLeft(" Network     = " + getKeyName("network_id", getKeyValue(setKeys("", table, rec, "link_id"), "network_id"))+"\n", subFont, rr) );
			keys.add( addLeft(" Application = " +  getKeyName("app_type_id", getKeyValue(setKeys("", table, rec, "link_id"), "app_type_id"))+"\n", subFont, rr) );
			keys.add( addLeft(" Value Type = " +   getKeyName("value_type_id", getKeyValue(setKeys("", table, rec, "link_id"), "value_type_id"))+"\n", subFont, rr) );
			keys.add( addLeft(" Aggregation= " +   rr.getAggregation(), subFont, rr ) );
			addEmptyLine(keys, 1);
			
		} else if  ( section == "LINK" || section == "ONRAMP" ) {
			
			keys.add( addLeft(" Scenario    = " + getScenarioName(table, rec, section) + "\n", subFont, rr) );
			keys.add( addLeft(" Run #        = "+getKeyName("app_run_id", getKeyValue(setKeys("", table, rec), "app_run_id"))+"\n", subFont, rr) );
			keys.add( addLeft(" Network     = "+getKeyName("network_id", getKeyValue(setKeys("", table, rec), "network_id"))+"\n", subFont, rr) );
			keys.add( addLeft(" Link           = "+getKeyName("link_id", getKeyValue(setKeys("", table, rec), "link_id"))+"\n", subFont, rr) );
			keys.add( addLeft(" Application = "+getKeyName("app_type_id", getKeyValue(setKeys("", table, rec), "app_type_id"))+"\n", subFont, rr) );
			keys.add( addLeft(" Value Type = "+getKeyName("value_type_id", getKeyValue(setKeys("", table, rec), "value_type_id"))+"\n", subFont, rr) );
			keys.add( addLeft(" Aggregation= "+rr.getAggregation(), subFont, rr ) );
			addEmptyLine(keys, 1);
			
		} else if  ( section == "ROUTE" ) {
			
			keys.add( addLeft(" Scenario     = " + getScenarioName(table, rec, section) + "\n", subFont, rr) );
			keys.add( addLeft(" Run #        = "+getKeyName("app_run_id", getKeyValue(setKeys("", table, rec, "link_id"), "app_run_id"))+"\n", subFont, rr) );
			keys.add( addLeft(" Route          = "+getKeyName("route_id", routeValue)+"\n", subFont, rr) );
			keys.add( addLeft(" Application = "+getKeyName("app_type_id", getKeyValue(setKeys("", table, rec, "link_id"), "app_type_id"))+"\n", subFont, rr) );
			keys.add( addLeft(" Value Type = "+getKeyName("value_type_id", getKeyValue(setKeys("", table, rec, "link_id"), "value_type_id"))+"\n", subFont, rr) );
			keys.add( addLeft(" Aggregation= "+rr.getAggregation(), subFont, rr ) );
			addEmptyLine(keys, 1);
		}
		
		return;
		
	}
	
	
	/**
	 * Return name of a key id
	 * @param key - key type
	 * @param val - key id value
	 * @return
	 */
	@SuppressWarnings("unchecked")
	public static String getKeyName(String key, String val) {
		
		String name = new String();
		String nameQuery;
		java.util.List<Record> nameData;
		name = "";
		
		nameQuery = "SELECT name FROM networks WHERE id=xxx";
		
		if ( key == "value_type_id" ) return val;
		
		if ( key == "app_run_id" ) 
			
			nameQuery = "SELECT run_number FROM simulation_runs WHERE id=" + val;
					
		 else if ( key == "network_id" ) 
			
			nameQuery = "SELECT name FROM networks WHERE id=" + val;
		
		 else if ( key == "app_type_id" ) 
				
				nameQuery = "SELECT description FROM application_types WHERE id=" + val;
		
		 else if ( key == "route_id" ) 
			
			nameQuery = "SELECT name FROM routes WHERE id=" + val;
		
		 else if ( key == "link_id" )
		
			nameQuery = "SELECT name FROM link_names WHERE link_id=" + val;
		
		 else if ( key == "scenario_id" ) {
			
			 
			nameQuery = "SELECT name, description FROM scenarios WHERE id=" + val;
		 }

			try {
			
			nameData = BasePeer.executeQuery(nameQuery);
			
			} catch (TorqueException e) {
			e.printStackTrace();
			return val;
			}

			if ( nameData.size() > 0 ) {
			
				try {
				
					if ( key == "link_id" ) {
					
						for( int i=0; i<nameData.size(); i++)
							name += ((Record)nameData.get(i)).getValue(1).asString() + " ";		
					
					} else
					
							name = ((Record)nameData.get(0)).getValue(1).asString();
				
				if  ( name == null || name.indexOf("null") >= 0 ) {
					
					if (key == "scenario_id") {
						
						name = ((Record)nameData.get(0)).getValue(2).asString();
						if  ( name == null || name.indexOf("null") >= 0 ) name = val;
						
					} else
						name = val;
				}
				
			} catch (DataSetException e) {
				
				e.printStackTrace();
				return val;
			}
			

				
		} else
			name =" Unspecified";
		 			
		
		return name;
	}

	/**
	 * Add a contour chart to the document
	 * @param useTheseColumns
	 * @param listOfColumnNames
	 * @param data
	 * @param rr
	 * @return
	 * @throws DataSetException
	 * @throws DocumentException
	 * @throws IOException
	 */
	private void createContourCharts(Paragraph keys, 
									ArrayList<String> useTheseColumns, 
									ArrayList<String> listOfColumnNames, 
									java.util.List data, 
									ReportRequest rr,
									LengthToPixels conv)
			throws DataSetException, DocumentException, IOException {

		
		for (int i=0; i<useTheseColumns.size(); i++) {
			String name = useTheseColumns.get(i);
			
			// show keys on top of the page
			document.add(keys);
			
			// add chart
			document.add(createContourChart(name,listOfColumnNames, data, rr, conv));
			
			// Start a new page
			document.newPage();
		}
		
		return;
	}
	/**
	 * Add a contour chart to the document
	 * @param useTheseColumns
	 * @param listOfColumnNames
	 * @param data
	 * @param rr
	 * @return
	 * @throws DataSetException
	 * @throws DocumentException
	 * @throws IOException
	 */
	private static Paragraph createContourChart(String name, 
												ArrayList<String> listOfColumnNames, 
												java.util.List data, 
												ReportRequest rr,
												LengthToPixels conv)
			throws DataSetException, DocumentException, IOException {

		Paragraph section = new Paragraph();
		
		reportToStandard("Contour Chart: "+ name);

		int colunmNumber =3;
		double  unitMultiplier = 1.0;
		
		if (data.size() == 0 ) return section;	// Return empty if no data to report		

		colunmNumber = listOfColumnNames.indexOf(name) + 3;
		
		if ( rr.getUnits().equals("US") )
			unitMultiplier = toUS(name);
		else
			unitMultiplier = toMetric(name);	

		// Get first time stamp in milliseconds
		// ts must be the first in the column list
		
		long startOfTheChart = ((Record)data.get(0)).getValue(1).asTimestamp().getTime();
		//startOfTheChart -= getAggregationInMilliseconds(rr.getAggregation());

		
		ContourChart cc = new ContourChart(name);
		cc.setItemCount(0);
		cc.setCurrentItem(0);

		
		double min = ((Record)data.get(0)).getValue(colunmNumber).asBigDecimal().doubleValue();
		double max = ((Record)data.get(0)).getValue(colunmNumber).asBigDecimal().doubleValue();
		
		for (int row=0; row< data.size(); row++) {
			
			long t = ((Record)data.get(row)).getValue(1).asTimestamp().getTime();

			BigDecimal d = ((Record)data.get(row)).getValue(colunmNumber).asBigDecimal();
			
			
			Long linkId = ((Record)data.get(row)).getValue(2).asLong();
			
			if ( d != null ) {
				
				// Convert to the right units
				d = BigDecimal.valueOf(d.doubleValue()*unitMultiplier);
				//d = new BigDecimal(row);
				
				// Add to the chart 
				conv.setXYZ(cc, linkId, ((double)(t - startOfTheChart ))/1000.0/60.0/60.0, d.doubleValue());
				
				if ( min > d.doubleValue() ) min = d.doubleValue();
				if ( max < d.doubleValue() ) max = d.doubleValue();

			}

		}
		
		cc.setMax(max);
		cc.setMin(min); 	
		
		section.add(addCenter("CONTOUR CHART: " + name, subFont));
		addEmptyLine(section, 1);
		
		JFreeChart chart;
	    String fileName;
	    
	    fileName = "chart" + rr.getChartId() + ".png";
	    rr.incrementChartId();
		java.io.File chartFile = new File(fileName);
		//chart = cc.createChart(cc.createDataset());
		chart = cc.createFreeChart(cc.createDataset(), null, "Time, Hours",  min,  max, rr);
		
		ChartUtilities.saveChartAsPNG(chartFile, /*1.0f,*/ chart, 900, 600);
		
		Image chartImage = Image.getInstance(fileName);
		
		chartImage.setAlignment(Element.ALIGN_CENTER);
		
		chartImage.scalePercent(60.0f);
		
		section.add(chartImage);
		addChartToSlide(fileName, rr);
		
		addEmptyLine(section, 1);
		
		return section;

	}

	/**
	 * Add tables to the document
	 * @param useTheseColumns
	 * @param listOfColumnNames
	 * @param data
	 * @param rr
	 * @return
	 * @throws DataSetException
	 * @throws DocumentException
	 * @throws IOException
	 */
	private void createTables(Paragraph keys, ArrayList<String> useTheseColumns, ArrayList<String> listOfColumnNames, java.util.List data, ReportRequest rr)
			throws DataSetException, DocumentException, IOException {		
	

		document.newPage();
		
		for ( int i=0; i<useTheseColumns.size(); i = i + 6 ) {
		
			ArrayList<String>  list = new ArrayList<String> ();

			for( int j=i; j<i+6 && j < useTheseColumns.size() ; j++)
				list.add(useTheseColumns.get(j));
			// Add keys
			document.add(keys);
			// Add table
			document.add(createTable(list, listOfColumnNames, data, rr));
			// Start a new page
			document.newPage();
		}
		
		return;
	}
	
	/**
	 * Add table to the document
	 * @param useTheseColumns
	 * @param listOfColumnNames
	 * @param data
	 * @param rr
	 * @return
	 * @throws DataSetException
	 * @throws DocumentException
	 * @throws IOException
	 */
	private static Paragraph createTable(ArrayList<String> useTheseColumns, ArrayList<String> listOfColumnNames, java.util.List data, ReportRequest rr)
			throws DataSetException, DocumentException, IOException {

		Paragraph section = new Paragraph();
		
		if ( useTheseColumns == null ) return section;
		
		PdfPTable table = new PdfPTable(useTheseColumns.size()+1);
		int[] colunmNumber = new int[useTheseColumns.size()+1];
		double[] unitMultiplier = new double[useTheseColumns.size()];
		float[] tableWidth = new float[useTheseColumns.size()+1];
		
		if (data.size() == 0 ) return section;	// Return empty if no data to report
		
		tableWidth[0]= 80f;
		colunmNumber[0] = 1; 	// this is to indicate the position of the time stamp in the select statement results 
								// ts must be at the first position
								// select result record data numbering starts at 1 , not zero
		
		for (int i=0; i<useTheseColumns.size(); i++) {
			
			tableWidth[i+1] = 40f;
			
			colunmNumber[i+1] = listOfColumnNames.indexOf(useTheseColumns.get(i)) + 2;
			if ( rr.getUnits().equals("US") )
				unitMultiplier[i] = toUS(useTheseColumns.get(i));
			else
				unitMultiplier[i] = toMetric(useTheseColumns.get(i));
			
			if ( useTheseColumns.get(i).equals("travel_time") ) 
				unitMultiplier[i] *= rr.getMultiplier();
			
			
		}
		
		table.setTotalWidth(tableWidth);

		table.addCell(new PdfPCell(new Phrase("Time Stamp",tableFont)));

		for (int i=0; i<useTheseColumns.size(); i++ ) {
			
			String name = useTheseColumns.get(i);
			table.addCell(new PdfPCell(new Phrase(name,tableFont)));

		}

		table.setHeaderRows(1);

		// Get first time stamp in milliseconds
		// ts must be the first in the column list
		
		// Service minimal time if needed
		int minimalTime = useTheseColumns.indexOf("minimal_time");
		int delay = listOfColumnNames.indexOf("delay");
		int vht = listOfColumnNames.indexOf("vht");
		int vmt = listOfColumnNames.indexOf("vmt");
		boolean minimalTimeFlag = false;
		if ( delay >= 0 && vht >= 0 && vmt >=0 && minimalTime >=0 ) minimalTimeFlag = true;
		
		int maxRows = 17;
		int currentRow =0;
		String dd[][] = new String[maxRows][useTheseColumns.size()+1];
		
		for (int row=0; row< data.size(); row++) {
			
			String temp = ((Record)data.get(row)).getValue(1).asString().toLowerCase();
			table.addCell(new PdfPCell(new Phrase( temp ,subFont)));
			//long t = ((Record)data.get(row)).getValue(1).asTimestamp().getTime();			
			dd[currentRow][0] = temp;
			
			for (int i=0; i<useTheseColumns.size(); i++ ) {
				
				BigDecimal d;
				
				if ( i == minimalTime ) {
					
					if ( minimalTimeFlag ) {
						
						double del	= 	((Record)data.get(row)).getValue(delay + 2).asDouble();
						double vh 	= 	((Record)data.get(row)).getValue(vht + 2).asDouble();
						double vm 	= 	((Record)data.get(row)).getValue(vmt + 2).asDouble();
						double length = rr.getLinkLength();
						
						if ( vm > 1E-6 )
							d = BigDecimal.valueOf( (vh-del)/vm*length );
						else
							d = BigDecimal.valueOf(0.0);
					}
					else
						d = BigDecimal.valueOf(0.0);
				} else
				
					d= ((Record)data.get(row)).getValue(colunmNumber[i+1]).asBigDecimal();
				
				if ( d == null ) {
					
					temp = " ";	
					
				} else {	
					
					// Convert to the right units
					d = BigDecimal.valueOf(d.doubleValue()*unitMultiplier[i]);
				
					// Add to the table
					if (d.doubleValue() > 999.99 ) 
						d=d.setScale(3,BigDecimal.ROUND_HALF_UP);
					else 
						d=d.setScale(6,BigDecimal.ROUND_HALF_UP);
					
					temp = d.toString();
					

				}
				
				table.addCell(new PdfPCell(new Phrase(temp,tableFont)));
				dd[currentRow][i+1] = temp;
						
			}	
			
			if ( ++currentRow == maxRows ) {
				
				currentRow = 0;
				addTableToSlide(maxRows, useTheseColumns.size()+1, useTheseColumns, dd, rr);
			}
		}
		
		if ( currentRow > 0 )
			addTableToSlide(currentRow, useTheseColumns.size()+1, useTheseColumns, dd, rr);
			
		section.add(addCenter("DATA:" + " " + listToString(useTheseColumns), subFont));
		addEmptyLine(section, 1);
		section.add(table);
		addEmptyLine(section, 1);
		return section;

	}
	
	private static void addTableToSlide(int rows, int cols, ArrayList<String> names, String[][] data, ReportRequest rr) {
		
		 Table table = new Table(rows+1, cols);
		 // Column names
		 TableCell ts = table.getCell(0, 0);
         ts.setText("time stamp");
         RichTextRun rt1 = ts.getTextRun().getRichTextRuns()[0];
         rt1.setFontName(slideFont);
         rt1.setFontSize(10);
         ts.setVerticalAlignment(TextBox.AnchorMiddle);
         ts.setHorizontalAlignment(TextBox.AlignCenter);
         
		 for (int j=0; j< names.size(); j++) {
	        TableCell cell = table.getCell(0, j+1);
            cell.setText(names.get(j));

            RichTextRun rt = cell.getTextRun().getRichTextRuns()[0];
            rt.setFontName(slideFont);
            rt.setFontSize(10);
            cell.setVerticalAlignment(TextBox.AnchorMiddle);
            cell.setHorizontalAlignment(TextBox.AlignCenter);
		 }
		 
		 // Data
	      for (int i = 0; i < rows; i++) {
	          for (int j = 0; j < cols; j++) {
	              TableCell cell = table.getCell(i+1, j);
	              cell.setText(data[i][j]);

	              RichTextRun rt = cell.getTextRun().getRichTextRuns()[0];
	              rt.setFontName(slideFont);
	              rt.setFontSize(10);

	              cell.setVerticalAlignment(TextBox.AnchorMiddle);
	              cell.setHorizontalAlignment(TextBox.AlignCenter);
	          }
	      }

	      //set table borders
	      Line border = table.createBorder();
	      border.setLineColor(Color.black);
	      border.setLineWidth(1.0);
	      table.setAllBorders(border);

	      //set width of the 1st column
	      table.setColumnWidth(0, 110);
	      table.setRowHeight(0, 20);
	      
	      //set width of the remaining columns
	      for (int i=1; i<cols; i++) 	    	  
	    	  table.setColumnWidth(i, 90);
	      
	      for (int i=0; i<=rows; i++ )
	    	  table.setRowHeight(i, 20);

	      
	      // Add slide
		   Slide slide = rr.getPpt().createSlide();	    	    
		   TextBox box = new TextBox();
		   TextRun title = box.getTextRun();
		   title.setRunType(TextHeaderAtom.TITLE_TYPE);
		   title.setText(rr.getTitle() + " Data");
		   title.getRichTextRuns()[0].setFontSize(24);
		   title.getRichTextRuns()[0].setFontSize(24);
	       box.setAnchor(new Rectangle(36, 20, 648, 50));
	       slide.addShape(box);
	        
	       TextBox box1 = new TextBox();
		   TextRun keys = box1.getTextRun();
		   keys.setRunType(TextHeaderAtom.TITLE_TYPE);
		   keys.setText(rr.getKeys());
		   keys.getRichTextRuns()[0].setFontSize(14);
	       box1.setAnchor(new Rectangle(30, 55, 630, 90));
	       slide.addShape(box1);

	       slide.addShape(table);
	       	       
	       table.moveTo((7-cols)*45+30, 116);

	}

	/**
	 * Add time series chart to the document
	 * @param useTheseColumns
	 * @param listOfColumnNames
	 * @param data
	 * @param rr
	 * @return
	 * @throws DataSetException
	 * @throws DocumentException
	 * @throws IOException
	 */
	private void createTimeSeriesChart(Paragraph keys, ArrayList<String> useTheseColumns, ArrayList<String> listOfColumnNames, java.util.List data, ReportRequest rr)
			throws DataSetException, DocumentException, IOException {

		XYSeriesCollection chartData = new XYSeriesCollection();
		java.util.List<XYSeries> curves = new ArrayList<XYSeries>();
		Paragraph section = new Paragraph();
		
		int[] colunmNumber = new int[useTheseColumns.size()+1];
		double[] unitMultiplier = new double[useTheseColumns.size()];

		
		if (data.size() == 0 ) return;	// Return empty if no data to report
		section.add(keys);
		
		colunmNumber[0] = 1; 	// this is to indicate the position of the time stamp in the select statement results 
								// ts must be at the first position
								// select result record data numbering starts at 1 , not zero
		
		for (int i=0; i<useTheseColumns.size(); i++) {
	
			colunmNumber[i+1] = listOfColumnNames.indexOf(useTheseColumns.get(i)) + 2;
			if ( rr.getUnits().equals("US") )
				unitMultiplier[i] = toUS(useTheseColumns.get(i));
			else
				unitMultiplier[i] = toMetric(useTheseColumns.get(i));
			
			if ( useTheseColumns.get(i).equals("travel_time") ) 
				unitMultiplier[i] *= rr.getMultiplier();
			
		}

		for (int i=0; i<useTheseColumns.size(); i++ ) {
			
			String name = useTheseColumns.get(i);
			curves.add(i,new XYSeries(name) );
		}

		// Get first time stamp in milliseconds
		// ts must be the first in the column list
		
		long startOfTheChart = ((Record)data.get(0)).getValue(1).asTimestamp().getTime();
		startOfTheChart -= getAggregationInMilliseconds(rr.getAggregation());


		// Service minimal time if needed
		int minimalTime = useTheseColumns.indexOf("minimal_time");
		int delay = listOfColumnNames.indexOf("delay");
		int vht = listOfColumnNames.indexOf("vht");
		int vmt = listOfColumnNames.indexOf("vmt");
		boolean minimalTimeFlag = false;
		if ( delay >= 0 && vht >= 0 && vmt >=0 && minimalTime >=0 ) minimalTimeFlag = true;
		
		for (int row=0; row< data.size(); row++) {
			
			long t = ((Record)data.get(row)).getValue(1).asTimestamp().getTime();
			
			for (int i=0; i<useTheseColumns.size(); i++ ) {
				
				BigDecimal d;
				if ( i == minimalTime ) {
					
					if ( minimalTimeFlag ) {
						
						double del	= 	((Record)data.get(row)).getValue(delay + 2).asDouble();
						double vh 	= 	((Record)data.get(row)).getValue(vht + 2).asDouble();
						double vm 	= 	((Record)data.get(row)).getValue(vmt + 2).asDouble();
						double length = rr.getLinkLength();
						
						if ( vm > 1E-6 )
							d = BigDecimal.valueOf( (vh-del)/vm*length );
						else
							d = BigDecimal.valueOf(0.0);
					}
					else
						d = BigDecimal.valueOf(0.0);
				}
					
				else {
					
					d= ((Record)data.get(row)).getValue(colunmNumber[i+1]).asBigDecimal();					
				}
				
				if ( d != null ) {
					
					// Convert to the right units
					d = BigDecimal.valueOf(d.doubleValue()*unitMultiplier[i]);
					
					// Add to the chart 
					curves.get(i).add(((double)(t - startOfTheChart))/1000.0/60.0/60.0, d.doubleValue());

				}
						
			}	
		}
		
		for (int i=0; i<useTheseColumns.size(); i++ ) {
			
			chartData.addSeries(curves.get(i));
		}
		
		section.add(addCenter("TIME SERIES CHART:" + " " + listToString(useTheseColumns), subFont));
		addEmptyLine(section, 1);
		section.add(createChart(chartData, useTheseColumns.size(), rr));
		addEmptyLine(section, 1);
		addEmptyLine(section, 1);
		
		document.add(section);

		document.newPage();
		
		return;

	}


	/**
	 * create chart
	 * @param section
	 * @param chartData
	 * @throws IOException 
	 * @throws BadElementException 
	 */
	private static Paragraph createChart(XYSeriesCollection chartData, int nSeries, ReportRequest rr) throws IOException, BadElementException {
		
		Paragraph section = new Paragraph();
		
		// Generate the chart
		JFreeChart chart = ChartFactory.createXYLineChart(
		null, // Title
		"Time, Hours", // x-axis Label
		" ", // y-axis Label
		chartData, // Data
		PlotOrientation.VERTICAL, // Plot Orientation
		true, // Show Legend
		true, // Use tooltips
		false // Configure chart to generate URLs?
		);
	       
	        chart.getXYPlot().setBackgroundPaint(Color.WHITE); // Change background to white
	        chart.getXYPlot().setDomainGridlinePaint(Color.LIGHT_GRAY); // Change grid line color
	        chart.getXYPlot().setOutlineStroke(new BasicStroke(0.3f, BasicStroke.CAP_ROUND, BasicStroke.JOIN_MITER, 50, new float[] { 1f, 2f }, 0));
	        chart.getXYPlot().setRangeGridlinePaint(Color.LIGHT_GRAY);	        
	        chart.getXYPlot().getDomainAxis().setLabelFont( chart.getXYPlot().getDomainAxis().getLabelFont().deriveFont(new Float(12f)) );
	        chart.getXYPlot().getRenderer().setSeriesOutlineStroke(1,  new BasicStroke(10.0f, BasicStroke.CAP_ROUND, BasicStroke.JOIN_MITER, 50, new float[] { 1f, 2f }, 0));
	        chart.getXYPlot().getRangeAxis().setLabelFont( chart.getXYPlot().getRangeAxis().getLabelFont().deriveFont(new Float(14f)) );
	        chart.getXYPlot().getLegendItems().get(0).setLabelFont( chart.getXYPlot().getRangeAxis().getLabelFont().deriveFont(new Float(14f)) );
	                
	        chart.getXYPlot().getRenderer().setBaseOutlineStroke(new BasicStroke(4f));
	        chart.getXYPlot().getRenderer().setBaseStroke(new BasicStroke(4f));
	        chart.getXYPlot().getRenderer().setSeriesOutlineStroke(1, new BasicStroke(4f));
	        
	        Dimension dimension = new Dimension(6,6);
	        Shape shape = new Rectangle(dimension);
	        chart.getXYPlot().getRenderer(0).setSeriesShape(0, shape);
	        
	        // set a few custom plot features
	        XYPlot plot = (XYPlot) chart.getPlot();
	        Shape[] cross = DefaultDrawingSupplier.createStandardSeriesShapes();
	        //if ( rr.getColor() ) plot.setBackgroundPaint(new Color(0xf7ebdb));
	        plot.setDomainGridlinesVisible(true);
	        plot.setDomainGridlinePaint(Color.lightGray);
	        plot.setRangeGridlinePaint(Color.lightGray);
	        XYItemRenderer renderer = (XYItemRenderer) plot.getRenderer();
	        renderer.setSeriesShape(0, cross[0]);
	      
	        renderer.setSeriesItemLabelsVisible(1, true);
	        
	        if ( !rr.getColor() ) {
	        	
	     
		        
		        
		        for ( int i=0; i<nSeries; i++)  {
		        	
		        	if ( i==1 ) {
		        		
		        		Stroke dash = new BasicStroke(1.0f, // Width
			                    BasicStroke.CAP_SQUARE,    	// End cap
			                    BasicStroke.JOIN_ROUND,    	// Join style
			                    10.0f,                     	// Miter limit
			                    new float[] {2.0f,4.0f}, 	// Dash pattern
			                    0.0f);                     	// Dash phase
				        
				        renderer.setSeriesStroke(1,dash);	
		        	} else
		        	if ( i==2 ) {
		        		
		        		Stroke dash = new BasicStroke(1.0f, // Width
			                    BasicStroke.CAP_SQUARE,    	// End cap
			                    //BasicStroke.JOIN_MITER,    	
			                    BasicStroke.JOIN_ROUND,    	// Join style
			                    10.0f,                     	// Miter limit
			                    new float[] {6.0f,8.0f}, 	// Dash pattern
			                    0.0f);                     	// Dash phase
				        
				        renderer.setSeriesStroke(1,dash);	
		        	}
		        	
		        	renderer.setSeriesPaint(i, Color.BLACK);
		        }
	        
	        }
	        
	        renderer.setSeriesItemLabelsVisible(1, true);
	        plot.setRenderer(renderer);
	        
	    String fileName;
	    fileName = "chart" + rr.getChartId() + ".png";
	    rr.incrementChartId();
		java.io.File chartFile = new File(fileName);
		
		org.jfree.chart.ChartUtilities.saveChartAsPNG(chartFile, /*1.0f,*/ chart, 900, 600);	
			
		Image chartImage = Image.getInstance(fileName);
		
		chartImage.setAlignment(Element.ALIGN_CENTER);
		
		chartImage.scalePercent(60.0f);
		
		section.add(chartImage);
		addChartToSlide(fileName, rr);

		return section;
		
        
	}
	
	
	/**
	 * Add chart image to the slide show
	 * @param fileName
	 * @param rr
	 * @throws IOException
	 */
	public static void addChartToSlide(String fileName, ReportRequest rr) throws IOException {
		
		// Add slide
	    Slide slide = rr.getPpt().createSlide();	    	    
	    TextBox box = new TextBox();
	    TextRun title = box.getTextRun();
	    title.setRunType(TextHeaderAtom.TITLE_TYPE);
	    title.setText(rr.getTitle() + " Chart");
	    title.getRichTextRuns()[0].setFontSize(24);
        box.setAnchor(new Rectangle(36, 20, 648, 50));
        slide.addShape(box);
        
        TextBox box1 = new TextBox();
	    TextRun keys = box1.getTextRun();
	    keys.setRunType(TextHeaderAtom.TITLE_TYPE);
	    keys.setText(rr.getKeys());
	    keys.getRichTextRuns()[0].setFontSize(14);
        box1.setAnchor(new Rectangle(30, 55, 630, 90));
        slide.addShape(box1);
        
        // Add the chart as a picture
        int idx = rr.getPpt().addPicture(new File(fileName), Picture.PNG);

        Picture pict = new Picture(idx);

        //Set image position in the slide
        pict.setAnchor(new java.awt.Rectangle(30, 100, 630, 400));
        slide.addShape(pict);
		
	}

	private static void addEmptyLine(Paragraph paragraph, int number) {
		for (int i = 0; i < number; i++) {
			paragraph.add(new Paragraph(" "));
		}
	}
	
	/**
	 * return a Unit conversion multiplier from database units to metric units
	 * @param name
	 * @return
	 */
	public static double toMetric(String name) {
		
		if ( name.equals("in_flow")) 	return 3600.0; 			// vehicle/hour
		if ( name.equals("out_flow")) 	return 3600.0; 			// vehicle/hour
		if ( name.equals("density")) 	return 1000.0; 			// vehicle/km
		if ( name.equals("jam_density"))return 1000.0; 			// vehicle/km
		
		if ( name.equals("speed")) 					return 3600.0/1000.0; 	// km/hour		
		if ( name.equals("free_flow_speed")) 		return 3600.0/1000.0; 	// km/hour
		if ( name.equals("critical_speed")) 		return 3600.0/1000.0; 	// km/hour
		if ( name.equals("congestion_wave_speed")) 	return 3600.0/1000.0; 	// km/hour
		
		if ( name.equals("capasity")) 		return 3600.0; 	// vehicle/hour
		if ( name.equals("capasity_drop")) 	return 3600.0; 	// vehicle/hour
		
		if ( name.equals("vht")) 	return 1.0/3600.0; 	// vehicle*hour
		if ( name.equals("vmt") ) 	return 1.0/1000.0; 	// vehicle*km
		if ( name.equals("delay") ) return 1.0/3600.0; 	// vehicle*hour
		
		if ( name.equals("travel_time") ) 		return 1.0; 			// seconds
		if ( name.equals("productivity_loss") )	return 1.0/3600.0/1000.0; 	// lane*km*hour
		if ( name.equals("vc_ratio") ) 			return 1.0/1000.0; 			// km/vehicle
		if ( name == "length" )					return 1.0/1000.0; 			// km
		
		return 1.0;
	}
	
	/**
	 * return a Unit conversion multiplier from database units to US units
	 * @param name
	 * @return
	 */
	public static double toUS(String name) {
		
		if ( name == "in_flow" ) 	return 3600.0; 			// vehicle/hour
		if ( name == "out_flow" ) 	return 3600.0; 			// vehicle/hour
		if ( name == "density" ) 	return 1000.0*1.609344;	// vehicle/mile
		if ( name == "jam_density" ) return 1000.0*1.609344;// vehicle/mile
		
		if ( name == "speed" ) 					return 3600.0/1000.0/1.609344; 	// mile/hour		
		if ( name == "free_flow_speed" ) 		return 3600.0/1000.0/1.609344; 	// mile/hour
		if ( name == "critical_speed" ) 		return 3600.0/1000.0/1.609344; 	// mile/hour
		if ( name == "congestion_wave_speed" ) 	return 3600.0/1000.0/1.609344; 	// mile/hour
		
		if ( name == "capasity" ) 		return 3600.0; 	// vehicle/hour
		if ( name == "capasity_drop" ) 	return 3600.0; 	// vehicle/hour
		
		if ( name == "vht" ) 	return 1.0/3600.0; 			// vehicle*hour
		if ( name == "vmt" ) 	return 1.0/1000.0/1.609344;	// vehicle*mile
		if ( name == "delay" ) 	return 1.0/3600.0; 			// vehicle*hour
		
		if ( name == "travel_time" ) 		return 1.0; 						// seconds
		if ( name == "productivity_loss" ) 	return 1.0/3600.0/1000.0/1.609344; 	// lane*mile*hour
		if ( name == "vc_ratio" ) 			return 1.0/1000.0/1.609344;			// mile/hour; 			// mile/vehicle
		if ( name == "length" )				return 1.0/1000.0/1.609344; 	// miles	
		
		return 1.0;
	}
	
}
