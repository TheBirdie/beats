//
// This file was generated by the JavaTM Architecture for XML Binding(JAXB) Reference Implementation, v2.2.4 
// See <a href="http://java.sun.com/xml/jaxb">http://java.sun.com/xml/jaxb</a> 
// Any modifications to this file will be lost upon recompilation of the source schema. 
// Generated on: 2012.02.28 at 12:55:28 PM PST 
//


package com.relteq.sirius.jaxb;

import javax.xml.bind.annotation.XmlAccessType;
import javax.xml.bind.annotation.XmlAccessorType;
import javax.xml.bind.annotation.XmlAttribute;
import javax.xml.bind.annotation.XmlElement;
import javax.xml.bind.annotation.XmlRootElement;
import javax.xml.bind.annotation.XmlType;


/**
 * <p>Java class for anonymous complex type.
 * 
 * <p>The following schema fragment specifies the expected content contained within this class.
 * 
 * <pre>
 * &lt;complexType>
 *   &lt;complexContent>
 *     &lt;restriction base="{http://www.w3.org/2001/XMLSchema}anyType">
 *       &lt;all>
 *         &lt;element ref="{}From"/>
 *         &lt;element ref="{}To"/>
 *         &lt;element ref="{}EncodedPolyline"/>
 *       &lt;/all>
 *       &lt;attribute name="avoidHighways" use="required" type="{http://www.w3.org/2001/XMLSchema}boolean" />
 *       &lt;attribute name="road_name" type="{http://www.w3.org/2001/XMLSchema}string" />
 *     &lt;/restriction>
 *   &lt;/complexContent>
 * &lt;/complexType>
 * </pre>
 * 
 * 
 */
@XmlAccessorType(XmlAccessType.FIELD)
@XmlType(name = "", propOrder = {

})
@XmlRootElement(name = "DirectionsCacheEntry")
public class DirectionsCacheEntry {

    @XmlElement(name = "From", required = true)
    protected From from;
    @XmlElement(name = "To", required = true)
    protected To to;
    @XmlElement(name = "EncodedPolyline", required = true)
    protected EncodedPolyline encodedPolyline;
    @XmlAttribute(name = "avoidHighways", required = true)
    protected boolean avoidHighways;
    @XmlAttribute(name = "road_name")
    protected String roadName;

    /**
     * Gets the value of the from property.
     * 
     * @return
     *     possible object is
     *     {@link From }
     *     
     */
    public From getFrom() {
        return from;
    }

    /**
     * Sets the value of the from property.
     * 
     * @param value
     *     allowed object is
     *     {@link From }
     *     
     */
    public void setFrom(From value) {
        this.from = value;
    }

    /**
     * Gets the value of the to property.
     * 
     * @return
     *     possible object is
     *     {@link To }
     *     
     */
    public To getTo() {
        return to;
    }

    /**
     * Sets the value of the to property.
     * 
     * @param value
     *     allowed object is
     *     {@link To }
     *     
     */
    public void setTo(To value) {
        this.to = value;
    }

    /**
     * Gets the value of the encodedPolyline property.
     * 
     * @return
     *     possible object is
     *     {@link EncodedPolyline }
     *     
     */
    public EncodedPolyline getEncodedPolyline() {
        return encodedPolyline;
    }

    /**
     * Sets the value of the encodedPolyline property.
     * 
     * @param value
     *     allowed object is
     *     {@link EncodedPolyline }
     *     
     */
    public void setEncodedPolyline(EncodedPolyline value) {
        this.encodedPolyline = value;
    }

    /**
     * Gets the value of the avoidHighways property.
     * 
     */
    public boolean isAvoidHighways() {
        return avoidHighways;
    }

    /**
     * Sets the value of the avoidHighways property.
     * 
     */
    public void setAvoidHighways(boolean value) {
        this.avoidHighways = value;
    }

    /**
     * Gets the value of the roadName property.
     * 
     * @return
     *     possible object is
     *     {@link String }
     *     
     */
    public String getRoadName() {
        return roadName;
    }

    /**
     * Sets the value of the roadName property.
     * 
     * @param value
     *     allowed object is
     *     {@link String }
     *     
     */
    public void setRoadName(String value) {
        this.roadName = value;
    }

}
