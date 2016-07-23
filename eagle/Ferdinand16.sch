<?xml version="1.0" encoding="utf-8"?>
<!DOCTYPE eagle SYSTEM "eagle.dtd">
<eagle version="7.5.0">
<drawing>
<settings>
<setting alwaysvectorfont="no"/>
<setting verticaltext="up"/>
</settings>
<grid distance="0.1" unitdist="inch" unit="inch" style="lines" multiple="1" display="no" altdistance="0.01" altunitdist="inch" altunit="inch"/>
<layers>
<layer number="1" name="Top" color="4" fill="1" visible="no" active="no"/>
<layer number="16" name="Bottom" color="1" fill="1" visible="no" active="no"/>
<layer number="17" name="Pads" color="2" fill="1" visible="no" active="no"/>
<layer number="18" name="Vias" color="2" fill="1" visible="no" active="no"/>
<layer number="19" name="Unrouted" color="6" fill="1" visible="no" active="no"/>
<layer number="20" name="Dimension" color="15" fill="1" visible="no" active="no"/>
<layer number="21" name="tPlace" color="15" fill="1" visible="no" active="no"/>
<layer number="22" name="bPlace" color="7" fill="1" visible="no" active="no"/>
<layer number="23" name="tOrigins" color="15" fill="1" visible="no" active="no"/>
<layer number="24" name="bOrigins" color="15" fill="1" visible="no" active="no"/>
<layer number="25" name="tNames" color="7" fill="1" visible="no" active="no"/>
<layer number="26" name="bNames" color="7" fill="1" visible="no" active="no"/>
<layer number="27" name="tValues" color="7" fill="1" visible="no" active="no"/>
<layer number="28" name="bValues" color="7" fill="1" visible="no" active="no"/>
<layer number="29" name="tStop" color="7" fill="3" visible="no" active="no"/>
<layer number="30" name="bStop" color="7" fill="6" visible="no" active="no"/>
<layer number="31" name="tCream" color="7" fill="4" visible="no" active="no"/>
<layer number="32" name="bCream" color="7" fill="5" visible="no" active="no"/>
<layer number="33" name="tFinish" color="6" fill="3" visible="no" active="no"/>
<layer number="34" name="bFinish" color="6" fill="6" visible="no" active="no"/>
<layer number="35" name="tGlue" color="7" fill="4" visible="no" active="no"/>
<layer number="36" name="bGlue" color="7" fill="5" visible="no" active="no"/>
<layer number="37" name="tTest" color="7" fill="1" visible="no" active="no"/>
<layer number="38" name="bTest" color="7" fill="1" visible="no" active="no"/>
<layer number="39" name="tKeepout" color="4" fill="11" visible="no" active="no"/>
<layer number="40" name="bKeepout" color="1" fill="11" visible="no" active="no"/>
<layer number="41" name="tRestrict" color="4" fill="10" visible="no" active="no"/>
<layer number="42" name="bRestrict" color="1" fill="10" visible="no" active="no"/>
<layer number="43" name="vRestrict" color="2" fill="10" visible="no" active="no"/>
<layer number="44" name="Drills" color="7" fill="1" visible="no" active="no"/>
<layer number="45" name="Holes" color="7" fill="1" visible="no" active="no"/>
<layer number="46" name="Milling" color="3" fill="1" visible="no" active="no"/>
<layer number="47" name="Measures" color="7" fill="1" visible="no" active="no"/>
<layer number="48" name="Document" color="7" fill="1" visible="no" active="no"/>
<layer number="49" name="Reference" color="7" fill="1" visible="no" active="no"/>
<layer number="50" name="dxf" color="7" fill="1" visible="no" active="no"/>
<layer number="51" name="tDocu" color="14" fill="1" visible="no" active="no"/>
<layer number="52" name="bDocu" color="7" fill="1" visible="no" active="no"/>
<layer number="53" name="tGND_GNDA" color="7" fill="9" visible="no" active="no"/>
<layer number="54" name="bGND_GNDA" color="1" fill="9" visible="no" active="no"/>
<layer number="56" name="wert" color="7" fill="1" visible="no" active="no"/>
<layer number="57" name="tCAD" color="7" fill="1" visible="no" active="no"/>
<layer number="59" name="tCarbon" color="7" fill="1" visible="no" active="no"/>
<layer number="60" name="bCarbon" color="7" fill="1" visible="no" active="no"/>
<layer number="90" name="Modules" color="5" fill="1" visible="yes" active="yes"/>
<layer number="91" name="Nets" color="2" fill="1" visible="yes" active="yes"/>
<layer number="92" name="Busses" color="1" fill="1" visible="yes" active="yes"/>
<layer number="93" name="Pins" color="2" fill="1" visible="no" active="yes"/>
<layer number="94" name="Symbols" color="4" fill="1" visible="yes" active="yes"/>
<layer number="95" name="Names" color="7" fill="1" visible="yes" active="yes"/>
<layer number="96" name="Values" color="7" fill="1" visible="yes" active="yes"/>
<layer number="97" name="Info" color="7" fill="1" visible="yes" active="yes"/>
<layer number="98" name="Guide" color="6" fill="1" visible="yes" active="yes"/>
<layer number="99" name="SpiceOrder" color="7" fill="1" visible="yes" active="yes"/>
<layer number="100" name="Muster" color="7" fill="1" visible="no" active="no"/>
<layer number="101" name="Patch_Top" color="12" fill="4" visible="yes" active="yes"/>
<layer number="102" name="Vscore" color="7" fill="1" visible="yes" active="yes"/>
<layer number="103" name="tMap" color="7" fill="1" visible="yes" active="yes"/>
<layer number="104" name="Name" color="16" fill="1" visible="yes" active="yes"/>
<layer number="105" name="tPlate" color="7" fill="1" visible="yes" active="yes"/>
<layer number="106" name="bPlate" color="7" fill="1" visible="yes" active="yes"/>
<layer number="107" name="Crop" color="7" fill="1" visible="yes" active="yes"/>
<layer number="108" name="tplace-old" color="10" fill="1" visible="yes" active="yes"/>
<layer number="109" name="ref-old" color="11" fill="1" visible="yes" active="yes"/>
<layer number="110" name="fp0" color="7" fill="1" visible="yes" active="yes"/>
<layer number="111" name="LPC17xx" color="7" fill="1" visible="yes" active="yes"/>
<layer number="112" name="tSilk" color="7" fill="1" visible="yes" active="yes"/>
<layer number="113" name="IDFDebug" color="7" fill="1" visible="yes" active="yes"/>
<layer number="114" name="Badge_Outline" color="7" fill="1" visible="yes" active="yes"/>
<layer number="115" name="ReferenceISLANDS" color="7" fill="1" visible="yes" active="yes"/>
<layer number="116" name="Patch_BOT" color="9" fill="4" visible="yes" active="yes"/>
<layer number="118" name="Rect_Pads" color="7" fill="1" visible="yes" active="yes"/>
<layer number="121" name="_tsilk" color="7" fill="1" visible="yes" active="yes"/>
<layer number="122" name="_bsilk" color="7" fill="1" visible="yes" active="yes"/>
<layer number="123" name="tTestmark" color="7" fill="1" visible="yes" active="yes"/>
<layer number="124" name="bTestmark" color="7" fill="1" visible="yes" active="yes"/>
<layer number="125" name="_tNames" color="7" fill="1" visible="yes" active="yes"/>
<layer number="126" name="_bNames" color="7" fill="1" visible="yes" active="yes"/>
<layer number="127" name="_tValues" color="7" fill="1" visible="yes" active="yes"/>
<layer number="128" name="_bValues" color="7" fill="1" visible="yes" active="yes"/>
<layer number="129" name="Mask" color="7" fill="1" visible="yes" active="yes"/>
<layer number="131" name="tAdjust" color="7" fill="1" visible="yes" active="yes"/>
<layer number="132" name="bAdjust" color="7" fill="1" visible="yes" active="yes"/>
<layer number="144" name="Drill_legend" color="7" fill="1" visible="yes" active="yes"/>
<layer number="150" name="Notes" color="7" fill="1" visible="yes" active="yes"/>
<layer number="151" name="HeatSink" color="7" fill="1" visible="yes" active="yes"/>
<layer number="152" name="_bDocu" color="7" fill="1" visible="yes" active="yes"/>
<layer number="153" name="FabDoc1" color="7" fill="1" visible="yes" active="yes"/>
<layer number="154" name="FabDoc2" color="7" fill="1" visible="yes" active="yes"/>
<layer number="155" name="FabDoc3" color="7" fill="1" visible="yes" active="yes"/>
<layer number="199" name="Contour" color="7" fill="1" visible="yes" active="yes"/>
<layer number="200" name="200bmp" color="1" fill="10" visible="yes" active="yes"/>
<layer number="201" name="201bmp" color="2" fill="10" visible="yes" active="yes"/>
<layer number="202" name="202bmp" color="3" fill="10" visible="yes" active="yes"/>
<layer number="203" name="203bmp" color="4" fill="10" visible="yes" active="yes"/>
<layer number="204" name="204bmp" color="5" fill="10" visible="yes" active="yes"/>
<layer number="205" name="205bmp" color="6" fill="10" visible="yes" active="yes"/>
<layer number="206" name="206bmp" color="7" fill="10" visible="yes" active="yes"/>
<layer number="207" name="207bmp" color="8" fill="10" visible="yes" active="yes"/>
<layer number="208" name="208bmp" color="9" fill="10" visible="yes" active="yes"/>
<layer number="209" name="209bmp" color="7" fill="1" visible="yes" active="yes"/>
<layer number="210" name="210bmp" color="7" fill="1" visible="yes" active="yes"/>
<layer number="211" name="211bmp" color="7" fill="1" visible="yes" active="yes"/>
<layer number="212" name="212bmp" color="7" fill="1" visible="yes" active="yes"/>
<layer number="213" name="213bmp" color="7" fill="1" visible="yes" active="yes"/>
<layer number="214" name="214bmp" color="7" fill="1" visible="yes" active="yes"/>
<layer number="215" name="215bmp" color="7" fill="1" visible="yes" active="yes"/>
<layer number="216" name="216bmp" color="7" fill="1" visible="yes" active="yes"/>
<layer number="217" name="217bmp" color="18" fill="1" visible="no" active="no"/>
<layer number="218" name="218bmp" color="19" fill="1" visible="no" active="no"/>
<layer number="219" name="219bmp" color="20" fill="1" visible="no" active="no"/>
<layer number="220" name="220bmp" color="21" fill="1" visible="no" active="no"/>
<layer number="221" name="221bmp" color="22" fill="1" visible="no" active="no"/>
<layer number="222" name="222bmp" color="23" fill="1" visible="no" active="no"/>
<layer number="223" name="223bmp" color="24" fill="1" visible="no" active="no"/>
<layer number="224" name="224bmp" color="25" fill="1" visible="no" active="no"/>
<layer number="225" name="225bmp" color="7" fill="1" visible="yes" active="yes"/>
<layer number="226" name="226bmp" color="7" fill="1" visible="yes" active="yes"/>
<layer number="227" name="227bmp" color="7" fill="1" visible="yes" active="yes"/>
<layer number="228" name="228bmp" color="7" fill="1" visible="yes" active="yes"/>
<layer number="229" name="229bmp" color="7" fill="1" visible="yes" active="yes"/>
<layer number="230" name="230bmp" color="7" fill="1" visible="yes" active="yes"/>
<layer number="231" name="231bmp" color="7" fill="1" visible="yes" active="yes"/>
<layer number="232" name="Eagle3D_PG2" color="7" fill="1" visible="no" active="no"/>
<layer number="233" name="Eagle3D_PG3" color="7" fill="1" visible="no" active="no"/>
<layer number="248" name="Housing" color="7" fill="1" visible="yes" active="yes"/>
<layer number="249" name="Edge" color="7" fill="1" visible="yes" active="yes"/>
<layer number="250" name="Descript" color="3" fill="1" visible="no" active="no"/>
<layer number="251" name="SMDround" color="12" fill="11" visible="no" active="no"/>
<layer number="254" name="cooling" color="7" fill="1" visible="yes" active="yes"/>
<layer number="255" name="routoute" color="7" fill="1" visible="yes" active="yes"/>
</layers>
<schematic xreflabel="%F%N/%S.%C%R" xrefpart="/%S.%C%R">
<libraries>
<library name="SparkFun-DigitalIC">
<description>&lt;h3&gt;SparkFun Electronics' preferred foot prints&lt;/h3&gt;
In this library you'll find all manner of digital ICs- microcontrollers, memory chips, logic chips, FPGAs, etc.&lt;br&gt;&lt;br&gt;
We've spent an enormous amount of time creating and checking these footprints and parts, but it is the end user's responsibility to ensure correctness and suitablity for a given componet or application. If you enjoy using this library, please buy one of our products at www.sparkfun.com.
&lt;br&gt;&lt;br&gt;
&lt;b&gt;Licensing:&lt;/b&gt; Creative Commons ShareAlike 4.0 International - https://creativecommons.org/licenses/by-sa/4.0/ 
&lt;br&gt;&lt;br&gt;
You are welcome to use this library for commercial purposes. For attribution, we ask that when you begin to sell your device using our footprint, you email us with a link to the product being sold. We want bragging rights that we helped (in a very small part) to create your 8th world wonder. We would like the opportunity to feature your device on our homepage.</description>
<packages>
<package name="DIL40_0.9">
<description>Dual inline 40 pin package for mbed microcontroller development platform.</description>
<wire x1="-25.4" y1="-13.97" x2="-25.4" y2="-3.81" width="0.127" layer="21"/>
<wire x1="-25.4" y1="-3.81" x2="-25.4" y2="3.81" width="0.127" layer="21"/>
<wire x1="-25.4" y1="3.81" x2="-25.4" y2="13.97" width="0.127" layer="21"/>
<wire x1="-25.4" y1="13.97" x2="27.94" y2="13.97" width="0.127" layer="21"/>
<wire x1="27.94" y1="13.97" x2="27.94" y2="-13.97" width="0.127" layer="21"/>
<wire x1="27.94" y1="-13.97" x2="-25.4" y2="-13.97" width="0.127" layer="21"/>
<wire x1="-25.4" y1="3.81" x2="-16.51" y2="3.81" width="0.127" layer="21"/>
<wire x1="-16.51" y1="3.81" x2="-16.51" y2="-3.81" width="0.127" layer="21"/>
<wire x1="-16.51" y1="-3.81" x2="-25.4" y2="-3.81" width="0.127" layer="21"/>
<pad name="1" x="-22.86" y="-11.43" drill="1" diameter="1.397" shape="octagon" rot="R90"/>
<pad name="2" x="-20.32" y="-11.43" drill="1" diameter="1.397" shape="octagon" rot="R90"/>
<pad name="3" x="-17.78" y="-11.43" drill="1" diameter="1.397" shape="octagon" rot="R90"/>
<pad name="4" x="-15.24" y="-11.43" drill="1" diameter="1.397" shape="octagon" rot="R90"/>
<pad name="5" x="-12.7" y="-11.43" drill="1" diameter="1.397" shape="octagon" rot="R90"/>
<pad name="6" x="-10.16" y="-11.43" drill="1" diameter="1.397" shape="octagon" rot="R90"/>
<pad name="7" x="-7.62" y="-11.43" drill="1" diameter="1.397" shape="octagon" rot="R90"/>
<pad name="8" x="-5.08" y="-11.43" drill="1" diameter="1.397" shape="octagon" rot="R90"/>
<pad name="9" x="-2.54" y="-11.43" drill="1" diameter="1.397" shape="octagon" rot="R90"/>
<pad name="10" x="0" y="-11.43" drill="1" diameter="1.397" shape="octagon" rot="R90"/>
<pad name="11" x="2.54" y="-11.43" drill="1" diameter="1.397" shape="octagon" rot="R90"/>
<pad name="12" x="5.08" y="-11.43" drill="1" diameter="1.397" shape="octagon" rot="R90"/>
<pad name="13" x="7.62" y="-11.43" drill="1" diameter="1.397" shape="octagon" rot="R90"/>
<pad name="14" x="10.16" y="-11.43" drill="1" diameter="1.397" shape="octagon" rot="R90"/>
<pad name="15" x="12.7" y="-11.43" drill="1" diameter="1.397" shape="octagon" rot="R90"/>
<pad name="16" x="15.24" y="-11.43" drill="1" diameter="1.397" shape="octagon" rot="R90"/>
<pad name="17" x="17.78" y="-11.43" drill="1" diameter="1.397" shape="octagon" rot="R90"/>
<pad name="18" x="20.32" y="-11.43" drill="1" diameter="1.397" shape="octagon" rot="R90"/>
<pad name="19" x="22.86" y="-11.43" drill="1" diameter="1.397" shape="octagon" rot="R90"/>
<pad name="20" x="25.4" y="-11.43" drill="1" diameter="1.397" shape="octagon" rot="R90"/>
<pad name="21" x="25.4" y="11.43" drill="1" diameter="1.397" shape="octagon" rot="R90"/>
<pad name="22" x="22.86" y="11.43" drill="1" diameter="1.397" shape="octagon" rot="R90"/>
<pad name="23" x="20.32" y="11.43" drill="1" diameter="1.397" shape="octagon" rot="R90"/>
<pad name="24" x="17.78" y="11.43" drill="1" diameter="1.397" shape="octagon" rot="R90"/>
<pad name="25" x="15.24" y="11.43" drill="1" diameter="1.397" shape="octagon" rot="R90"/>
<pad name="26" x="12.7" y="11.43" drill="1" diameter="1.397" shape="octagon" rot="R90"/>
<pad name="27" x="10.16" y="11.43" drill="1" diameter="1.397" shape="octagon" rot="R90"/>
<pad name="28" x="7.62" y="11.43" drill="1" diameter="1.397" shape="octagon" rot="R90"/>
<pad name="29" x="5.08" y="11.43" drill="1" diameter="1.397" shape="octagon" rot="R90"/>
<pad name="30" x="2.54" y="11.43" drill="1" diameter="1.397" shape="octagon" rot="R90"/>
<pad name="31" x="0" y="11.43" drill="1" diameter="1.397" shape="octagon" rot="R90"/>
<pad name="32" x="-2.54" y="11.43" drill="1" diameter="1.397" shape="octagon" rot="R90"/>
<pad name="33" x="-5.08" y="11.43" drill="1" diameter="1.397" shape="octagon" rot="R90"/>
<pad name="34" x="-7.62" y="11.43" drill="1" diameter="1.397" shape="octagon" rot="R90"/>
<pad name="35" x="-10.16" y="11.43" drill="1" diameter="1.397" shape="octagon" rot="R90"/>
<pad name="36" x="-12.7" y="11.43" drill="1" diameter="1.397" shape="octagon" rot="R90"/>
<pad name="37" x="-15.24" y="11.43" drill="1" diameter="1.397" shape="octagon" rot="R90"/>
<pad name="38" x="-17.78" y="11.43" drill="1" diameter="1.397" shape="octagon" rot="R90"/>
<pad name="39" x="-20.32" y="11.43" drill="1" diameter="1.397" shape="octagon" rot="R90"/>
<pad name="40" x="-22.86" y="11.43" drill="1" diameter="1.397" shape="octagon" rot="R90"/>
<text x="-25.4" y="-3.81" size="1.27" layer="25" rot="R90">&gt;NAME</text>
<text x="-2.54" y="-1.27" size="1.27" layer="27">&gt;VALUE</text>
</package>
</packages>
<symbols>
<symbol name="MBED_NXP_LPC1768">
<description>&lt;h3&gt;mbed NXP LPC1768&lt;/h3&gt;
&lt;p&gt;Microcontroller development board featuring an ARM cortex M3 and an online IDE with driverless USB programming.&lt;/p&gt;</description>
<wire x1="-33.02" y1="25.4" x2="-33.02" y2="-27.94" width="0.254" layer="94"/>
<wire x1="-33.02" y1="-27.94" x2="33.02" y2="-27.94" width="0.254" layer="94"/>
<wire x1="33.02" y1="-27.94" x2="33.02" y2="25.4" width="0.254" layer="94"/>
<wire x1="33.02" y1="25.4" x2="2.54" y2="25.4" width="0.254" layer="94"/>
<wire x1="2.54" y1="25.4" x2="-2.54" y2="25.4" width="0.254" layer="94"/>
<wire x1="-2.54" y1="25.4" x2="-33.02" y2="25.4" width="0.254" layer="94"/>
<wire x1="2.54" y1="25.4" x2="-2.54" y2="25.4" width="0.254" layer="94" curve="-180"/>
<text x="-33.02" y="26.67" size="1.778" layer="95">&gt;NAME</text>
<text x="-31.75" y="15.24" size="1.4224" layer="94">nR</text>
<text x="31.75" y="19.05" size="1.4224" layer="94" rot="R180">IF-</text>
<text x="31.75" y="16.51" size="1.4224" layer="94" rot="R180">IF+</text>
<text x="31.75" y="13.97" size="1.4224" layer="94" rot="R180">Ethernet RD-</text>
<text x="31.75" y="11.43" size="1.4224" layer="94" rot="R180">Ethernet RD+</text>
<text x="31.75" y="8.89" size="1.4224" layer="94" rot="R180">Ethernet TD-</text>
<text x="31.75" y="6.35" size="1.4224" layer="94" rot="R180">Ethernet TD+</text>
<text x="31.75" y="3.81" size="1.4224" layer="94" rot="R180">USB D-</text>
<text x="31.75" y="1.27" size="1.4224" layer="94" rot="R180">USB D+</text>
<text x="31.75" y="24.13" size="1.4224" layer="94" rot="R180">Vout</text>
<text x="31.75" y="21.59" size="1.4224" layer="94" rot="R180">Vu</text>
<text x="31.75" y="-24.13" size="1.4224" layer="94" rot="R180">p21 / PwmOut</text>
<text x="31.75" y="-21.59" size="1.4224" layer="94" rot="R180">p22 / PwmOut</text>
<text x="31.75" y="-19.05" size="1.4224" layer="94" rot="R180">p23 / PwmOut</text>
<text x="31.75" y="-16.51" size="1.4224" layer="94" rot="R180">p24 / PwmOut</text>
<text x="31.75" y="-13.97" size="1.4224" layer="94" rot="R180">p25 / PwmOut</text>
<text x="31.75" y="-11.43" size="1.4224" layer="94" rot="R180">p26 / PwmOut</text>
<text x="31.75" y="-8.89" size="1.4224" layer="94" rot="R180">p27 / I2C scl / Serial rx</text>
<text x="31.75" y="-6.35" size="1.4224" layer="94" rot="R180">p28 / I2C sda / Serial tx</text>
<text x="31.75" y="-3.81" size="1.4224" layer="94" rot="R180">p29 / CAN td</text>
<text x="31.75" y="-1.27" size="1.4224" layer="94" rot="R180">p30 / CAN rd</text>
<text x="-31.75" y="12.7" size="1.4224" layer="94">p5 / SPI mosi</text>
<text x="-31.75" y="10.16" size="1.4224" layer="94">p6 / SPI miso</text>
<text x="-31.75" y="7.62" size="1.4224" layer="94">p7 / SPI sck</text>
<text x="-31.75" y="5.08" size="1.4224" layer="94">p8</text>
<text x="-31.75" y="2.54" size="1.4224" layer="94">p9 / Serial tx / I2C sda</text>
<text x="-31.75" y="0" size="1.4224" layer="94">p10 / Serial rx / I2C scl</text>
<text x="-31.75" y="-2.54" size="1.4224" layer="94">p11 / SPI mosi</text>
<text x="-31.75" y="-5.08" size="1.4224" layer="94">p12 / SPI miso</text>
<text x="-31.75" y="-7.62" size="1.4224" layer="94">p13 / SPI SCK / Serial tx</text>
<text x="-31.75" y="-10.16" size="1.4224" layer="94">p14 / Serial rx</text>
<text x="-31.75" y="-12.7" size="1.4224" layer="94">p15 / AnalogIn</text>
<text x="-31.75" y="-15.24" size="1.4224" layer="94">p16 / AnalogIn</text>
<text x="-31.75" y="-17.78" size="1.4224" layer="94">p17 / AnalogIn</text>
<text x="-31.75" y="-20.32" size="1.4224" layer="94">p18 / AnalogIn / AnalogOut</text>
<text x="-31.75" y="-22.86" size="1.4224" layer="94">p19 / AnalogIn</text>
<text x="-31.75" y="-25.4" size="1.4224" layer="94">p20 / AnalogIn</text>
<text x="-25.4" y="17.78" size="1.6764" layer="94">mbed NXP LPC1768 Microcontroller</text>
<text x="-31.75" y="20.32" size="1.4224" layer="94">Vin</text>
<text x="-31.75" y="17.78" size="1.4224" layer="94">Vb</text>
<text x="-31.75" y="22.86" size="1.4224" layer="94">Gnd</text>
<pin name="P$1" x="-38.1" y="22.86" visible="off" length="middle"/>
<pin name="P$2" x="-38.1" y="20.32" visible="off" length="middle"/>
<pin name="P$3" x="-38.1" y="17.78" visible="off" length="middle"/>
<pin name="P$4" x="-38.1" y="15.24" visible="off" length="middle"/>
<pin name="P$5" x="-38.1" y="12.7" visible="off" length="middle"/>
<pin name="P$6" x="-38.1" y="10.16" visible="off" length="middle"/>
<pin name="P$7" x="-38.1" y="7.62" visible="off" length="middle"/>
<pin name="P$8" x="-38.1" y="5.08" visible="off" length="middle"/>
<pin name="P$9" x="-38.1" y="2.54" visible="off" length="middle"/>
<pin name="P$10" x="-38.1" y="0" visible="off" length="middle"/>
<pin name="P$11" x="-38.1" y="-2.54" visible="off" length="middle"/>
<pin name="P$12" x="-38.1" y="-5.08" visible="off" length="middle"/>
<pin name="P$13" x="-38.1" y="-7.62" visible="off" length="middle"/>
<pin name="P$14" x="-38.1" y="-10.16" visible="off" length="middle"/>
<pin name="P$15" x="-38.1" y="-12.7" visible="off" length="middle"/>
<pin name="P$16" x="-38.1" y="-15.24" visible="off" length="middle"/>
<pin name="P$17" x="-38.1" y="-17.78" visible="off" length="middle"/>
<pin name="P$18" x="-38.1" y="-20.32" visible="off" length="middle"/>
<pin name="P$19" x="-38.1" y="-22.86" visible="off" length="middle"/>
<pin name="P$20" x="-38.1" y="-25.4" visible="off" length="middle"/>
<pin name="P$21" x="38.1" y="-25.4" visible="off" length="middle" rot="R180"/>
<pin name="P$22" x="38.1" y="-22.86" visible="off" length="middle" rot="R180"/>
<pin name="P$23" x="38.1" y="-20.32" visible="off" length="middle" rot="R180"/>
<pin name="P$24" x="38.1" y="-17.78" visible="off" length="middle" rot="R180"/>
<pin name="P$25" x="38.1" y="-15.24" visible="off" length="middle" rot="R180"/>
<pin name="P$26" x="38.1" y="-12.7" visible="off" length="middle" rot="R180"/>
<pin name="P$27" x="38.1" y="-10.16" visible="off" length="middle" rot="R180"/>
<pin name="P$28" x="38.1" y="-7.62" visible="off" length="middle" rot="R180"/>
<pin name="P$29" x="38.1" y="-5.08" visible="off" length="middle" rot="R180"/>
<pin name="P$30" x="38.1" y="-2.54" visible="off" length="middle" rot="R180"/>
<pin name="P$31" x="38.1" y="0" visible="off" length="middle" rot="R180"/>
<pin name="P$32" x="38.1" y="2.54" visible="off" length="middle" rot="R180"/>
<pin name="P$33" x="38.1" y="5.08" visible="off" length="middle" rot="R180"/>
<pin name="P$34" x="38.1" y="7.62" visible="off" length="middle" rot="R180"/>
<pin name="P$35" x="38.1" y="10.16" visible="off" length="middle" rot="R180"/>
<pin name="P$36" x="38.1" y="12.7" visible="off" length="middle" rot="R180"/>
<pin name="P$37" x="38.1" y="15.24" visible="off" length="middle" rot="R180"/>
<pin name="P$38" x="38.1" y="17.78" visible="off" length="middle" rot="R180"/>
<pin name="P$39" x="38.1" y="20.32" visible="off" length="middle" rot="R180"/>
<pin name="P$40" x="38.1" y="22.86" visible="off" length="middle" rot="R180"/>
</symbol>
</symbols>
<devicesets>
<deviceset name="MBED_NXP_LPC1768">
<gates>
<gate name="G$1" symbol="MBED_NXP_LPC1768" x="0" y="0"/>
</gates>
<devices>
<device name="" package="DIL40_0.9">
<connects>
<connect gate="G$1" pin="P$1" pad="1"/>
<connect gate="G$1" pin="P$10" pad="10"/>
<connect gate="G$1" pin="P$11" pad="11"/>
<connect gate="G$1" pin="P$12" pad="12"/>
<connect gate="G$1" pin="P$13" pad="13"/>
<connect gate="G$1" pin="P$14" pad="14"/>
<connect gate="G$1" pin="P$15" pad="15"/>
<connect gate="G$1" pin="P$16" pad="16"/>
<connect gate="G$1" pin="P$17" pad="17"/>
<connect gate="G$1" pin="P$18" pad="18"/>
<connect gate="G$1" pin="P$19" pad="19"/>
<connect gate="G$1" pin="P$2" pad="2"/>
<connect gate="G$1" pin="P$20" pad="20"/>
<connect gate="G$1" pin="P$21" pad="21"/>
<connect gate="G$1" pin="P$22" pad="22"/>
<connect gate="G$1" pin="P$23" pad="23"/>
<connect gate="G$1" pin="P$24" pad="24"/>
<connect gate="G$1" pin="P$25" pad="25"/>
<connect gate="G$1" pin="P$26" pad="26"/>
<connect gate="G$1" pin="P$27" pad="27"/>
<connect gate="G$1" pin="P$28" pad="28"/>
<connect gate="G$1" pin="P$29" pad="29"/>
<connect gate="G$1" pin="P$3" pad="3"/>
<connect gate="G$1" pin="P$30" pad="30"/>
<connect gate="G$1" pin="P$31" pad="31"/>
<connect gate="G$1" pin="P$32" pad="32"/>
<connect gate="G$1" pin="P$33" pad="33"/>
<connect gate="G$1" pin="P$34" pad="34"/>
<connect gate="G$1" pin="P$35" pad="35"/>
<connect gate="G$1" pin="P$36" pad="36"/>
<connect gate="G$1" pin="P$37" pad="37"/>
<connect gate="G$1" pin="P$38" pad="38"/>
<connect gate="G$1" pin="P$39" pad="39"/>
<connect gate="G$1" pin="P$4" pad="4"/>
<connect gate="G$1" pin="P$40" pad="40"/>
<connect gate="G$1" pin="P$5" pad="5"/>
<connect gate="G$1" pin="P$6" pad="6"/>
<connect gate="G$1" pin="P$7" pad="7"/>
<connect gate="G$1" pin="P$8" pad="8"/>
<connect gate="G$1" pin="P$9" pad="9"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
</devices>
</deviceset>
</devicesets>
</library>
<library name="adam_custom">
<description>&lt;b&gt;Adam's Custom Parts&lt;/b&gt;</description>
<packages>
<package name="1X04">
<wire x1="6.985" y1="1.27" x2="8.255" y2="1.27" width="0.2032" layer="21"/>
<wire x1="8.255" y1="1.27" x2="8.89" y2="0.635" width="0.2032" layer="21"/>
<wire x1="8.89" y1="-0.635" x2="8.255" y2="-1.27" width="0.2032" layer="21"/>
<wire x1="3.81" y1="0.635" x2="4.445" y2="1.27" width="0.2032" layer="21"/>
<wire x1="4.445" y1="1.27" x2="5.715" y2="1.27" width="0.2032" layer="21"/>
<wire x1="5.715" y1="1.27" x2="6.35" y2="0.635" width="0.2032" layer="21"/>
<wire x1="6.35" y1="-0.635" x2="5.715" y2="-1.27" width="0.2032" layer="21"/>
<wire x1="5.715" y1="-1.27" x2="4.445" y2="-1.27" width="0.2032" layer="21"/>
<wire x1="4.445" y1="-1.27" x2="3.81" y2="-0.635" width="0.2032" layer="21"/>
<wire x1="6.985" y1="1.27" x2="6.35" y2="0.635" width="0.2032" layer="21"/>
<wire x1="6.35" y1="-0.635" x2="6.985" y2="-1.27" width="0.2032" layer="21"/>
<wire x1="8.255" y1="-1.27" x2="6.985" y2="-1.27" width="0.2032" layer="21"/>
<wire x1="-0.635" y1="1.27" x2="0.635" y2="1.27" width="0.2032" layer="21"/>
<wire x1="0.635" y1="1.27" x2="1.27" y2="0.635" width="0.2032" layer="21"/>
<wire x1="1.27" y1="-0.635" x2="0.635" y2="-1.27" width="0.2032" layer="21"/>
<wire x1="1.27" y1="0.635" x2="1.905" y2="1.27" width="0.2032" layer="21"/>
<wire x1="1.905" y1="1.27" x2="3.175" y2="1.27" width="0.2032" layer="21"/>
<wire x1="3.175" y1="1.27" x2="3.81" y2="0.635" width="0.2032" layer="21"/>
<wire x1="3.81" y1="-0.635" x2="3.175" y2="-1.27" width="0.2032" layer="21"/>
<wire x1="3.175" y1="-1.27" x2="1.905" y2="-1.27" width="0.2032" layer="21"/>
<wire x1="1.905" y1="-1.27" x2="1.27" y2="-0.635" width="0.2032" layer="21"/>
<wire x1="-1.27" y1="0.635" x2="-1.27" y2="-0.635" width="0.2032" layer="21"/>
<wire x1="-0.635" y1="1.27" x2="-1.27" y2="0.635" width="0.2032" layer="21"/>
<wire x1="-1.27" y1="-0.635" x2="-0.635" y2="-1.27" width="0.2032" layer="21"/>
<wire x1="0.635" y1="-1.27" x2="-0.635" y2="-1.27" width="0.2032" layer="21"/>
<wire x1="8.89" y1="0.635" x2="8.89" y2="-0.635" width="0.2032" layer="21"/>
<pad name="1" x="0" y="0" drill="1.016" diameter="1.8796" shape="square" rot="R90"/>
<pad name="2" x="2.54" y="0" drill="1.016" diameter="1.8796" rot="R90"/>
<pad name="3" x="5.08" y="0" drill="1.016" diameter="1.8796" rot="R90"/>
<pad name="4" x="7.62" y="0" drill="1.016" diameter="1.8796" rot="R90"/>
<text x="-1.3462" y="1.8288" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-1.27" y="-3.175" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="7.366" y1="-0.254" x2="7.874" y2="0.254" layer="51"/>
<rectangle x1="4.826" y1="-0.254" x2="5.334" y2="0.254" layer="51"/>
<rectangle x1="2.286" y1="-0.254" x2="2.794" y2="0.254" layer="51"/>
<rectangle x1="-0.254" y1="-0.254" x2="0.254" y2="0.254" layer="51"/>
</package>
<package name="1X04_NO_SILK">
<pad name="1" x="0" y="0" drill="1.016" diameter="1.8796" shape="square" rot="R90"/>
<pad name="2" x="2.54" y="0" drill="1.016" diameter="1.8796" rot="R90"/>
<pad name="3" x="5.08" y="0" drill="1.016" diameter="1.8796" rot="R90"/>
<pad name="4" x="7.62" y="0" drill="1.016" diameter="1.8796" rot="R90"/>
<text x="-1.3462" y="1.8288" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-1.27" y="-3.175" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="7.366" y1="-0.254" x2="7.874" y2="0.254" layer="51"/>
<rectangle x1="4.826" y1="-0.254" x2="5.334" y2="0.254" layer="51"/>
<rectangle x1="2.286" y1="-0.254" x2="2.794" y2="0.254" layer="51"/>
<rectangle x1="-0.254" y1="-0.254" x2="0.254" y2="0.254" layer="51"/>
</package>
</packages>
<symbols>
<symbol name="9DOF_SENSOR_STICK">
<wire x1="6.35" y1="0" x2="-2.54" y2="0" width="0.4064" layer="94"/>
<wire x1="3.81" y1="7.62" x2="5.08" y2="7.62" width="0.6096" layer="94"/>
<wire x1="3.81" y1="5.08" x2="5.08" y2="5.08" width="0.6096" layer="94"/>
<wire x1="3.81" y1="2.54" x2="5.08" y2="2.54" width="0.6096" layer="94"/>
<wire x1="-2.54" y1="12.7" x2="-2.54" y2="0" width="0.4064" layer="94"/>
<wire x1="6.35" y1="0" x2="6.35" y2="12.7" width="0.4064" layer="94"/>
<wire x1="-2.54" y1="12.7" x2="6.35" y2="12.7" width="0.4064" layer="94"/>
<wire x1="3.81" y1="10.16" x2="5.08" y2="10.16" width="0.6096" layer="94"/>
<text x="-2.54" y="-2.54" size="1.778" layer="96">&gt;VALUE</text>
<text x="-2.54" y="13.462" size="1.778" layer="95">&gt;NAME</text>
<pin name="1" x="10.16" y="2.54" visible="pad" length="middle" direction="pas" swaplevel="1" rot="R180"/>
<pin name="2" x="10.16" y="5.08" visible="pad" length="middle" direction="pas" swaplevel="1" rot="R180"/>
<pin name="3" x="10.16" y="7.62" visible="pad" length="middle" direction="pas" swaplevel="1" rot="R180"/>
<pin name="4" x="10.16" y="10.16" visible="pad" length="middle" direction="pas" swaplevel="1" rot="R180"/>
<text x="-2.032" y="7.62" size="1.778" layer="94" align="center-left">GND</text>
<text x="-2.032" y="5.08" size="1.778" layer="94" align="center-left">SDA</text>
<text x="-2.032" y="2.54" size="1.778" layer="94" align="center-left">SCL</text>
<text x="-2.032" y="10.16" size="1.778" layer="94" align="center-left">VCC</text>
</symbol>
<symbol name="DOCFIELD">
<wire x1="0" y1="0" x2="71.12" y2="0" width="0.254" layer="94"/>
<wire x1="101.6" y1="15.24" x2="87.63" y2="15.24" width="0.254" layer="94"/>
<wire x1="0" y1="0" x2="0" y2="5.08" width="0.254" layer="94"/>
<wire x1="0" y1="5.08" x2="71.12" y2="5.08" width="0.254" layer="94"/>
<wire x1="0" y1="5.08" x2="0" y2="15.24" width="0.254" layer="94"/>
<wire x1="101.6" y1="15.24" x2="101.6" y2="5.08" width="0.254" layer="94"/>
<wire x1="71.12" y1="5.08" x2="71.12" y2="0" width="0.254" layer="94"/>
<wire x1="71.12" y1="5.08" x2="87.63" y2="5.08" width="0.254" layer="94"/>
<wire x1="71.12" y1="0" x2="101.6" y2="0" width="0.254" layer="94"/>
<wire x1="87.63" y1="15.24" x2="87.63" y2="5.08" width="0.254" layer="94"/>
<wire x1="87.63" y1="15.24" x2="0" y2="15.24" width="0.254" layer="94"/>
<wire x1="87.63" y1="5.08" x2="101.6" y2="5.08" width="0.254" layer="94"/>
<wire x1="101.6" y1="5.08" x2="101.6" y2="0" width="0.254" layer="94"/>
<wire x1="0" y1="15.24" x2="0" y2="22.86" width="0.254" layer="94"/>
<wire x1="101.6" y1="35.56" x2="0" y2="35.56" width="0.254" layer="94"/>
<wire x1="101.6" y1="35.56" x2="101.6" y2="22.86" width="0.254" layer="94"/>
<wire x1="0" y1="22.86" x2="101.6" y2="22.86" width="0.254" layer="94"/>
<wire x1="0" y1="22.86" x2="0" y2="35.56" width="0.254" layer="94"/>
<wire x1="101.6" y1="22.86" x2="101.6" y2="15.24" width="0.254" layer="94"/>
<text x="1.27" y="1.27" size="2.54" layer="94" font="vector">Date:</text>
<text x="12.7" y="1.27" size="2.54" layer="94" font="vector">&gt;LAST_DATE_TIME</text>
<text x="72.39" y="1.27" size="2.54" layer="94" font="vector">Sheet:</text>
<text x="86.36" y="1.27" size="2.54" layer="94" font="vector">&gt;SHEET</text>
<text x="88.9" y="11.43" size="2.54" layer="94" font="vector">REV:</text>
<text x="1.524" y="17.78" size="2.54" layer="94" font="vector">TITLE:</text>
<text x="15.494" y="17.78" size="2.7432" layer="94" font="vector">&gt;DRAWING_NAME</text>
<text x="2.54" y="31.75" size="1.9304" layer="94">Released under the Creative Commons</text>
<text x="2.54" y="27.94" size="1.9304" layer="94">Attribution Share-Alike 4.0 License</text>
<text x="2.54" y="24.13" size="1.9304" layer="94"> https://creativecommons.org/licenses/by-sa/4.0/</text>
<text x="1.27" y="11.43" size="2.54" layer="94">Design by:</text>
<text x="19.05" y="11.43" size="2.54" layer="94">&gt;DESIGNER</text>
</symbol>
</symbols>
<devicesets>
<deviceset name="9DOF_SENSOR_STICK" prefix="J" uservalue="yes">
<description>Sparkfun 9DoF Sensor Stick.</description>
<gates>
<gate name="G$1" symbol="9DOF_SENSOR_STICK" x="2.54" y="2.54"/>
</gates>
<devices>
<device name="" package="1X04">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
<connect gate="G$1" pin="3" pad="3"/>
<connect gate="G$1" pin="4" pad="4"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="_NO_SILK" package="1X04_NO_SILK">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
<connect gate="G$1" pin="3" pad="3"/>
<connect gate="G$1" pin="4" pad="4"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
</devices>
</deviceset>
<deviceset name="CC-BY-SA-DOCFIELD">
<description>&lt;b&gt;Creative Commons Attribution-Share Alike 4.0 License Document Field&lt;/b&gt;&lt;p&gt;
Based on document field found in Sparkfun Eagle libraries here: https://github.com/sparkfun/SparkFun-Eagle-Libraries</description>
<gates>
<gate name="G$1" symbol="DOCFIELD" x="0" y="0"/>
</gates>
<devices>
<device name="">
<technologies>
<technology name=""/>
</technologies>
</device>
</devices>
</deviceset>
</devicesets>
</library>
<library name="SparkFun-Electromechanical">
<description>&lt;h3&gt;SparkFun Electronics' preferred foot prints&lt;/h3&gt;
In this library you'll find anything that moves- switches, relays, buttons, potentiometers. Also, anything that goes on a board but isn't electrical in nature- screws, standoffs, etc.&lt;br&gt;&lt;br&gt;
We've spent an enormous amount of time creating and checking these footprints and parts, but it is the end user's responsibility to ensure correctness and suitablity for a given componet or application. If you enjoy using this library, please buy one of our products at www.sparkfun.com.
&lt;br&gt;&lt;br&gt;
&lt;b&gt;Licensing:&lt;/b&gt; Creative Commons ShareAlike 4.0 International - https://creativecommons.org/licenses/by-sa/4.0/ 
&lt;br&gt;&lt;br&gt;
You are welcome to use this library for commercial purposes. For attribution, we ask that when you begin to sell your device using our footprint, you email us with a link to the product being sold. We want bragging rights that we helped (in a very small part) to create your 8th world wonder. We would like the opportunity to feature your device on our homepage.</description>
<packages>
<package name="TACTILE-PTH">
<description>&lt;b&gt;OMRON SWITCH&lt;/b&gt;</description>
<wire x1="3.048" y1="1.016" x2="3.048" y2="2.54" width="0.2032" layer="51"/>
<wire x1="3.048" y1="2.54" x2="2.54" y2="3.048" width="0.2032" layer="51"/>
<wire x1="2.54" y1="-3.048" x2="3.048" y2="-2.54" width="0.2032" layer="51"/>
<wire x1="3.048" y1="-2.54" x2="3.048" y2="-1.016" width="0.2032" layer="51"/>
<wire x1="-2.54" y1="3.048" x2="-3.048" y2="2.54" width="0.2032" layer="51"/>
<wire x1="-3.048" y1="2.54" x2="-3.048" y2="1.016" width="0.2032" layer="51"/>
<wire x1="-2.54" y1="-3.048" x2="-3.048" y2="-2.54" width="0.2032" layer="51"/>
<wire x1="-3.048" y1="-2.54" x2="-3.048" y2="-1.016" width="0.2032" layer="51"/>
<wire x1="2.54" y1="-3.048" x2="2.159" y2="-3.048" width="0.2032" layer="51"/>
<wire x1="-2.54" y1="-3.048" x2="-2.159" y2="-3.048" width="0.2032" layer="51"/>
<wire x1="-2.54" y1="3.048" x2="-2.159" y2="3.048" width="0.2032" layer="51"/>
<wire x1="2.54" y1="3.048" x2="2.159" y2="3.048" width="0.2032" layer="51"/>
<wire x1="2.159" y1="3.048" x2="-2.159" y2="3.048" width="0.2032" layer="21"/>
<wire x1="-2.159" y1="-3.048" x2="2.159" y2="-3.048" width="0.2032" layer="21"/>
<wire x1="3.048" y1="0.998" x2="3.048" y2="-1.016" width="0.2032" layer="21"/>
<wire x1="-3.048" y1="1.028" x2="-3.048" y2="-1.016" width="0.2032" layer="21"/>
<wire x1="-2.54" y1="1.27" x2="-2.54" y2="0.508" width="0.2032" layer="51"/>
<wire x1="-2.54" y1="-0.508" x2="-2.54" y2="-1.27" width="0.2032" layer="51"/>
<wire x1="-2.54" y1="0.508" x2="-2.159" y2="-0.381" width="0.2032" layer="51"/>
<circle x="0" y="0" radius="1.778" width="0.2032" layer="21"/>
<pad name="1" x="-3.2512" y="2.2606" drill="1.016" diameter="1.8796"/>
<pad name="2" x="3.2512" y="2.2606" drill="1.016" diameter="1.8796"/>
<pad name="3" x="-3.2512" y="-2.2606" drill="1.016" diameter="1.8796"/>
<pad name="4" x="3.2512" y="-2.2606" drill="1.016" diameter="1.8796"/>
<text x="-2.54" y="3.81" size="1.27" layer="25" ratio="10">&gt;NAME</text>
</package>
<package name="TACTILE_SWITCH_SMD-2">
<wire x1="1.905" y1="1.27" x2="1.905" y2="0.445" width="0.127" layer="51"/>
<wire x1="1.905" y1="0.445" x2="2.16" y2="-0.01" width="0.127" layer="51"/>
<wire x1="1.905" y1="-0.23" x2="1.905" y2="-1.115" width="0.127" layer="51"/>
<wire x1="-2.25" y1="2.25" x2="2.25" y2="2.25" width="0.127" layer="51"/>
<wire x1="2.25" y1="2.25" x2="2.25" y2="-2.25" width="0.127" layer="51"/>
<wire x1="2.25" y1="-2.25" x2="-2.25" y2="-2.25" width="0.127" layer="51"/>
<wire x1="-2.25" y1="-2.25" x2="-2.25" y2="2.25" width="0.127" layer="51"/>
<wire x1="-2.2" y1="0.8" x2="-2.2" y2="-0.8" width="0.2032" layer="21"/>
<wire x1="1.3" y1="2.2" x2="-1.3" y2="2.2" width="0.2032" layer="21"/>
<wire x1="2.2" y1="-0.8" x2="2.2" y2="0.8" width="0.2032" layer="21"/>
<wire x1="-1.3" y1="-2.2" x2="1.3" y2="-2.2" width="0.2032" layer="21"/>
<wire x1="2.2" y1="0.8" x2="1.8" y2="0.8" width="0.2032" layer="21"/>
<wire x1="2.2" y1="-0.8" x2="1.8" y2="-0.8" width="0.2032" layer="21"/>
<wire x1="-1.8" y1="0.8" x2="-2.2" y2="0.8" width="0.2032" layer="21"/>
<wire x1="-1.8" y1="-0.8" x2="-2.2" y2="-0.8" width="0.2032" layer="21"/>
<circle x="0" y="0" radius="1.27" width="0.2032" layer="21"/>
<smd name="1" x="2.225" y="1.75" dx="1.1" dy="0.7" layer="1" rot="R90"/>
<smd name="2" x="2.225" y="-1.75" dx="1.1" dy="0.7" layer="1" rot="R90"/>
<smd name="3" x="-2.225" y="-1.75" dx="1.1" dy="0.7" layer="1" rot="R90"/>
<smd name="4" x="-2.225" y="1.75" dx="1.1" dy="0.7" layer="1" rot="R90"/>
<text x="-0.889" y="1.778" size="0.4064" layer="25">&gt;NAME</text>
<text x="-0.889" y="-2.032" size="0.4064" layer="27">&gt;Value</text>
</package>
<package name="TACTILE-PTH-12MM">
<wire x1="5" y1="-1.3" x2="5" y2="-0.7" width="0.2032" layer="51"/>
<wire x1="5" y1="-0.7" x2="4.5" y2="-0.2" width="0.2032" layer="51"/>
<wire x1="5" y1="0.2" x2="5" y2="1" width="0.2032" layer="51"/>
<wire x1="-6" y1="4" x2="-6" y2="5" width="0.2032" layer="21"/>
<wire x1="-5" y1="6" x2="5" y2="6" width="0.2032" layer="21"/>
<wire x1="6" y1="5" x2="6" y2="4" width="0.2032" layer="21"/>
<wire x1="6" y1="1" x2="6" y2="-1" width="0.2032" layer="21"/>
<wire x1="6" y1="-4" x2="6" y2="-5" width="0.2032" layer="21"/>
<wire x1="5" y1="-6" x2="-5" y2="-6" width="0.2032" layer="21"/>
<wire x1="-6" y1="-5" x2="-6" y2="-4" width="0.2032" layer="21"/>
<wire x1="-6" y1="-1" x2="-6" y2="1" width="0.2032" layer="21"/>
<wire x1="-6" y1="5" x2="-5" y2="6" width="0.2032" layer="21" curve="-90"/>
<wire x1="5" y1="6" x2="6" y2="5" width="0.2032" layer="21" curve="-90"/>
<wire x1="6" y1="-5" x2="5" y2="-6" width="0.2032" layer="21" curve="-90"/>
<wire x1="-5" y1="-6" x2="-6" y2="-5" width="0.2032" layer="21" curve="-90"/>
<circle x="0" y="0" radius="3.5" width="0.2032" layer="21"/>
<circle x="-4.5" y="4.5" radius="0.3" width="0.7" layer="21"/>
<circle x="4.5" y="4.5" radius="0.3" width="0.7" layer="21"/>
<circle x="4.5" y="-4.5" radius="0.3" width="0.7" layer="21"/>
<circle x="-4.5" y="-4.5" radius="0.3" width="0.7" layer="21"/>
<pad name="4" x="-6.25" y="2.5" drill="1.2" diameter="2.159"/>
<pad name="2" x="-6.25" y="-2.5" drill="1.2" diameter="2.159"/>
<pad name="1" x="6.25" y="-2.5" drill="1.2" diameter="2.159"/>
<pad name="3" x="6.25" y="2.5" drill="1.2" diameter="2.159"/>
</package>
<package name="TACTILE-SWITCH-1101NE">
<description>SparkFun SKU# COM-08229</description>
<wire x1="-3" y1="1.1" x2="-3" y2="-1.1" width="0.127" layer="51"/>
<wire x1="3" y1="1.1" x2="3" y2="-1.1" width="0.127" layer="51"/>
<wire x1="-2.75" y1="1.75" x2="-3" y2="1.5" width="0.2032" layer="21" curve="90"/>
<wire x1="-2.75" y1="1.75" x2="2.75" y2="1.75" width="0.2032" layer="21"/>
<wire x1="2.75" y1="1.75" x2="3" y2="1.5" width="0.2032" layer="21" curve="-90"/>
<wire x1="3" y1="-1.5" x2="2.75" y2="-1.75" width="0.2032" layer="21" curve="-90"/>
<wire x1="2.75" y1="-1.75" x2="-2.75" y2="-1.75" width="0.2032" layer="21"/>
<wire x1="-3" y1="-1.5" x2="-2.75" y2="-1.75" width="0.2032" layer="21" curve="90"/>
<wire x1="-3" y1="-1.5" x2="-3" y2="-1.1" width="0.2032" layer="21"/>
<wire x1="-3" y1="1.1" x2="-3" y2="1.5" width="0.2032" layer="21"/>
<wire x1="3" y1="1.1" x2="3" y2="1.5" width="0.2032" layer="21"/>
<wire x1="3" y1="-1.5" x2="3" y2="-1.1" width="0.2032" layer="21"/>
<wire x1="-1.5" y1="0.75" x2="1.5" y2="0.75" width="0.2032" layer="21"/>
<wire x1="1.5" y1="-0.75" x2="-1.5" y2="-0.75" width="0.2032" layer="21"/>
<wire x1="-1.5" y1="-0.75" x2="-1.5" y2="0.75" width="0.2032" layer="21"/>
<wire x1="1.5" y1="-0.75" x2="1.5" y2="0.75" width="0.2032" layer="21"/>
<wire x1="-2" y1="0" x2="-1" y2="0" width="0.127" layer="51"/>
<wire x1="-1" y1="0" x2="0.1" y2="0.5" width="0.127" layer="51"/>
<wire x1="0.3" y1="0" x2="2" y2="0" width="0.127" layer="51"/>
<smd name="1" x="-3.15" y="0" dx="2.3" dy="1.6" layer="1" rot="R180"/>
<smd name="2" x="3.15" y="0" dx="2.3" dy="1.6" layer="1" rot="R180"/>
<text x="-3" y="2" size="0.762" layer="25">&gt;NAME</text>
<text x="-3" y="-2.7" size="0.762" layer="27">&gt;VALUE</text>
</package>
<package name="REED_SWITCH_PTH">
<wire x1="-6.985" y1="-0.635" x2="6.985" y2="-0.635" width="0.127" layer="21"/>
<wire x1="-6.985" y1="-0.635" x2="-6.985" y2="0" width="0.127" layer="21"/>
<wire x1="-6.985" y1="0" x2="-6.985" y2="0.635" width="0.127" layer="21"/>
<wire x1="-6.985" y1="0.635" x2="6.985" y2="0.635" width="0.127" layer="21"/>
<wire x1="6.985" y1="0.635" x2="6.985" y2="0" width="0.127" layer="21"/>
<wire x1="6.985" y1="0" x2="6.985" y2="-0.635" width="0.127" layer="21"/>
<wire x1="-6.985" y1="0" x2="-7.62" y2="0" width="0.127" layer="21"/>
<wire x1="6.985" y1="0" x2="7.62" y2="0" width="0.127" layer="21"/>
<pad name="P$1" x="-8.89" y="0" drill="1.016" diameter="1.8796"/>
<pad name="P$2" x="8.89" y="0" drill="1.016" diameter="1.8796"/>
</package>
<package name="TACTILE_SWITCH_TALL">
<wire x1="-3" y1="-3" x2="3" y2="-3" width="0.254" layer="21"/>
<wire x1="3" y1="-3" x2="3" y2="3" width="0.254" layer="21"/>
<wire x1="3" y1="3" x2="-3" y2="3" width="0.254" layer="21"/>
<wire x1="-3" y1="3" x2="-3" y2="-3" width="0.254" layer="21"/>
<circle x="0" y="0" radius="1.75" width="0.254" layer="21"/>
<smd name="A1" x="-3.975" y="-2.25" dx="1.3" dy="1.55" layer="1" rot="R90"/>
<smd name="A2" x="3.975" y="-2.25" dx="1.3" dy="1.55" layer="1" rot="R90"/>
<smd name="B1" x="-3.975" y="2.25" dx="1.3" dy="1.55" layer="1" rot="R90"/>
<smd name="B2" x="3.975" y="2.25" dx="1.3" dy="1.55" layer="1" rot="R90"/>
</package>
<package name="REED_SWITCH_PLASTIC">
<wire x1="-7.5" y1="-1.65" x2="7.5" y2="-1.65" width="0.127" layer="21"/>
<wire x1="-7.5" y1="-1.65" x2="-7.5" y2="0" width="0.127" layer="21"/>
<wire x1="-7.5" y1="0" x2="-7.5" y2="1.65" width="0.127" layer="21"/>
<wire x1="-7.5" y1="1.65" x2="7.5" y2="1.65" width="0.127" layer="21"/>
<wire x1="7.5" y1="1.65" x2="7.5" y2="0" width="0.127" layer="21"/>
<wire x1="7.5" y1="0" x2="7.5" y2="-1.65" width="0.127" layer="21"/>
<wire x1="-7.5" y1="0" x2="-7.72" y2="0" width="0.127" layer="21"/>
<wire x1="7.5" y1="0" x2="7.72" y2="0" width="0.127" layer="21"/>
<pad name="P$1" x="-8.89" y="0" drill="1.016" diameter="1.8796"/>
<pad name="P$2" x="8.89" y="0" drill="1.016" diameter="1.8796"/>
</package>
<package name="TACTILE-PTH-SIDEEZ">
<wire x1="1.5" y1="-3.8" x2="-1.5" y2="-3.8" width="0.2032" layer="51"/>
<wire x1="-3.65" y1="-2" x2="-3.65" y2="3.5" width="0.2032" layer="51"/>
<wire x1="-3.65" y1="3.5" x2="-3" y2="3.5" width="0.2032" layer="51"/>
<wire x1="3" y1="3.5" x2="3.65" y2="3.5" width="0.2032" layer="51"/>
<wire x1="3.65" y1="3.5" x2="3.65" y2="-2" width="0.2032" layer="51"/>
<wire x1="-3" y1="2" x2="3" y2="2" width="0.2032" layer="51"/>
<wire x1="-3" y1="2" x2="-3" y2="3.5" width="0.2032" layer="51"/>
<wire x1="3" y1="2" x2="3" y2="3.5" width="0.2032" layer="51"/>
<wire x1="-3.65" y1="-2" x2="-1.5" y2="-2" width="0.2032" layer="51"/>
<wire x1="-1.5" y1="-2" x2="1.5" y2="-2" width="0.2032" layer="51"/>
<wire x1="1.5" y1="-2" x2="3.65" y2="-2" width="0.2032" layer="51"/>
<wire x1="1.5" y1="-2" x2="1.5" y2="-3.8" width="0.2032" layer="51"/>
<wire x1="-1.5" y1="-2" x2="-1.5" y2="-3.8" width="0.2032" layer="51"/>
<wire x1="-3.65" y1="1" x2="-3.65" y2="-2" width="0.2032" layer="21"/>
<wire x1="-3.65" y1="-2" x2="3.65" y2="-2" width="0.2032" layer="21"/>
<wire x1="3.65" y1="-2" x2="3.65" y2="1" width="0.2032" layer="21"/>
<wire x1="2" y1="2" x2="-2" y2="2" width="0.2032" layer="21"/>
<pad name="ANCHOR1" x="-3.5" y="2.5" drill="1.2" diameter="2.2" stop="no"/>
<pad name="ANCHOR2" x="3.5" y="2.5" drill="1.2" diameter="2.2" stop="no"/>
<pad name="1" x="-2.5" y="0" drill="0.8" diameter="1.7" stop="no"/>
<pad name="2" x="2.5" y="0" drill="0.8" diameter="1.7" stop="no"/>
<text x="-2.54" y="3.81" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<circle x="2.5" y="0" radius="0.4445" width="0" layer="29"/>
<circle x="-2.5" y="0" radius="0.4445" width="0" layer="29"/>
<circle x="-3.5" y="2.5" radius="0.635" width="0" layer="29"/>
<circle x="3.5" y="2.5" radius="0.635" width="0" layer="29"/>
<circle x="-3.5" y="2.5" radius="1.143" width="0" layer="30"/>
<circle x="2.5" y="0" radius="0.889" width="0" layer="30"/>
<circle x="-2.5" y="0" radius="0.889" width="0" layer="30"/>
<circle x="3.5" y="2.5" radius="1.143" width="0" layer="30"/>
</package>
<package name="TACTILE_SWITCH_SMD-3">
<wire x1="-2.04" y1="-0.44" x2="-2.04" y2="0.47" width="0.2032" layer="21"/>
<wire x1="-1.04" y1="1.14" x2="1.04" y2="1.14" width="0.2032" layer="21"/>
<circle x="0" y="0" radius="0.8" width="0.15" layer="21"/>
<smd name="1" x="-1.8" y="1.1" dx="0.8" dy="1" layer="1" rot="R90"/>
<smd name="2" x="1.8" y="1.1" dx="0.8" dy="1" layer="1" rot="R90"/>
<smd name="3" x="-1.8" y="-1.1" dx="0.8" dy="1" layer="1" rot="R90"/>
<smd name="4" x="1.8" y="-1.1" dx="0.8" dy="1" layer="1" rot="R90"/>
<wire x1="2.06" y1="-0.44" x2="2.06" y2="0.47" width="0.2032" layer="21"/>
<wire x1="-1.04" y1="-1.16" x2="1.04" y2="-1.16" width="0.2032" layer="21"/>
</package>
<package name="TACTILE-SMD-12MM">
<wire x1="5" y1="-1.3" x2="5" y2="-0.7" width="0.2032" layer="51"/>
<wire x1="5" y1="-0.7" x2="4.5" y2="-0.2" width="0.2032" layer="51"/>
<wire x1="5" y1="0.2" x2="5" y2="1" width="0.2032" layer="51"/>
<wire x1="-6" y1="4" x2="-6" y2="5" width="0.2032" layer="21"/>
<wire x1="-5" y1="6" x2="5" y2="6" width="0.2032" layer="21"/>
<wire x1="6" y1="5" x2="6" y2="4" width="0.2032" layer="21"/>
<wire x1="6" y1="1" x2="6" y2="-1" width="0.2032" layer="21"/>
<wire x1="6" y1="-4" x2="6" y2="-5" width="0.2032" layer="21"/>
<wire x1="5" y1="-6" x2="-5" y2="-6" width="0.2032" layer="21"/>
<wire x1="-6" y1="-5" x2="-6" y2="-4" width="0.2032" layer="21"/>
<wire x1="-6" y1="-1" x2="-6" y2="1" width="0.2032" layer="21"/>
<circle x="0" y="0" radius="3.5" width="0.2032" layer="21"/>
<circle x="-4.5" y="4.5" radius="0.3" width="0.7" layer="21"/>
<circle x="4.5" y="4.5" radius="0.3" width="0.7" layer="21"/>
<circle x="4.5" y="-4.5" radius="0.3" width="0.7" layer="21"/>
<circle x="-4.5" y="-4.5" radius="0.3" width="0.7" layer="21"/>
<smd name="4" x="-6.975" y="2.5" dx="1.6" dy="1.55" layer="1"/>
<smd name="2" x="-6.975" y="-2.5" dx="1.6" dy="1.55" layer="1"/>
<smd name="1" x="6.975" y="-2.5" dx="1.6" dy="1.55" layer="1"/>
<smd name="3" x="6.975" y="2.5" dx="1.6" dy="1.55" layer="1"/>
<wire x1="-6" y1="-5" x2="-5" y2="-6" width="0.2032" layer="21"/>
<wire x1="6" y1="-5" x2="5" y2="-6" width="0.2032" layer="21"/>
<wire x1="6" y1="5" x2="5" y2="6" width="0.2032" layer="21"/>
<wire x1="-5" y1="6" x2="-6" y2="5" width="0.2032" layer="21"/>
</package>
<package name="TACTILE-PTH-EZ">
<wire x1="3.048" y1="1.016" x2="3.048" y2="2.54" width="0.2032" layer="51"/>
<wire x1="3.048" y1="2.54" x2="2.54" y2="3.048" width="0.2032" layer="51"/>
<wire x1="2.54" y1="-3.048" x2="3.048" y2="-2.54" width="0.2032" layer="51"/>
<wire x1="3.048" y1="-2.54" x2="3.048" y2="-1.016" width="0.2032" layer="51"/>
<wire x1="-2.54" y1="3.048" x2="-3.048" y2="2.54" width="0.2032" layer="51"/>
<wire x1="-3.048" y1="2.54" x2="-3.048" y2="1.016" width="0.2032" layer="51"/>
<wire x1="-2.54" y1="-3.048" x2="-3.048" y2="-2.54" width="0.2032" layer="51"/>
<wire x1="-3.048" y1="-2.54" x2="-3.048" y2="-1.016" width="0.2032" layer="51"/>
<wire x1="2.54" y1="-3.048" x2="2.159" y2="-3.048" width="0.2032" layer="51"/>
<wire x1="-2.54" y1="-3.048" x2="-2.159" y2="-3.048" width="0.2032" layer="51"/>
<wire x1="-2.54" y1="3.048" x2="-2.159" y2="3.048" width="0.2032" layer="51"/>
<wire x1="2.54" y1="3.048" x2="2.159" y2="3.048" width="0.2032" layer="51"/>
<wire x1="2.159" y1="3.048" x2="-2.159" y2="3.048" width="0.2032" layer="21"/>
<wire x1="-2.159" y1="-3.048" x2="2.159" y2="-3.048" width="0.2032" layer="21"/>
<wire x1="3.048" y1="0.998" x2="3.048" y2="-1.016" width="0.2032" layer="21"/>
<wire x1="-3.048" y1="1.028" x2="-3.048" y2="-1.016" width="0.2032" layer="21"/>
<wire x1="-2.54" y1="1.27" x2="-2.54" y2="0.508" width="0.2032" layer="51"/>
<wire x1="-2.54" y1="-0.508" x2="-2.54" y2="-1.27" width="0.2032" layer="51"/>
<wire x1="-2.54" y1="0.508" x2="-2.159" y2="-0.381" width="0.2032" layer="51"/>
<circle x="0" y="0" radius="1.778" width="0.2032" layer="21"/>
<pad name="1" x="-3.2512" y="2.2606" drill="1.016" diameter="1.8796" stop="no"/>
<pad name="2" x="3.2512" y="2.2606" drill="1.016" diameter="1.8796" stop="no"/>
<pad name="3" x="-3.2512" y="-2.2606" drill="1.016" diameter="1.8796" stop="no"/>
<pad name="4" x="3.2512" y="-2.2606" drill="1.016" diameter="1.8796" stop="no"/>
<text x="-2.54" y="3.81" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<polygon width="0.127" layer="30">
<vertex x="-3.2664" y="3.142"/>
<vertex x="-3.2589" y="3.1445" curve="89.986886"/>
<vertex x="-4.1326" y="2.286"/>
<vertex x="-4.1351" y="2.2657" curve="90.00652"/>
<vertex x="-3.2563" y="1.392"/>
<vertex x="-3.2487" y="1.3869" curve="90.006616"/>
<vertex x="-2.3826" y="2.2403"/>
<vertex x="-2.3775" y="2.2683" curve="89.98711"/>
</polygon>
<polygon width="0.127" layer="29">
<vertex x="-3.2462" y="2.7026"/>
<vertex x="-3.2589" y="2.7051" curve="90.026544"/>
<vertex x="-3.6881" y="2.2733"/>
<vertex x="-3.6881" y="2.2632" curve="89.974074"/>
<vertex x="-3.2562" y="1.8213"/>
<vertex x="-3.2259" y="1.8186" curve="90.051271"/>
<vertex x="-2.8093" y="2.2658"/>
<vertex x="-2.8093" y="2.2606" curve="90.012964"/>
</polygon>
<polygon width="0.127" layer="30">
<vertex x="3.2411" y="3.1395"/>
<vertex x="3.2486" y="3.142" curve="89.986886"/>
<vertex x="2.3749" y="2.2835"/>
<vertex x="2.3724" y="2.2632" curve="90.00652"/>
<vertex x="3.2512" y="1.3895"/>
<vertex x="3.2588" y="1.3844" curve="90.006616"/>
<vertex x="4.1249" y="2.2378"/>
<vertex x="4.13" y="2.2658" curve="89.98711"/>
</polygon>
<polygon width="0.127" layer="29">
<vertex x="3.2613" y="2.7001"/>
<vertex x="3.2486" y="2.7026" curve="90.026544"/>
<vertex x="2.8194" y="2.2708"/>
<vertex x="2.8194" y="2.2607" curve="89.974074"/>
<vertex x="3.2513" y="1.8188"/>
<vertex x="3.2816" y="1.8161" curve="90.051271"/>
<vertex x="3.6982" y="2.2633"/>
<vertex x="3.6982" y="2.2581" curve="90.012964"/>
</polygon>
<polygon width="0.127" layer="30">
<vertex x="-3.2613" y="-1.3868"/>
<vertex x="-3.2538" y="-1.3843" curve="89.986886"/>
<vertex x="-4.1275" y="-2.2428"/>
<vertex x="-4.13" y="-2.2631" curve="90.00652"/>
<vertex x="-3.2512" y="-3.1368"/>
<vertex x="-3.2436" y="-3.1419" curve="90.006616"/>
<vertex x="-2.3775" y="-2.2885"/>
<vertex x="-2.3724" y="-2.2605" curve="89.98711"/>
</polygon>
<polygon width="0.127" layer="29">
<vertex x="-3.2411" y="-1.8262"/>
<vertex x="-3.2538" y="-1.8237" curve="90.026544"/>
<vertex x="-3.683" y="-2.2555"/>
<vertex x="-3.683" y="-2.2656" curve="89.974074"/>
<vertex x="-3.2511" y="-2.7075"/>
<vertex x="-3.2208" y="-2.7102" curve="90.051271"/>
<vertex x="-2.8042" y="-2.263"/>
<vertex x="-2.8042" y="-2.2682" curve="90.012964"/>
</polygon>
<polygon width="0.127" layer="30">
<vertex x="3.2411" y="-1.3843"/>
<vertex x="3.2486" y="-1.3818" curve="89.986886"/>
<vertex x="2.3749" y="-2.2403"/>
<vertex x="2.3724" y="-2.2606" curve="90.00652"/>
<vertex x="3.2512" y="-3.1343"/>
<vertex x="3.2588" y="-3.1394" curve="90.006616"/>
<vertex x="4.1249" y="-2.286"/>
<vertex x="4.13" y="-2.258" curve="89.98711"/>
</polygon>
<polygon width="0.127" layer="29">
<vertex x="3.2613" y="-1.8237"/>
<vertex x="3.2486" y="-1.8212" curve="90.026544"/>
<vertex x="2.8194" y="-2.253"/>
<vertex x="2.8194" y="-2.2631" curve="89.974074"/>
<vertex x="3.2513" y="-2.705"/>
<vertex x="3.2816" y="-2.7077" curve="90.051271"/>
<vertex x="3.6982" y="-2.2605"/>
<vertex x="3.6982" y="-2.2657" curve="90.012964"/>
</polygon>
</package>
<package name="TACTILE-SWITCH-SMD">
<wire x1="-1.54" y1="-2.54" x2="-2.54" y2="-1.54" width="0.2032" layer="51"/>
<wire x1="-2.54" y1="-1.24" x2="-2.54" y2="1.27" width="0.2032" layer="21"/>
<wire x1="-2.54" y1="1.54" x2="-1.54" y2="2.54" width="0.2032" layer="51"/>
<wire x1="-1.54" y1="2.54" x2="1.54" y2="2.54" width="0.2032" layer="21"/>
<wire x1="1.54" y1="2.54" x2="2.54" y2="1.54" width="0.2032" layer="51"/>
<wire x1="2.54" y1="1.24" x2="2.54" y2="-1.24" width="0.2032" layer="21"/>
<wire x1="2.54" y1="-1.54" x2="1.54" y2="-2.54" width="0.2032" layer="51"/>
<wire x1="1.54" y1="-2.54" x2="-1.54" y2="-2.54" width="0.2032" layer="21"/>
<wire x1="1.905" y1="1.27" x2="1.905" y2="0.445" width="0.127" layer="51"/>
<wire x1="1.905" y1="0.445" x2="2.16" y2="-0.01" width="0.127" layer="51"/>
<wire x1="1.905" y1="-0.23" x2="1.905" y2="-1.115" width="0.127" layer="51"/>
<circle x="0" y="0" radius="1.27" width="0.2032" layer="21"/>
<smd name="1" x="-2.794" y="1.905" dx="0.762" dy="1.524" layer="1" rot="R90"/>
<smd name="2" x="2.794" y="1.905" dx="0.762" dy="1.524" layer="1" rot="R90"/>
<smd name="3" x="-2.794" y="-1.905" dx="0.762" dy="1.524" layer="1" rot="R90"/>
<smd name="4" x="2.794" y="-1.905" dx="0.762" dy="1.524" layer="1" rot="R90"/>
<text x="-0.889" y="1.778" size="0.4064" layer="25">&gt;NAME</text>
<text x="-0.889" y="-2.032" size="0.4064" layer="27">&gt;Value</text>
</package>
<package name="TACTILE_SWITCH-SMD-RIGHT-ANGLE">
<hole x="0" y="0.9" drill="0.7"/>
<hole x="0" y="-0.9" drill="0.7"/>
<smd name="1" x="-1.95" y="0" dx="2" dy="1.1" layer="1" rot="R90"/>
<smd name="2" x="1.95" y="0" dx="2" dy="1.1" layer="1" rot="R90"/>
<wire x1="-2" y1="1.2" x2="-2" y2="1.5" width="0.127" layer="21"/>
<wire x1="-2" y1="1.5" x2="2" y2="1.5" width="0.127" layer="21"/>
<wire x1="2" y1="1.5" x2="2" y2="1.2" width="0.127" layer="21"/>
<wire x1="-2" y1="-1.2" x2="-2" y2="-1.5" width="0.127" layer="21"/>
<wire x1="-2" y1="-1.5" x2="-0.7" y2="-1.5" width="0.127" layer="21"/>
<wire x1="-0.7" y1="-1.5" x2="0.7" y2="-1.5" width="0.127" layer="21"/>
<wire x1="0.7" y1="-1.5" x2="2" y2="-1.5" width="0.127" layer="21"/>
<wire x1="2" y1="-1.5" x2="2" y2="-1.2" width="0.127" layer="21"/>
<wire x1="-0.7" y1="-2.1" x2="0.7" y2="-2.1" width="0.127" layer="21"/>
<wire x1="0.7" y1="-2.1" x2="0.7" y2="-1.5" width="0.127" layer="21"/>
<wire x1="-0.7" y1="-2.1" x2="-0.7" y2="-1.5" width="0.127" layer="21"/>
<text x="-2" y="1.7" size="0.8128" layer="25" ratio="15">&gt;NAME</text>
</package>
<package name="TACTILE_SWITCH_SMD_4.6X2.8MM">
<description>&lt;h3&gt;4.6 x 2.8mm Tactile Switch&lt;/h3&gt;

&lt;p&gt;&lt;a href="http://www.digikey.com/product-detail/en/KMR231NG%20LFS/CKN10246CT-ND/2176497"&gt;Example&lt;/a&gt;&lt;/p&gt;</description>
<smd name="3" x="2.05" y="0.8" dx="0.9" dy="1" layer="1"/>
<smd name="2" x="2.05" y="-0.8" dx="0.9" dy="1" layer="1"/>
<smd name="1" x="-2.05" y="-0.8" dx="0.9" dy="1" layer="1"/>
<smd name="4" x="-2.05" y="0.8" dx="0.9" dy="1" layer="1"/>
<wire x1="-2.1" y1="1.4" x2="-2.1" y2="-1.4" width="0.127" layer="51"/>
<wire x1="2.1" y1="-1.4" x2="2.1" y2="1.4" width="0.127" layer="51"/>
<wire x1="-2.1" y1="1.4" x2="2.1" y2="1.4" width="0.127" layer="51"/>
<wire x1="-2.1" y1="-1.4" x2="2.1" y2="-1.4" width="0.127" layer="51"/>
<circle x="0" y="0" radius="0.805" width="0.127" layer="21"/>
<wire x1="1.338" y1="-1.4" x2="-1.338" y2="-1.4" width="0.2032" layer="21"/>
<wire x1="-1.338" y1="1.4" x2="1.338" y2="1.4" width="0.2032" layer="21"/>
<wire x1="-2.1" y1="0.13" x2="-2.1" y2="-0.13" width="0.2032" layer="21"/>
<wire x1="2.1" y1="-0.13" x2="2.1" y2="0.13" width="0.2032" layer="21"/>
<text x="-2.54" y="1.524" size="0.8128" layer="25">&gt;Name</text>
<text x="-2.54" y="-1.524" size="0.8128" layer="27" align="top-left">&gt;Value</text>
<rectangle x1="-2.3" y1="0.5" x2="-2.1" y2="1.1" layer="51"/>
<rectangle x1="-2.3" y1="-1.1" x2="-2.1" y2="-0.5" layer="51"/>
<rectangle x1="2.1" y1="-1.1" x2="2.3" y2="-0.5" layer="51" rot="R180"/>
<rectangle x1="2.1" y1="0.5" x2="2.3" y2="1.1" layer="51" rot="R180"/>
</package>
</packages>
<symbols>
<symbol name="SWITCH-MOMENTARY-2">
<wire x1="1.905" y1="0" x2="2.54" y2="0" width="0.254" layer="94"/>
<wire x1="-2.54" y1="0" x2="1.905" y2="1.27" width="0.254" layer="94"/>
<circle x="-2.54" y="0" radius="0.127" width="0.4064" layer="94"/>
<circle x="2.54" y="0" radius="0.127" width="0.4064" layer="94"/>
<text x="-2.54" y="2.54" size="1.778" layer="95">&gt;NAME</text>
<text x="-2.54" y="-2.54" size="1.778" layer="96">&gt;VALUE</text>
<pin name="1" x="-5.08" y="0" visible="off" length="short" direction="pas" swaplevel="2"/>
<pin name="2" x="5.08" y="0" visible="off" length="short" direction="pas" swaplevel="1" rot="R180"/>
</symbol>
</symbols>
<devicesets>
<deviceset name="SWITCH-MOMENTARY-2" prefix="S">
<description>Various NO switches- pushbuttons, reed, etc</description>
<gates>
<gate name="G$1" symbol="SWITCH-MOMENTARY-2" x="0" y="0"/>
</gates>
<devices>
<device name="PTH" package="TACTILE-PTH">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="3"/>
</connects>
<technologies>
<technology name="">
<attribute name="PROD_ID" value=" SWCH-08441" constant="no"/>
</technology>
</technologies>
</device>
<device name="SMD-2" package="TACTILE_SWITCH_SMD-2">
<connects>
<connect gate="G$1" pin="1" pad="2"/>
<connect gate="G$1" pin="2" pad="1"/>
</connects>
<technologies>
<technology name="">
<attribute name="PROD_ID" value="SWCH-09213"/>
</technology>
</technologies>
</device>
<device name="12MM" package="TACTILE-PTH-12MM">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="3"/>
</connects>
<technologies>
<technology name="">
<attribute name="PROD_ID" value="SWCH-09185" constant="no"/>
</technology>
</technologies>
</device>
<device name="SMD-1101NE" package="TACTILE-SWITCH-1101NE">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name="">
<attribute name="PROD_ID" value="SWCH-00815" constant="no"/>
</technology>
</technologies>
</device>
<device name="PTH_REED" package="REED_SWITCH_PTH">
<connects>
<connect gate="G$1" pin="1" pad="P$1"/>
<connect gate="G$1" pin="2" pad="P$2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="SMD-4" package="TACTILE_SWITCH_TALL">
<connects>
<connect gate="G$1" pin="1" pad="A2"/>
<connect gate="G$1" pin="2" pad="B2"/>
</connects>
<technologies>
<technology name="">
<attribute name="PROD_ID" value="SWCH-11966" constant="no"/>
</technology>
</technologies>
</device>
<device name="PTH_REED2" package="REED_SWITCH_PLASTIC">
<connects>
<connect gate="G$1" pin="1" pad="P$1"/>
<connect gate="G$1" pin="2" pad="P$2"/>
</connects>
<technologies>
<technology name="">
<attribute name="PROD_ID" value="SWCH-10467" constant="no"/>
</technology>
</technologies>
</device>
<device name="SIDE_EZ" package="TACTILE-PTH-SIDEEZ">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="SMD-3" package="TACTILE_SWITCH_SMD-3">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="4"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="SMD-12MM" package="TACTILE-SMD-12MM">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="3"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="PTH_EZ" package="TACTILE-PTH-EZ">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="3"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="SMD" package="TACTILE-SWITCH-SMD">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="3"/>
</connects>
<technologies>
<technology name="">
<attribute name="PROD_ID" value="SWCH-08247" constant="no"/>
</technology>
</technologies>
</device>
<device name="SMD-REDUNDANT" package="TACTILE-SWITCH-SMD">
<connects>
<connect gate="G$1" pin="1" pad="1 2"/>
<connect gate="G$1" pin="2" pad="3 4"/>
</connects>
<technologies>
<technology name="">
<attribute name="PROD_ID" value="SWCH-08247" constant="no"/>
</technology>
</technologies>
</device>
<device name="TACTILE-SWITCH-SMD-RIGHT-ANGLE" package="TACTILE_SWITCH-SMD-RIGHT-ANGLE">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name="">
<attribute name="PROD_ID" value="COMP-12265" constant="no"/>
</technology>
</technologies>
</device>
<device name="SMD-4.6X2.8MM" package="TACTILE_SWITCH_SMD_4.6X2.8MM">
<connects>
<connect gate="G$1" pin="1" pad="1 2"/>
<connect gate="G$1" pin="2" pad="3 4"/>
</connects>
<technologies>
<technology name="">
<attribute name="PROD_ID" value="SWCH-13065"/>
</technology>
</technologies>
</device>
</devices>
</deviceset>
</devicesets>
</library>
<library name="SparkFun-Capacitors">
<description>&lt;h3&gt;SparkFun Electronics' preferred foot prints&lt;/h3&gt;
In this library you'll find resistors, capacitors, inductors, test points, jumper pads, etc.&lt;br&gt;&lt;br&gt;
We've spent an enormous amount of time creating and checking these footprints and parts, but it is the end user's responsibility to ensure correctness and suitablity for a given componet or application. If you enjoy using this library, please buy one of our products at www.sparkfun.com.
&lt;br&gt;&lt;br&gt;
&lt;b&gt;Licensing:&lt;/b&gt; Creative Commons ShareAlike 4.0 International - https://creativecommons.org/licenses/by-sa/4.0/ 
&lt;br&gt;&lt;br&gt;
You are welcome to use this library for commercial purposes. For attribution, we ask that when you begin to sell your device using our footprint, you email us with a link to the product being sold. We want bragging rights that we helped (in a very small part) to create your 8th world wonder. We would like the opportunity to feature your device on our homepage.</description>
<packages>
<package name="CAP-PTH-SMALL-KIT">
<description>&lt;h3&gt;CAP-PTH-SMALL-KIT&lt;/h3&gt;
Commonly used for small ceramic capacitors. Like our 0.1uF (http://www.sparkfun.com/products/8375) or 22pF caps (http://www.sparkfun.com/products/8571).&lt;br&gt;
&lt;br&gt;
&lt;b&gt;Warning:&lt;/b&gt; This is the KIT version of this package. This package has a smaller diameter top stop mask, which doesn't cover the diameter of the pad. This means only the bottom side of the pads' copper will be exposed. You'll only be able to solder to the bottom side.</description>
<wire x1="0" y1="0.635" x2="0" y2="-0.635" width="0.254" layer="21"/>
<wire x1="-2.667" y1="1.27" x2="2.667" y2="1.27" width="0.254" layer="21"/>
<wire x1="2.667" y1="1.27" x2="2.667" y2="-1.27" width="0.254" layer="21"/>
<wire x1="2.667" y1="-1.27" x2="-2.667" y2="-1.27" width="0.254" layer="21"/>
<wire x1="-2.667" y1="-1.27" x2="-2.667" y2="1.27" width="0.254" layer="21"/>
<pad name="1" x="-1.397" y="0" drill="1.016" diameter="2.032" stop="no"/>
<pad name="2" x="1.397" y="0" drill="1.016" diameter="2.032" stop="no"/>
<polygon width="0.127" layer="30">
<vertex x="-1.4021" y="-0.9475" curve="-90"/>
<vertex x="-2.357" y="-0.0178" curve="-90.011749"/>
<vertex x="-1.4046" y="0.9576" curve="-90"/>
<vertex x="-0.4546" y="-0.0204" curve="-90.024193"/>
</polygon>
<polygon width="0.127" layer="29">
<vertex x="-1.4046" y="-0.4395" curve="-90.012891"/>
<vertex x="-1.8491" y="-0.0153" curve="-90"/>
<vertex x="-1.4046" y="0.452" curve="-90"/>
<vertex x="-0.9627" y="-0.0051" curve="-90.012967"/>
</polygon>
<polygon width="0.127" layer="30">
<vertex x="1.397" y="-0.9475" curve="-90"/>
<vertex x="0.4421" y="-0.0178" curve="-90.011749"/>
<vertex x="1.3945" y="0.9576" curve="-90"/>
<vertex x="2.3445" y="-0.0204" curve="-90.024193"/>
</polygon>
<polygon width="0.127" layer="29">
<vertex x="1.3945" y="-0.4395" curve="-90.012891"/>
<vertex x="0.95" y="-0.0153" curve="-90"/>
<vertex x="1.3945" y="0.452" curve="-90"/>
<vertex x="1.8364" y="-0.0051" curve="-90.012967"/>
</polygon>
</package>
</packages>
<symbols>
<symbol name="CAP">
<wire x1="0" y1="2.54" x2="0" y2="2.032" width="0.1524" layer="94"/>
<wire x1="0" y1="0" x2="0" y2="0.508" width="0.1524" layer="94"/>
<text x="1.524" y="2.921" size="1.778" layer="95">&gt;NAME</text>
<text x="1.524" y="-2.159" size="1.778" layer="96">&gt;VALUE</text>
<rectangle x1="-2.032" y1="0.508" x2="2.032" y2="1.016" layer="94"/>
<rectangle x1="-2.032" y1="1.524" x2="2.032" y2="2.032" layer="94"/>
<pin name="1" x="0" y="5.08" visible="off" length="short" direction="pas" swaplevel="1" rot="R270"/>
<pin name="2" x="0" y="-2.54" visible="off" length="short" direction="pas" swaplevel="1" rot="R90"/>
</symbol>
</symbols>
<devicesets>
<deviceset name="0.1UF-50V-20%(PTH)" prefix="C" uservalue="yes">
<description>CAP-08370</description>
<gates>
<gate name="G$1" symbol="CAP" x="0" y="-2.54"/>
</gates>
<devices>
<device name="KIT-EZ" package="CAP-PTH-SMALL-KIT">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name="">
<attribute name="PROD_ID" value="CAP-08370" constant="no"/>
</technology>
</technologies>
</device>
</devices>
</deviceset>
</devicesets>
</library>
<library name="adafruit">
<packages>
<package name="FEATHERWING">
<wire x1="2.54" y1="0" x2="48.26" y2="0" width="0" layer="20"/>
<wire x1="48.26" y1="0" x2="50.8" y2="2.54" width="0" layer="20" curve="90"/>
<wire x1="50.8" y1="2.54" x2="50.8" y2="20.32" width="0" layer="20"/>
<wire x1="50.8" y1="20.32" x2="48.26" y2="22.86" width="0" layer="20" curve="90"/>
<wire x1="48.26" y1="22.86" x2="2.54" y2="22.86" width="0" layer="20"/>
<wire x1="2.54" y1="22.86" x2="0" y2="20.32" width="0" layer="20" curve="90"/>
<wire x1="0" y1="20.32" x2="0" y2="2.54" width="0" layer="20"/>
<wire x1="0" y1="2.54" x2="2.54" y2="0" width="0" layer="20" curve="90"/>
<hole x="48.26" y="20.32" drill="2.54"/>
<hole x="48.26" y="2.54" drill="2.54"/>
<pad name="P$1" x="2.54" y="20.32" drill="2.5" diameter="3.81"/>
<pad name="P$2" x="2.54" y="2.54" drill="2.5" diameter="3.81"/>
<pad name="28" x="16.51" y="21.59" drill="1" diameter="1.778"/>
<pad name="27" x="19.05" y="21.59" drill="1" diameter="1.778"/>
<pad name="26" x="21.59" y="21.59" drill="1" diameter="1.778"/>
<pad name="25" x="24.13" y="21.59" drill="1" diameter="1.778"/>
<pad name="24" x="26.67" y="21.59" drill="1" diameter="1.778"/>
<pad name="23" x="29.21" y="21.59" drill="1" diameter="1.778"/>
<pad name="22" x="31.75" y="21.59" drill="1" diameter="1.778"/>
<pad name="21" x="34.29" y="21.59" drill="1" diameter="1.778"/>
<pad name="20" x="36.83" y="21.59" drill="1" diameter="1.778"/>
<pad name="19" x="39.37" y="21.59" drill="1" diameter="1.778"/>
<pad name="18" x="41.91" y="21.59" drill="1" diameter="1.778"/>
<pad name="17" x="44.45" y="21.59" drill="1" diameter="1.778"/>
<pad name="5" x="16.51" y="1.27" drill="1" diameter="1.778"/>
<pad name="6" x="19.05" y="1.27" drill="1" diameter="1.778"/>
<pad name="7" x="21.59" y="1.27" drill="1" diameter="1.778"/>
<pad name="8" x="24.13" y="1.27" drill="1" diameter="1.778"/>
<pad name="9" x="26.67" y="1.27" drill="1" diameter="1.778"/>
<pad name="10" x="29.21" y="1.27" drill="1" diameter="1.778"/>
<pad name="11" x="31.75" y="1.27" drill="1" diameter="1.778"/>
<pad name="12" x="34.29" y="1.27" drill="1" diameter="1.778"/>
<pad name="13" x="36.83" y="1.27" drill="1" diameter="1.778"/>
<pad name="14" x="39.37" y="1.27" drill="1" diameter="1.778"/>
<pad name="15" x="41.91" y="1.27" drill="1" diameter="1.778"/>
<pad name="16" x="44.45" y="1.27" drill="1" diameter="1.778"/>
<pad name="4" x="13.97" y="1.27" drill="1" diameter="1.778"/>
<pad name="3" x="11.43" y="1.27" drill="1" diameter="1.778"/>
<pad name="2" x="8.89" y="1.27" drill="1" diameter="1.778"/>
<pad name="1" x="6.35" y="1.27" drill="1" diameter="1.778"/>
</package>
<package name="FEATHERWING_DIM">
<wire x1="2.54" y1="0" x2="48.26" y2="0" width="0" layer="21"/>
<wire x1="48.26" y1="0" x2="50.8" y2="2.54" width="0" layer="21" curve="90"/>
<wire x1="50.8" y1="2.54" x2="50.8" y2="20.32" width="0" layer="21"/>
<wire x1="50.8" y1="20.32" x2="48.26" y2="22.86" width="0" layer="21" curve="90"/>
<wire x1="48.26" y1="22.86" x2="2.54" y2="22.86" width="0" layer="21"/>
<wire x1="2.54" y1="22.86" x2="0" y2="20.32" width="0" layer="21" curve="90"/>
<wire x1="0" y1="20.32" x2="0" y2="13.716" width="0" layer="21"/>
<wire x1="0" y1="13.716" x2="0.508" y2="13.208" width="0" layer="21"/>
<wire x1="0.508" y1="13.208" x2="0.508" y2="9.652" width="0" layer="21"/>
<wire x1="0.508" y1="9.652" x2="0" y2="9.144" width="0" layer="21"/>
<wire x1="0" y1="9.144" x2="0" y2="2.54" width="0" layer="21"/>
<wire x1="0" y1="2.54" x2="2.54" y2="0" width="0" layer="21" curve="90"/>
<hole x="48.26" y="20.32" drill="2.54"/>
<hole x="48.26" y="2.54" drill="2.54"/>
<pad name="P$1" x="2.54" y="20.32" drill="2.5" diameter="3.81"/>
<pad name="P$2" x="2.54" y="2.54" drill="2.5" diameter="3.81"/>
<pad name="28" x="16.51" y="21.59" drill="1" diameter="1.778"/>
<pad name="27" x="19.05" y="21.59" drill="1" diameter="1.778"/>
<pad name="26" x="21.59" y="21.59" drill="1" diameter="1.778"/>
<pad name="25" x="24.13" y="21.59" drill="1" diameter="1.778"/>
<pad name="24" x="26.67" y="21.59" drill="1" diameter="1.778"/>
<pad name="23" x="29.21" y="21.59" drill="1" diameter="1.778"/>
<pad name="22" x="31.75" y="21.59" drill="1" diameter="1.778"/>
<pad name="21" x="34.29" y="21.59" drill="1" diameter="1.778"/>
<pad name="20" x="36.83" y="21.59" drill="1" diameter="1.778"/>
<pad name="19" x="39.37" y="21.59" drill="1" diameter="1.778"/>
<pad name="18" x="41.91" y="21.59" drill="1" diameter="1.778"/>
<pad name="17" x="44.45" y="21.59" drill="1" diameter="1.778"/>
<pad name="5" x="16.51" y="1.27" drill="1" diameter="1.778"/>
<pad name="6" x="19.05" y="1.27" drill="1" diameter="1.778"/>
<pad name="7" x="21.59" y="1.27" drill="1" diameter="1.778"/>
<pad name="8" x="24.13" y="1.27" drill="1" diameter="1.778"/>
<pad name="9" x="26.67" y="1.27" drill="1" diameter="1.778"/>
<pad name="10" x="29.21" y="1.27" drill="1" diameter="1.778"/>
<pad name="11" x="31.75" y="1.27" drill="1" diameter="1.778"/>
<pad name="12" x="34.29" y="1.27" drill="1" diameter="1.778"/>
<pad name="13" x="36.83" y="1.27" drill="1" diameter="1.778"/>
<pad name="14" x="39.37" y="1.27" drill="1" diameter="1.778"/>
<pad name="15" x="41.91" y="1.27" drill="1" diameter="1.778"/>
<pad name="16" x="44.45" y="1.27" drill="1" diameter="1.778"/>
<pad name="4" x="13.97" y="1.27" drill="1" diameter="1.778"/>
<pad name="3" x="11.43" y="1.27" drill="1" diameter="1.778"/>
<pad name="2" x="8.89" y="1.27" drill="1" diameter="1.778"/>
<pad name="1" x="6.35" y="1.27" drill="1" diameter="1.778"/>
</package>
<package name="DCJACK_2MM_PTH">
<description>DJ Jack 2.0mm PTH Right-Angle</description>
<wire x1="4.5" y1="14.2" x2="2.4" y2="14.2" width="0.2032" layer="21"/>
<wire x1="-4.5" y1="3.3" x2="-4.5" y2="0" width="0.2032" layer="51"/>
<wire x1="4.5" y1="0" x2="4.5" y2="3.3" width="0.2032" layer="51"/>
<wire x1="4.5" y1="0" x2="-4.5" y2="0" width="0.2032" layer="51"/>
<wire x1="4.5" y1="3.3" x2="4.5" y2="8.4" width="0.2032" layer="21"/>
<wire x1="4.5" y1="14.2" x2="4.5" y2="13.1" width="0.2032" layer="21"/>
<wire x1="-4.5" y1="3.3" x2="-4.5" y2="14.2" width="0.2032" layer="21"/>
<wire x1="-4.5" y1="14.2" x2="-2.6" y2="14.2" width="0.2032" layer="21"/>
<wire x1="-4.5" y1="3.3" x2="4.5" y2="3.3" width="0.2032" layer="21"/>
<wire x1="-4.5" y1="3.3" x2="4.5" y2="3.3" width="0.2032" layer="51"/>
<wire x1="-4.5" y1="3.3" x2="-4.5" y2="12.45" width="0.2032" layer="51"/>
<wire x1="-4.5" y1="12.45" x2="-4.5" y2="14.2" width="0.2032" layer="51"/>
<wire x1="4.5" y1="3.3" x2="4.5" y2="8.35" width="0.2032" layer="51"/>
<wire x1="4.5" y1="8.35" x2="4.5" y2="8.4" width="0.2032" layer="51"/>
<wire x1="-4.5" y1="14.2" x2="2.65" y2="14.2" width="0.2032" layer="51"/>
<wire x1="4.5" y1="14.2" x2="2.65" y2="14.2" width="0.2032" layer="51"/>
<wire x1="2.65" y1="14.2" x2="2.4" y2="14.2" width="0.2032" layer="51"/>
<wire x1="4.5" y1="14.2" x2="4.5" y2="8.35" width="0.2032" layer="51"/>
<wire x1="-4.5" y1="12.45" x2="4.4" y2="12.45" width="0.2032" layer="51"/>
<pad name="PWR" x="0" y="13.6" drill="3.2"/>
<pad name="GND" x="0" y="7.35" drill="2.8"/>
<pad name="GNDBREAK" x="4.8" y="10.75" drill="2.8" rot="R90"/>
<text x="-5.08" y="0" size="1.27" layer="25" rot="R90">&gt;NAME</text>
<text x="6.35" y="0" size="1.27" layer="27" rot="R90">&gt;VALUE</text>
<text x="-3.302" y="1.27" size="0.8128" layer="51">DC 2.0/2.1</text>
</package>
<package name="DCJACK_2MM_SMT">
<description>2.0/2.1mm DC Jack - SMT
&lt;p&gt;4UConnector: 03267&lt;/p&gt;
&lt;p&gt;Note: Small tRestrict polygon's were added to the ground pads to improve solderability when this part is used in combination with a ground pour.  By default, Eagle will product four large bridges to the ground pour significantly increasing the heat distribution on the pads and preventing lead-free solder from reflowing in certain situations.  For more details, see: http://www.microbuilder.eu/Blog/09-12-14/Reducing_Thermals_for_Large_Pads_in_Eagle.aspx&lt;/p&gt;</description>
<wire x1="-4" y1="4.5" x2="-5" y2="3.5" width="0.127" layer="51" curve="90"/>
<wire x1="-5" y1="3.5" x2="-5" y2="-3.5" width="0.127" layer="51"/>
<wire x1="-5" y1="-3.5" x2="-4" y2="-4.5" width="0.127" layer="51" curve="90"/>
<wire x1="-4" y1="-4.5" x2="10.254" y2="-4.5" width="0.127" layer="51"/>
<wire x1="10.254" y1="-4.5" x2="10.254" y2="-1.492" width="0.127" layer="51"/>
<wire x1="10.254" y1="-1.492" x2="9" y2="-1.492" width="0.127" layer="51"/>
<wire x1="9" y1="-1.492" x2="9" y2="4.5" width="0.127" layer="51"/>
<wire x1="9" y1="4.5" x2="-4" y2="4.5" width="0.127" layer="51"/>
<wire x1="-4" y1="4.5" x2="-5" y2="3.5" width="0.127" layer="21" curve="90"/>
<wire x1="-5" y1="3.5" x2="-5" y2="-3.5" width="0.127" layer="21"/>
<wire x1="-5" y1="-3.5" x2="-4" y2="-4.5" width="0.127" layer="21" curve="90"/>
<wire x1="10.254" y1="-4.5" x2="10.254" y2="-1.492" width="0.127" layer="21"/>
<wire x1="10.254" y1="-1.492" x2="9" y2="-1.492" width="0.127" layer="21"/>
<wire x1="9" y1="-1.492" x2="9" y2="4.5" width="0.127" layer="21"/>
<wire x1="-1.668" y1="4.5" x2="-4" y2="4.5" width="0.127" layer="21"/>
<wire x1="4.682" y1="4.5" x2="1.588" y2="4.5" width="0.127" layer="21"/>
<wire x1="9" y1="4.5" x2="7.938" y2="4.5" width="0.127" layer="21"/>
<wire x1="-4" y1="-4.5" x2="-1.684" y2="-4.5" width="0.127" layer="21"/>
<wire x1="1.588" y1="-4.5" x2="4.666" y2="-4.5" width="0.127" layer="21"/>
<wire x1="7.938" y1="-4.5" x2="10.254" y2="-4.5" width="0.127" layer="21"/>
<smd name="PWR1" x="0" y="5.5" dx="2.4" dy="2" layer="1"/>
<smd name="PWR2" x="6.2" y="5.5" dx="2.4" dy="2" layer="1"/>
<smd name="GNDBREAK" x="6.2" y="-5.5" dx="2.4" dy="2" layer="1"/>
<smd name="GND" x="0" y="-5.5" dx="2.4" dy="2" layer="1"/>
<text x="0.762" y="2.794" size="1.4224" layer="21" ratio="12" rot="R90">+</text>
<text x="-1.016" y="-3.81" size="0.8128" layer="21">GND</text>
<hole x="0" y="0" drill="1.6"/>
<hole x="4.5" y="0" drill="1.8"/>
<polygon width="0.0254" layer="41" spacing="0.254">
<vertex x="1.27" y="-5.7404"/>
<vertex x="1.27" y="-5.2578"/>
<vertex x="1.2954" y="-5.2578"/>
<vertex x="1.2954" y="-5.7404"/>
</polygon>
<polygon width="0.0254" layer="41" spacing="0.254">
<vertex x="-0.254" y="-4.4196"/>
<vertex x="0.2286" y="-4.4196"/>
<vertex x="0.2286" y="-4.445"/>
<vertex x="-0.254" y="-4.445"/>
</polygon>
<polygon width="0.0254" layer="41" spacing="0.254">
<vertex x="4.9022" y="-5.7404"/>
<vertex x="4.9022" y="-5.2578"/>
<vertex x="4.9276" y="-5.2578"/>
<vertex x="4.9276" y="-5.7404"/>
</polygon>
<polygon width="0.0254" layer="41" spacing="0.254">
<vertex x="6.4262" y="-4.445"/>
<vertex x="5.9436" y="-4.445"/>
<vertex x="5.9436" y="-4.4196"/>
<vertex x="6.4262" y="-4.4196"/>
</polygon>
</package>
</packages>
<symbols>
<symbol name="MICROSHIELD">
<wire x1="0" y1="33.02" x2="0" y2="22.86" width="0.254" layer="94"/>
<wire x1="0" y1="22.86" x2="0" y2="12.7" width="0.254" layer="94"/>
<wire x1="0" y1="12.7" x2="0" y2="0" width="0.254" layer="94"/>
<wire x1="0" y1="0" x2="48.26" y2="0" width="0.254" layer="94"/>
<wire x1="48.26" y1="0" x2="48.26" y2="33.02" width="0.254" layer="94"/>
<wire x1="48.26" y1="33.02" x2="12.7" y2="33.02" width="0.254" layer="94"/>
<pin name="!RESET" x="5.08" y="-5.08" length="middle" direction="in" rot="R90"/>
<pin name="3V" x="7.62" y="-5.08" length="middle" direction="sup" rot="R90"/>
<pin name="AREF" x="10.16" y="-5.08" length="middle" direction="pas" rot="R90"/>
<pin name="GND" x="12.7" y="-5.08" length="middle" direction="pwr" rot="R90"/>
<pin name="GPIOA0" x="15.24" y="-5.08" length="middle" rot="R90"/>
<pin name="GPIOA1" x="17.78" y="-5.08" length="middle" rot="R90"/>
<pin name="GPIOA2" x="20.32" y="-5.08" length="middle" rot="R90"/>
<pin name="GPIOA3" x="22.86" y="-5.08" length="middle" rot="R90"/>
<pin name="GPIOA4" x="25.4" y="-5.08" length="middle" rot="R90"/>
<pin name="GPIOA5" x="27.94" y="-5.08" length="middle" rot="R90"/>
<pin name="GPIOSCK" x="30.48" y="-5.08" length="middle" rot="R90"/>
<pin name="GPIOMOSI" x="33.02" y="-5.08" length="middle" rot="R90"/>
<pin name="GPIOMISO" x="35.56" y="-5.08" length="middle" rot="R90"/>
<pin name="GPIORX" x="38.1" y="-5.08" length="middle" rot="R90"/>
<pin name="GPIOTX" x="40.64" y="-5.08" length="middle" rot="R90"/>
<pin name="NC" x="43.18" y="-5.08" length="middle" direction="pas" rot="R90"/>
<pin name="GPIOSDA" x="43.18" y="38.1" length="middle" rot="R270"/>
<pin name="GPIOSCL" x="40.64" y="38.1" length="middle" rot="R270"/>
<circle x="45.72" y="30.48" radius="1.27" width="0.254" layer="94"/>
<circle x="45.72" y="2.54" radius="1.27" width="0.254" layer="94"/>
<pin name="GPIO5" x="38.1" y="38.1" length="middle" rot="R270"/>
<pin name="GPIO9" x="33.02" y="38.1" length="middle" rot="R270"/>
<pin name="GPIO6" x="35.56" y="38.1" length="middle" rot="R270"/>
<pin name="GPIO10" x="30.48" y="38.1" length="middle" rot="R270"/>
<pin name="GPIO11" x="27.94" y="38.1" length="middle" rot="R270"/>
<pin name="GPIO12" x="25.4" y="38.1" length="middle" rot="R270"/>
<pin name="GPIO13" x="22.86" y="38.1" length="middle" rot="R270"/>
<pin name="EN" x="17.78" y="38.1" length="middle" direction="pas" rot="R270"/>
<pin name="USB" x="20.32" y="38.1" length="middle" direction="sup" rot="R270"/>
<pin name="VBAT" x="15.24" y="38.1" length="middle" direction="sup" rot="R270"/>
<wire x1="12.7" y1="33.02" x2="5.08" y2="33.02" width="0.254" layer="94"/>
<wire x1="5.08" y1="33.02" x2="0" y2="33.02" width="0.254" layer="94"/>
<wire x1="5.08" y1="33.02" x2="5.08" y2="25.4" width="0.254" layer="94"/>
<wire x1="5.08" y1="25.4" x2="7.62" y2="25.4" width="0.254" layer="94"/>
<wire x1="7.62" y1="25.4" x2="10.16" y2="25.4" width="0.254" layer="94"/>
<wire x1="10.16" y1="25.4" x2="12.7" y2="25.4" width="0.254" layer="94"/>
<wire x1="12.7" y1="25.4" x2="12.7" y2="33.02" width="0.254" layer="94"/>
<wire x1="7.62" y1="27.94" x2="7.62" y2="25.4" width="0.254" layer="94"/>
<wire x1="10.16" y1="27.94" x2="10.16" y2="25.4" width="0.254" layer="94"/>
<wire x1="0" y1="22.86" x2="5.08" y2="22.86" width="0.254" layer="94"/>
<wire x1="5.08" y1="22.86" x2="5.08" y2="12.7" width="0.254" layer="94"/>
<wire x1="5.08" y1="12.7" x2="0" y2="12.7" width="0.254" layer="94"/>
<circle x="2.54" y="2.54" radius="1.27" width="0.254" layer="94"/>
<circle x="2.54" y="30.48" radius="1.27" width="0.254" layer="94"/>
</symbol>
<symbol name="DCBARREL">
<wire x1="0" y1="0" x2="0" y2="-2.54" width="0.1524" layer="94"/>
<wire x1="0" y1="-2.54" x2="-1.27" y2="-2.54" width="0.254" layer="94"/>
<wire x1="-5.08" y1="-2.54" x2="-3.81" y2="-2.54" width="0.254" layer="94"/>
<wire x1="-3.81" y1="-2.54" x2="-2.54" y2="-1.27" width="0.254" layer="94"/>
<wire x1="-2.54" y1="-1.27" x2="-1.27" y2="-2.54" width="0.254" layer="94"/>
<wire x1="0" y1="3.175" x2="0" y2="1.905" width="0.254" layer="94"/>
<wire x1="-4.445" y1="3.175" x2="-4.445" y2="1.905" width="0.254" layer="94" curve="180"/>
<wire x1="0" y1="1.905" x2="-4.445" y2="1.905" width="0.254" layer="94"/>
<wire x1="0" y1="3.175" x2="-4.445" y2="3.175" width="0.254" layer="94"/>
<text x="-5.08" y="5.08" size="1.27" layer="95">&gt;NAME</text>
<text x="-5.08" y="-5.08" size="1.27" layer="96">&gt;VALUE</text>
<pin name="PWR" x="2.54" y="2.54" visible="pad" length="short" direction="pwr" rot="R180"/>
<pin name="GNDBREAK" x="2.54" y="0" visible="pad" length="short" direction="pwr" rot="R180"/>
<pin name="GND" x="2.54" y="-2.54" visible="pad" length="short" direction="pwr" rot="R180"/>
</symbol>
</symbols>
<devicesets>
<deviceset name="FEATHERWING" prefix="MS">
<gates>
<gate name="G$1" symbol="MICROSHIELD" x="-25.4" y="-15.24"/>
</gates>
<devices>
<device name="" package="FEATHERWING">
<connects>
<connect gate="G$1" pin="!RESET" pad="1"/>
<connect gate="G$1" pin="3V" pad="2"/>
<connect gate="G$1" pin="AREF" pad="3"/>
<connect gate="G$1" pin="EN" pad="27"/>
<connect gate="G$1" pin="GND" pad="4"/>
<connect gate="G$1" pin="GPIO10" pad="22"/>
<connect gate="G$1" pin="GPIO11" pad="23"/>
<connect gate="G$1" pin="GPIO12" pad="24"/>
<connect gate="G$1" pin="GPIO13" pad="25"/>
<connect gate="G$1" pin="GPIO5" pad="19"/>
<connect gate="G$1" pin="GPIO6" pad="20"/>
<connect gate="G$1" pin="GPIO9" pad="21"/>
<connect gate="G$1" pin="GPIOA0" pad="5"/>
<connect gate="G$1" pin="GPIOA1" pad="6"/>
<connect gate="G$1" pin="GPIOA2" pad="7"/>
<connect gate="G$1" pin="GPIOA3" pad="8"/>
<connect gate="G$1" pin="GPIOA4" pad="9"/>
<connect gate="G$1" pin="GPIOA5" pad="10"/>
<connect gate="G$1" pin="GPIOMISO" pad="13"/>
<connect gate="G$1" pin="GPIOMOSI" pad="12"/>
<connect gate="G$1" pin="GPIORX" pad="14"/>
<connect gate="G$1" pin="GPIOSCK" pad="11"/>
<connect gate="G$1" pin="GPIOSCL" pad="18"/>
<connect gate="G$1" pin="GPIOSDA" pad="17"/>
<connect gate="G$1" pin="GPIOTX" pad="15"/>
<connect gate="G$1" pin="NC" pad="16"/>
<connect gate="G$1" pin="USB" pad="26"/>
<connect gate="G$1" pin="VBAT" pad="28"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="_NODIM" package="FEATHERWING_DIM">
<connects>
<connect gate="G$1" pin="!RESET" pad="1"/>
<connect gate="G$1" pin="3V" pad="2"/>
<connect gate="G$1" pin="AREF" pad="3"/>
<connect gate="G$1" pin="EN" pad="27"/>
<connect gate="G$1" pin="GND" pad="4"/>
<connect gate="G$1" pin="GPIO10" pad="22"/>
<connect gate="G$1" pin="GPIO11" pad="23"/>
<connect gate="G$1" pin="GPIO12" pad="24"/>
<connect gate="G$1" pin="GPIO13" pad="25"/>
<connect gate="G$1" pin="GPIO5" pad="19"/>
<connect gate="G$1" pin="GPIO6" pad="20"/>
<connect gate="G$1" pin="GPIO9" pad="21"/>
<connect gate="G$1" pin="GPIOA0" pad="5"/>
<connect gate="G$1" pin="GPIOA1" pad="6"/>
<connect gate="G$1" pin="GPIOA2" pad="7"/>
<connect gate="G$1" pin="GPIOA3" pad="8"/>
<connect gate="G$1" pin="GPIOA4" pad="9"/>
<connect gate="G$1" pin="GPIOA5" pad="10"/>
<connect gate="G$1" pin="GPIOMISO" pad="13"/>
<connect gate="G$1" pin="GPIOMOSI" pad="12"/>
<connect gate="G$1" pin="GPIORX" pad="14"/>
<connect gate="G$1" pin="GPIOSCK" pad="11"/>
<connect gate="G$1" pin="GPIOSCL" pad="18"/>
<connect gate="G$1" pin="GPIOSDA" pad="17"/>
<connect gate="G$1" pin="GPIOTX" pad="15"/>
<connect gate="G$1" pin="NC" pad="16"/>
<connect gate="G$1" pin="USB" pad="26"/>
<connect gate="G$1" pin="VBAT" pad="28"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
</devices>
</deviceset>
<deviceset name="DCBARREL" prefix="CN" uservalue="yes">
<description>&lt;p&gt;&lt;b&gt;2.0mm DC Barrel Jack&lt;/b&gt;&lt;/p&gt;
&lt;b&gt;DCJACK_2MM_PTH&lt;/b&gt; - Through Hole Jack (4UConnector: 05537)</description>
<gates>
<gate name="G$1" symbol="DCBARREL" x="0" y="0"/>
</gates>
<devices>
<device name="PTH" package="DCJACK_2MM_PTH">
<connects>
<connect gate="G$1" pin="GND" pad="GND"/>
<connect gate="G$1" pin="GNDBREAK" pad="GNDBREAK"/>
<connect gate="G$1" pin="PWR" pad="PWR"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="SMT" package="DCJACK_2MM_SMT">
<connects>
<connect gate="G$1" pin="GND" pad="GNDBREAK"/>
<connect gate="G$1" pin="GNDBREAK" pad="GND"/>
<connect gate="G$1" pin="PWR" pad="PWR1"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
</devices>
</deviceset>
</devicesets>
</library>
<library name="pinhead">
<description>&lt;b&gt;Pin Header Connectors&lt;/b&gt;&lt;p&gt;
&lt;author&gt;Created by librarian@cadsoft.de&lt;/author&gt;</description>
<packages>
<package name="1X03">
<description>&lt;b&gt;PIN HEADER&lt;/b&gt;</description>
<wire x1="-3.175" y1="1.27" x2="-1.905" y2="1.27" width="0.1524" layer="21"/>
<wire x1="-1.905" y1="1.27" x2="-1.27" y2="0.635" width="0.1524" layer="21"/>
<wire x1="-1.27" y1="0.635" x2="-1.27" y2="-0.635" width="0.1524" layer="21"/>
<wire x1="-1.27" y1="-0.635" x2="-1.905" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="-1.27" y1="0.635" x2="-0.635" y2="1.27" width="0.1524" layer="21"/>
<wire x1="-0.635" y1="1.27" x2="0.635" y2="1.27" width="0.1524" layer="21"/>
<wire x1="0.635" y1="1.27" x2="1.27" y2="0.635" width="0.1524" layer="21"/>
<wire x1="1.27" y1="0.635" x2="1.27" y2="-0.635" width="0.1524" layer="21"/>
<wire x1="1.27" y1="-0.635" x2="0.635" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="0.635" y1="-1.27" x2="-0.635" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="-0.635" y1="-1.27" x2="-1.27" y2="-0.635" width="0.1524" layer="21"/>
<wire x1="-3.81" y1="0.635" x2="-3.81" y2="-0.635" width="0.1524" layer="21"/>
<wire x1="-3.175" y1="1.27" x2="-3.81" y2="0.635" width="0.1524" layer="21"/>
<wire x1="-3.81" y1="-0.635" x2="-3.175" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="-1.905" y1="-1.27" x2="-3.175" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="1.27" y1="0.635" x2="1.905" y2="1.27" width="0.1524" layer="21"/>
<wire x1="1.905" y1="1.27" x2="3.175" y2="1.27" width="0.1524" layer="21"/>
<wire x1="3.175" y1="1.27" x2="3.81" y2="0.635" width="0.1524" layer="21"/>
<wire x1="3.81" y1="0.635" x2="3.81" y2="-0.635" width="0.1524" layer="21"/>
<wire x1="3.81" y1="-0.635" x2="3.175" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="3.175" y1="-1.27" x2="1.905" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="1.905" y1="-1.27" x2="1.27" y2="-0.635" width="0.1524" layer="21"/>
<pad name="1" x="-2.54" y="0" drill="1.016" shape="long" rot="R90"/>
<pad name="2" x="0" y="0" drill="1.016" shape="long" rot="R90"/>
<pad name="3" x="2.54" y="0" drill="1.016" shape="long" rot="R90"/>
<text x="-3.8862" y="1.8288" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-3.81" y="-3.175" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="-0.254" y1="-0.254" x2="0.254" y2="0.254" layer="51"/>
<rectangle x1="-2.794" y1="-0.254" x2="-2.286" y2="0.254" layer="51"/>
<rectangle x1="2.286" y1="-0.254" x2="2.794" y2="0.254" layer="51"/>
</package>
<package name="1X03/90">
<description>&lt;b&gt;PIN HEADER&lt;/b&gt;</description>
<wire x1="-3.81" y1="-1.905" x2="-1.27" y2="-1.905" width="0.1524" layer="21"/>
<wire x1="-1.27" y1="-1.905" x2="-1.27" y2="0.635" width="0.1524" layer="21"/>
<wire x1="-1.27" y1="0.635" x2="-3.81" y2="0.635" width="0.1524" layer="21"/>
<wire x1="-3.81" y1="0.635" x2="-3.81" y2="-1.905" width="0.1524" layer="21"/>
<wire x1="-2.54" y1="6.985" x2="-2.54" y2="1.27" width="0.762" layer="21"/>
<wire x1="-1.27" y1="-1.905" x2="1.27" y2="-1.905" width="0.1524" layer="21"/>
<wire x1="1.27" y1="-1.905" x2="1.27" y2="0.635" width="0.1524" layer="21"/>
<wire x1="1.27" y1="0.635" x2="-1.27" y2="0.635" width="0.1524" layer="21"/>
<wire x1="0" y1="6.985" x2="0" y2="1.27" width="0.762" layer="21"/>
<wire x1="1.27" y1="-1.905" x2="3.81" y2="-1.905" width="0.1524" layer="21"/>
<wire x1="3.81" y1="-1.905" x2="3.81" y2="0.635" width="0.1524" layer="21"/>
<wire x1="3.81" y1="0.635" x2="1.27" y2="0.635" width="0.1524" layer="21"/>
<wire x1="2.54" y1="6.985" x2="2.54" y2="1.27" width="0.762" layer="21"/>
<pad name="1" x="-2.54" y="-3.81" drill="1.016" shape="long" rot="R90"/>
<pad name="2" x="0" y="-3.81" drill="1.016" shape="long" rot="R90"/>
<pad name="3" x="2.54" y="-3.81" drill="1.016" shape="long" rot="R90"/>
<text x="-4.445" y="-3.81" size="1.27" layer="25" ratio="10" rot="R90">&gt;NAME</text>
<text x="5.715" y="-3.81" size="1.27" layer="27" rot="R90">&gt;VALUE</text>
<rectangle x1="-2.921" y1="0.635" x2="-2.159" y2="1.143" layer="21"/>
<rectangle x1="-0.381" y1="0.635" x2="0.381" y2="1.143" layer="21"/>
<rectangle x1="2.159" y1="0.635" x2="2.921" y2="1.143" layer="21"/>
<rectangle x1="-2.921" y1="-2.921" x2="-2.159" y2="-1.905" layer="21"/>
<rectangle x1="-0.381" y1="-2.921" x2="0.381" y2="-1.905" layer="21"/>
<rectangle x1="2.159" y1="-2.921" x2="2.921" y2="-1.905" layer="21"/>
</package>
</packages>
<symbols>
<symbol name="PINHD3">
<wire x1="-6.35" y1="-5.08" x2="1.27" y2="-5.08" width="0.4064" layer="94"/>
<wire x1="1.27" y1="-5.08" x2="1.27" y2="5.08" width="0.4064" layer="94"/>
<wire x1="1.27" y1="5.08" x2="-6.35" y2="5.08" width="0.4064" layer="94"/>
<wire x1="-6.35" y1="5.08" x2="-6.35" y2="-5.08" width="0.4064" layer="94"/>
<text x="-6.35" y="5.715" size="1.778" layer="95">&gt;NAME</text>
<text x="-6.35" y="-7.62" size="1.778" layer="96">&gt;VALUE</text>
<pin name="1" x="-2.54" y="2.54" visible="pad" length="short" direction="pas" function="dot"/>
<pin name="2" x="-2.54" y="0" visible="pad" length="short" direction="pas" function="dot"/>
<pin name="3" x="-2.54" y="-2.54" visible="pad" length="short" direction="pas" function="dot"/>
</symbol>
</symbols>
<devicesets>
<deviceset name="PINHD-1X3" prefix="JP" uservalue="yes">
<description>&lt;b&gt;PIN HEADER&lt;/b&gt;</description>
<gates>
<gate name="A" symbol="PINHD3" x="0" y="0"/>
</gates>
<devices>
<device name="" package="1X03">
<connects>
<connect gate="A" pin="1" pad="1"/>
<connect gate="A" pin="2" pad="2"/>
<connect gate="A" pin="3" pad="3"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="/90" package="1X03/90">
<connects>
<connect gate="A" pin="1" pad="1"/>
<connect gate="A" pin="2" pad="2"/>
<connect gate="A" pin="3" pad="3"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
</devices>
</deviceset>
</devicesets>
</library>
<library name="supply2">
<description>&lt;b&gt;Supply Symbols&lt;/b&gt;&lt;p&gt;
GND, VCC, 0V, +5V, -5V, etc.&lt;p&gt;
Please keep in mind, that these devices are necessary for the
automatic wiring of the supply signals.&lt;p&gt;
The pin name defined in the symbol is identical to the net which is to be wired automatically.&lt;p&gt;
In this library the device names are the same as the pin names of the symbols, therefore the correct signal names appear next to the supply symbols in the schematic.&lt;p&gt;
&lt;author&gt;Created by librarian@cadsoft.de&lt;/author&gt;</description>
<packages>
</packages>
<symbols>
<symbol name="GND">
<wire x1="-1.27" y1="0" x2="1.27" y2="0" width="0.254" layer="94"/>
<wire x1="1.27" y1="0" x2="0" y2="-1.27" width="0.254" layer="94"/>
<wire x1="0" y1="-1.27" x2="-1.27" y2="0" width="0.254" layer="94"/>
<text x="-1.905" y="-3.175" size="1.778" layer="96">&gt;VALUE</text>
<pin name="GND" x="0" y="2.54" visible="off" length="short" direction="sup" rot="R270"/>
</symbol>
<symbol name="+05V">
<wire x1="-0.635" y1="1.27" x2="0.635" y2="1.27" width="0.1524" layer="94"/>
<wire x1="0" y1="0.635" x2="0" y2="1.905" width="0.1524" layer="94"/>
<circle x="0" y="1.27" radius="1.27" width="0.254" layer="94"/>
<text x="-1.905" y="3.175" size="1.778" layer="96">&gt;VALUE</text>
<pin name="+5V" x="0" y="-2.54" visible="off" length="short" direction="sup" rot="R90"/>
</symbol>
</symbols>
<devicesets>
<deviceset name="GND" prefix="SUPPLY">
<description>&lt;b&gt;SUPPLY SYMBOL&lt;/b&gt;</description>
<gates>
<gate name="GND" symbol="GND" x="0" y="0"/>
</gates>
<devices>
<device name="">
<technologies>
<technology name=""/>
</technologies>
</device>
</devices>
</deviceset>
<deviceset name="+5V" prefix="SUPPLY">
<description>&lt;b&gt;SUPPLY SYMBOL&lt;/b&gt;</description>
<gates>
<gate name="+5V" symbol="+05V" x="0" y="0"/>
</gates>
<devices>
<device name="">
<technologies>
<technology name=""/>
</technologies>
</device>
</devices>
</deviceset>
</devicesets>
</library>
</libraries>
<attributes>
<attribute name="DESIGNER" value="Adam Green"/>
</attributes>
<variantdefs>
</variantdefs>
<classes>
<class number="0" name="default" width="0" drill="0">
</class>
</classes>
<parts>
<part name="U$1" library="SparkFun-DigitalIC" deviceset="MBED_NXP_LPC1768" device=""/>
<part name="J1" library="adam_custom" deviceset="9DOF_SENSOR_STICK" device="" value="Sparkfun 9DoF Sensor Stick"/>
<part name="S1" library="SparkFun-Electromechanical" deviceset="SWITCH-MOMENTARY-2" device="PTH" value="Reset"/>
<part name="C1" library="SparkFun-Capacitors" deviceset="0.1UF-50V-20%(PTH)" device="KIT-EZ" value="0.1uF"/>
<part name="MS1" library="adafruit" deviceset="FEATHERWING" device=""/>
<part name="JP1" library="pinhead" deviceset="PINHD-1X3" device="/90" value="DHB-10"/>
<part name="CN1" library="adafruit" deviceset="DCBARREL" device="PTH"/>
<part name="U$2" library="adam_custom" deviceset="CC-BY-SA-DOCFIELD" device=""/>
<part name="SUPPLY1" library="supply2" deviceset="GND" device=""/>
<part name="SUPPLY2" library="supply2" deviceset="+5V" device=""/>
<part name="SUPPLY3" library="supply2" deviceset="GND" device=""/>
<part name="SUPPLY4" library="supply2" deviceset="+5V" device=""/>
<part name="SUPPLY6" library="supply2" deviceset="GND" device=""/>
<part name="SUPPLY5" library="supply2" deviceset="GND" device=""/>
<part name="SUPPLY7" library="supply2" deviceset="+5V" device=""/>
<part name="SUPPLY8" library="supply2" deviceset="GND" device=""/>
<part name="SUPPLY9" library="supply2" deviceset="+5V" device=""/>
<part name="JP2" library="pinhead" deviceset="PINHD-1X3" device="/90" value="9DoF Supply Select"/>
</parts>
<sheets>
<sheet>
<plain>
<text x="149.86" y="38.1" size="1.778" layer="95">Adafruit Feather Huzzah ESP8266</text>
</plain>
<instances>
<instance part="U$1" gate="G$1" x="73.66" y="66.04"/>
<instance part="J1" gate="G$1" x="137.16" y="22.86" smashed="yes" rot="R180">
<attribute name="VALUE" x="142.24" y="30.48" size="1.778" layer="96" rot="R270"/>
<attribute name="NAME" x="139.7" y="9.398" size="1.778" layer="95" rot="R180"/>
</instance>
<instance part="S1" gate="G$1" x="7.62" y="76.2" rot="R90"/>
<instance part="C1" gate="G$1" x="15.24" y="76.2"/>
<instance part="MS1" gate="G$1" x="152.4" y="91.44" rot="R270"/>
<instance part="JP1" gate="A" x="180.34" y="17.78"/>
<instance part="CN1" gate="G$1" x="10.16" y="93.98"/>
<instance part="U$2" gate="G$1" x="0" y="0"/>
<instance part="SUPPLY1" gate="GND" x="15.24" y="88.9"/>
<instance part="SUPPLY2" gate="+5V" x="15.24" y="99.06"/>
<instance part="SUPPLY3" gate="GND" x="25.4" y="86.36"/>
<instance part="SUPPLY4" gate="+5V" x="30.48" y="99.06"/>
<instance part="SUPPLY6" gate="GND" x="7.62" y="66.04"/>
<instance part="SUPPLY5" gate="GND" x="124.46" y="7.62"/>
<instance part="SUPPLY7" gate="+5V" x="116.84" y="38.1"/>
<instance part="SUPPLY8" gate="GND" x="142.24" y="76.2"/>
<instance part="SUPPLY9" gate="+5V" x="195.58" y="81.28"/>
<instance part="JP2" gate="A" x="137.16" y="35.56" smashed="yes">
<attribute name="VALUE" x="126.238" y="41.91" size="1.778" layer="96"/>
</instance>
</instances>
<busses>
</busses>
<nets>
<net name="GND" class="0">
<segment>
<pinref part="CN1" gate="G$1" pin="GND"/>
<pinref part="SUPPLY1" gate="GND" pin="GND"/>
<wire x1="12.7" y1="91.44" x2="15.24" y2="91.44" width="0.1524" layer="91"/>
</segment>
<segment>
<pinref part="SUPPLY3" gate="GND" pin="GND"/>
<pinref part="U$1" gate="G$1" pin="P$1"/>
<wire x1="25.4" y1="88.9" x2="35.56" y2="88.9" width="0.1524" layer="91"/>
</segment>
<segment>
<pinref part="S1" gate="G$1" pin="1"/>
<pinref part="SUPPLY6" gate="GND" pin="GND"/>
<wire x1="7.62" y1="68.58" x2="7.62" y2="71.12" width="0.1524" layer="91"/>
<pinref part="C1" gate="G$1" pin="2"/>
<wire x1="15.24" y1="73.66" x2="15.24" y2="68.58" width="0.1524" layer="91"/>
<wire x1="15.24" y1="68.58" x2="7.62" y2="68.58" width="0.1524" layer="91"/>
<junction x="7.62" y="68.58"/>
</segment>
<segment>
<pinref part="J1" gate="G$1" pin="3"/>
<pinref part="SUPPLY5" gate="GND" pin="GND"/>
<wire x1="127" y1="15.24" x2="124.46" y2="15.24" width="0.1524" layer="91"/>
<wire x1="124.46" y1="15.24" x2="124.46" y2="10.16" width="0.1524" layer="91"/>
</segment>
<segment>
<pinref part="MS1" gate="G$1" pin="GND"/>
<pinref part="SUPPLY8" gate="GND" pin="GND"/>
<wire x1="147.32" y1="78.74" x2="142.24" y2="78.74" width="0.1524" layer="91"/>
</segment>
</net>
<net name="+5V" class="0">
<segment>
<pinref part="CN1" gate="G$1" pin="PWR"/>
<pinref part="SUPPLY2" gate="+5V" pin="+5V"/>
<wire x1="12.7" y1="96.52" x2="15.24" y2="96.52" width="0.1524" layer="91"/>
</segment>
<segment>
<pinref part="SUPPLY4" gate="+5V" pin="+5V"/>
<pinref part="U$1" gate="G$1" pin="P$2"/>
<wire x1="30.48" y1="96.52" x2="30.48" y2="86.36" width="0.1524" layer="91"/>
<wire x1="30.48" y1="86.36" x2="35.56" y2="86.36" width="0.1524" layer="91"/>
</segment>
<segment>
<pinref part="MS1" gate="G$1" pin="VBAT"/>
<pinref part="SUPPLY9" gate="+5V" pin="+5V"/>
<wire x1="190.5" y1="76.2" x2="195.58" y2="76.2" width="0.1524" layer="91"/>
<wire x1="195.58" y1="76.2" x2="195.58" y2="78.74" width="0.1524" layer="91"/>
</segment>
<segment>
<pinref part="SUPPLY7" gate="+5V" pin="+5V"/>
<pinref part="JP2" gate="A" pin="3"/>
<wire x1="116.84" y1="35.56" x2="116.84" y2="33.02" width="0.1524" layer="91"/>
<wire x1="116.84" y1="33.02" x2="134.62" y2="33.02" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$1" class="0">
<segment>
<pinref part="C1" gate="G$1" pin="1"/>
<pinref part="U$1" gate="G$1" pin="P$4"/>
<wire x1="15.24" y1="81.28" x2="35.56" y2="81.28" width="0.1524" layer="91"/>
<pinref part="S1" gate="G$1" pin="2"/>
<wire x1="7.62" y1="81.28" x2="15.24" y2="81.28" width="0.1524" layer="91"/>
<junction x="15.24" y="81.28"/>
</segment>
</net>
<net name="9DOF_SDA" class="0">
<segment>
<pinref part="U$1" gate="G$1" pin="P$9"/>
<wire x1="35.56" y1="68.58" x2="33.02" y2="68.58" width="0.1524" layer="91"/>
<label x="20.32" y="68.58" size="1.778" layer="95"/>
</segment>
<segment>
<pinref part="J1" gate="G$1" pin="2"/>
<wire x1="127" y1="17.78" x2="119.38" y2="17.78" width="0.1524" layer="91"/>
<label x="106.68" y="17.78" size="1.778" layer="95"/>
</segment>
</net>
<net name="9DOF_SCL" class="0">
<segment>
<pinref part="U$1" gate="G$1" pin="P$10"/>
<wire x1="35.56" y1="66.04" x2="33.02" y2="66.04" width="0.1524" layer="91"/>
<label x="20.32" y="66.04" size="1.778" layer="95"/>
</segment>
<segment>
<pinref part="J1" gate="G$1" pin="1"/>
<wire x1="127" y1="20.32" x2="119.38" y2="20.32" width="0.1524" layer="91"/>
<label x="106.68" y="20.32" size="1.778" layer="95"/>
</segment>
</net>
<net name="ESP8266_RX" class="0">
<segment>
<pinref part="U$1" gate="G$1" pin="P$13"/>
<wire x1="35.56" y1="58.42" x2="33.02" y2="58.42" width="0.1524" layer="91"/>
<label x="17.78" y="58.42" size="1.778" layer="95"/>
</segment>
<segment>
<pinref part="MS1" gate="G$1" pin="GPIORX"/>
<wire x1="147.32" y1="53.34" x2="144.78" y2="53.34" width="0.1524" layer="91"/>
<label x="129.54" y="53.34" size="1.778" layer="95"/>
</segment>
</net>
<net name="ESP8266_TX" class="0">
<segment>
<pinref part="U$1" gate="G$1" pin="P$14"/>
<wire x1="35.56" y1="55.88" x2="33.02" y2="55.88" width="0.1524" layer="91"/>
<label x="17.78" y="55.88" size="1.778" layer="95"/>
</segment>
<segment>
<pinref part="MS1" gate="G$1" pin="GPIOTX"/>
<wire x1="147.32" y1="50.8" x2="144.78" y2="50.8" width="0.1524" layer="91"/>
<label x="129.54" y="50.8" size="1.778" layer="95"/>
</segment>
</net>
<net name="DHB-10_RX" class="0">
<segment>
<pinref part="U$1" gate="G$1" pin="P$28"/>
<wire x1="111.76" y1="58.42" x2="114.3" y2="58.42" width="0.1524" layer="91"/>
<label x="114.3" y="58.42" size="1.778" layer="95"/>
</segment>
<segment>
<pinref part="JP1" gate="A" pin="3"/>
<wire x1="177.8" y1="15.24" x2="170.18" y2="15.24" width="0.1524" layer="91"/>
<label x="157.48" y="15.24" size="1.778" layer="95"/>
</segment>
</net>
<net name="DHB-10_TX" class="0">
<segment>
<pinref part="U$1" gate="G$1" pin="P$27"/>
<wire x1="111.76" y1="55.88" x2="114.3" y2="55.88" width="0.1524" layer="91"/>
<label x="114.3" y="55.88" size="1.778" layer="95"/>
</segment>
<segment>
<pinref part="JP1" gate="A" pin="2"/>
<wire x1="177.8" y1="17.78" x2="170.18" y2="17.78" width="0.1524" layer="91"/>
<label x="157.48" y="17.78" size="1.778" layer="95"/>
</segment>
</net>
<net name="N$20" class="0">
<segment>
<pinref part="JP1" gate="A" pin="1"/>
<wire x1="177.8" y1="20.32" x2="170.18" y2="20.32" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$2" class="0">
<segment>
<pinref part="J1" gate="G$1" pin="4"/>
<wire x1="127" y1="12.7" x2="121.92" y2="12.7" width="0.1524" layer="91"/>
<pinref part="JP2" gate="A" pin="2"/>
<wire x1="134.62" y1="35.56" x2="121.92" y2="35.56" width="0.1524" layer="91"/>
<wire x1="121.92" y1="35.56" x2="121.92" y2="12.7" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$3" class="0">
<segment>
<pinref part="JP2" gate="A" pin="1"/>
<wire x1="134.62" y1="38.1" x2="124.46" y2="38.1" width="0.1524" layer="91"/>
<wire x1="124.46" y1="38.1" x2="124.46" y2="86.36" width="0.1524" layer="91"/>
<pinref part="U$1" gate="G$1" pin="P$39"/>
<wire x1="124.46" y1="86.36" x2="111.76" y2="86.36" width="0.1524" layer="91"/>
</segment>
</net>
</nets>
</sheet>
</sheets>
</schematic>
</drawing>
</eagle>
