---
layout: default
title_url: /02_PINMAP/README.html
title: "Pinmap"
description: "Pinmap and Connections"
---

## Board Port Diagram

ROSRider card is equipped with number of ports and connectors, for connecting to encoder motors, and other peripherals.  
Pinmaps of each connector and port will be illustrated in detail.  

Below is a diagram of ports of ROSRider control card:  
  
[![ROSRider Pinmap](../images/ROSRider4D_portmap.png)](https://acada.dev/products)

<style type="text/css">

  table#portmap tbody tr {
  	border: none;
  }

  table.pinmap tbody tr td {
  	border: 2px solid black;
  }

  table.pinmap thead th {
  	border: 2px solid black;
  }

</style>

<table id="portmap">
<tbody>
<tr>
	<td colspan="2">
		header
	</td>
</tr>
<tr>
	<td style="border:none;"><img src="../images/pinmap/con_left_motor.png" alt="left motor connector"></td>
	<td style="border:none; vertical-align:middle;">
	    <table class="pinmap">
	    	<thead>
	    		<th>Pin Number</th>
	    		<th>Description</th>
	    	</thead>
	    	<tbody>
		    	<tr>
		    		<td>1</td>
		    		<td>Motor Negative MT1_M2</td>
		    	</tr>
		    	<tr>
		    		<td>2</td>
		    		<td>Motor Positive MT1_M1</td>
		    	</tr>   
		    	<tr>
		    		<td>3</td>
		    		<td>3.3V</td>
		    	</tr>   
		    	<tr>
		    		<td>4</td>
		    		<td>Encoder Phase B</td>
		    	</tr>  
		    	<tr>
		    		<td>5</td>
		    		<td>Encoder Phase A</td>
		    	</tr>  
		    	<tr>
		    		<td>6</td>
		    		<td>Ground</td>
		    	</tr>
	    	</tbody>     	    	    	   	 	
	    </table>
	</td>
</tr>
<tr></tr>
<tr>
	<td style="border:none;"><img src="../images/pinmap/con_right_motor.png" alt="right motor connector"></td>
	<td style="border:none; vertical-align:middle;">
	    <table class="pinmap">
	    	<thead>
	    		<th>Pin Number</th>
	    		<th>Description</th>
	    	</thead>
	    	<tbody>
		    	<tr>
		    		<td>6</td>
		    		<td>Ground</td>
		    	</tr>
		    	<tr>
		    		<td>5</td>
		    		<td>Encoder Phase A</td>
		    	</tr>   
		    	<tr>
		    		<td>4</td>
		    		<td>Encoder Phase B</td>
		    	</tr>   
		    	<tr>
		    		<td>3</td>
		    		<td>3.3V</td>
		    	</tr>  
		    	<tr>
		    		<td>2</td>
		    		<td>Motor Positive MT1_M1</td>
		    	</tr>  
		    	<tr>
		    		<td>1</td>
		    		<td>Motor Negative MT1_M2</td>
		    	</tr>
	    	</tbody>  	    	    	   	 	
	    </table>		
	</td>
</tr>
<tr></tr>
<tr>
	<td style="border:none;"><img src="../images/pinmap/con_servo.png" alt="servo connector"></td>
	<td style="border:none; vertical-align:middle;">
	    <table class="pinmap">
	    	<thead>
	    		<th>Pin Number</th>
	    		<th>Description</th>
	    	</thead>
	    	<tbody>
		    	<tr>
		    		<td>1</td>
		    		<td>Ground</td>
		    	</tr> 
		    	<tr>
		    		<td>2</td>
		    		<td>Ground</td>
		    	</tr>   
		    	<tr>
		    		<td>3</td>
		    		<td>AUX Power</td>
		    	</tr> 
		    	<tr>
		    		<td>4</td>
		    		<td>AUX Power</td>
		    	</tr>
		    	<tr>
		    		<td>5</td>
		    		<td>Servo1 PWM</td>
		    	</tr>   
		    	<tr>
		    		<td>6</td>
		    		<td>Servo2 PWM</td>
		    	</tr>
	    	</tbody>   	 	   	    	    	    	   	 	
	    </table>		
	</td>
</tr>
<tr></tr>
<tr>
	<td style="border:none;"><img src="../images/pinmap/con_power_aux.png" alt="power aux connector"></td>
	<td style="border:none; vertical-align:middle;">
	    <table class="pinmap">
	    	<thead>
	    		<th>Pin Number</th>
	    		<th>Description</th>
	    	</thead>
	    	<tbody>
		    	<tr>
		    		<td>1</td>
		    		<td>PWR AUX (5V)</td>
		    	</tr> 
		    	<tr>
		    		<td>2</td>
		    		<td>Ground</td>
		    	</tr>
	    	</tbody> 	 	   	    	    	    	   	 	
	    </table>		
	</td>
</tr>
<tr></tr>
<tr>
	<td style="border:none;"><img src="../images/pinmap/con_power_control.png" alt="power control port"></td>
	<td style="border:none; vertical-align:middle;">
	    <table class="pinmap">
	    	<thead>
	    		<th>Pin Number</th>
	    		<th>Description</th>
	    	</thead>
	    	<tbody>
		    	<tr>
		    		<td>1</td>
		    		<td>Ground</td>
		    	</tr> 
		    	<tr>
		    		<td>2</td>
		    		<td>Bootloader Button</td>
		    	</tr>   
		    	<tr>
		    		<td>3</td>
		    		<td>Power On</td>
		    	</tr>  
		    	<tr>
		    		<td>4</td>
		    		<td>Internal Reset</td>
		    	</tr>
	    	</tbody>   	   	 	 	   	    	    	    	   	 	
	    </table>		
	</td>
</tr>
<tr></tr>
<tr>
	<td style="border:none;"><img src="../images/pinmap/con_comm.png" alt="communications port"></td>
	<td style="border:none; vertical-align:middle;">
	    <table class="pinmap">
	    	<thead>
	    		<th>Pin Number</th>
	    		<th>Description</th>
	    	</thead>
	    	<tbody>
		    	<tr>
		    		<td>1</td>
		    		<td>Ground</td>
		    	</tr> 
		    	<tr>
		    		<td>2</td>
		    		<td>Reset</td>
		    	</tr>  
		    	<tr>
		    		<td>3</td>
		    		<td>SCL0</td>
		    	</tr> 
		    	<tr>
		    		<td>4</td>
		    		<td>SDA0</td>
		    	</tr>  
		    	<tr>
		    		<td>5</td>
		    		<td>TX0</td>
		    	</tr> 
		    	<tr>
		    		<td>6</td>
		    		<td>RX0</td>
		    	</tr>
	    	</tbody>     	    	  	 	   	    	    	    	   	 	
	    </table>		
	</td>
</tr>
<tr></tr>
<tr>
	<td style="border:none;"><img src="../images/pinmap/con_qwic_a.png" alt="qwic port a"></td>
	<td style="border:none; vertical-align:middle;">
		<table class="pinmap">
	    	<thead>
	    		<th>Pin Number</th>
	    		<th>Description</th>
	    	</thead>
	    	<tbody>
		    	<tr>
		    		<td>1</td>
		    		<td>Ground</td>
		    	</tr> 
		    	<tr>
		    		<td>2</td>
		    		<td>VCC</td>
		    	</tr>   
		    	<tr>
		    		<td>3</td>
		    		<td>SDA0</td>
		    	</tr>    
		    	<tr>
		    		<td>4</td>
		    		<td>SCL0</td>
		    	</tr>
	    	</tbody>    	  	 	 	   	    	    	    	   	 	
	    </table>
	</td>
</tr>
<tr></tr>
<tr>
	<td style="border:none;"><img src="../images/pinmap/con_qwic_b.png" alt="qwic port b"></td>
	<td style="border:none; vertical-align:middle;">
	    <table class="pinmap">
	    	<thead>
	    		<th>Pin Number</th>
	    		<th>Description</th>
	    	</thead>
	    	<tbody>
		    	<tr>
		    		<td>1</td>
		    		<td>Ground</td>
		    	</tr> 
		    	<tr>
		    		<td>2</td>
		    		<td>VCC</td>
		    	</tr>   
		    	<tr>
		    		<td>3</td>
		    		<td>SDA0</td>
		    	</tr>    
		    	<tr>
		    		<td>4</td>
		    		<td>SCL0</td>
		    	</tr>
	    	</tbody>  	 	   	    	    	    	   	 	
	    </table>		
	</td>
</tr>
<tr></tr>
<tr>
	<td style="border:none;"><img src="../images/pinmap/con_spi.png" alt="spi port"></td>
	<td style="border:none; vertical-align:middle;">
	    <table class="pinmap">
	    	<thead>
	    		<th>Pin Number</th>
	    		<th>Description</th>
	    	</thead>
	    	<tbody>
		    	<tr>
		    		<td>1</td>
		    		<td>Ground</td>
		    	</tr> 
		    	<tr>
		    		<td>2</td>
		    		<td>SSI0RX</td>
		    	</tr>   
		    	<tr>
		    		<td>3</td>
		    		<td>SSI0FSS</td>
		    	</tr> 
		    	<tr>
		    		<td>4</td>
		    		<td>SSI0TX</td>
		    	</tr>   
		    	<tr>
		    		<td>5</td>
		    		<td>SSI0CLK</td>
		    	</tr> 
		    	<tr>
		    		<td>6</td>
		    		<td>3.3V</td>
		    	</tr>
	    	</tbody>      	    	 	 	   	    	    	    	   	 	
	    </table>		
	</td>
</tr>
<tr></tr>
<tr>
	<td style="border:none;"><img src="../images/pinmap/con_serial.png" alt="serial port"></td>
	<td style="border:none; vertical-align:middle;">
	    <table class="pinmap">
	    	<thead>
	    		<th>Pin Number</th>
	    		<th>Description</th>
	    	</thead>
	    	<tbody>
		    	<tr>
		    		<td>1</td>
		    		<td>Ground</td>
		    	</tr> 
		    	<tr>
		    		<td>2</td>
		    		<td>RX0</td>
		    	</tr>   
		    	<tr>
		    		<td>3</td>
		    		<td>TX0</td>
		    	</tr> 
		     	<tr>
		    		<td>4</td>
		    		<td>Ground</td>
		    	</tr>
	    	</tbody>  	    	 	 	   	    	    	    	   	 	
	    </table>		
	</td>
</tr>
<tr></tr>
<tr>
	<td style="border:none;"><img src="../images/pinmap/con_xt30.png" alt="power connector"></td>
	<td style="border:none; vertical-align:middle;">
	    <table class="pinmap">
	    	<thead>
	    		<th>Pin Number</th>
	    		<th>Description</th>
	    	</thead>
	    	<tbody>
		    	<tr>
		    		<td>1</td>
		    		<td>Power Input</td>
		    	</tr> 
		    	<tr>
		    		<td>2</td>
		    		<td>Ground</td>
		    	</tr>
	    	</tbody> 	 	   	    	    	    	   	 	
	    </table>		
	</td>
</tr>
<tr></tr>
<tr>
	<td style="border:none;"><img src="../images/pinmap/con_battery.png" alt="cr1225 battery"></td>
	<td style="border:none; vertical-align:middle;">Use standard CR1225 Battery</td>
</tr>
<tr></tr>
</tbody>
</table>

## Left Motor Connector
## Right Motor Connector
## Servo Connector
## AUX Power Port
## Power Control Port
## Communications Port
## QWIC A Port
## QWIC B Port
## SPI Port
## Serial Port
## Power Connector
## Battery

__Next Chapter:__ [Connecting ROSRider to Host Computer](../03_CONNECT/README.md)