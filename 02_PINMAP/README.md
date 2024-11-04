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

<table>
<tr>
	<td><img src="../images/pinmap/con_left_motor.png" alt="left motor connector"></td>
	<td>
	    <table>
	    	<thead>
	    		<td>Pin No</td>
	    		<td>Description</td>
	    	</thead>
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
	    </table>
	</td>
</tr>
<table>

<div style="display: flex; flex-direction: row;">
  <div style="flex: 1;">

  </div>
  <div style="flex: 2; padding-left:20px;">
  	
  </div>
</div>

## Right Motor Connector

<div style="display: flex; flex-direction: row;">
  <div style="flex: 1;">
  	<img src="../images/pinmap/con_right_motor.png" alt="right motor connector">
  </div>	
  <div style="flex: 2; padding-left:20px;">
    <table>
    	<thead>
    		<td>Pin No</td>
    		<td>Description</td>
    	</thead>
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
    </table>
  </div>
</div>

## Servo Connector

<div style="display: flex; flex-direction: row;">
  <div style="flex: 1;">
  	<img src="../images/pinmap/con_servo.png" alt="">
  </div>	
  <div style="flex: 2; padding-left:20px;">
    <table>
    	<thead>
    		<td>Pin No</td>
    		<td>Description</td>
    	</thead>
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
    </table>
  </div>
</div>

## AUX Power Port

<div style="display: flex; flex-direction: row;">
  <div style="flex: 1;">
  	<img src="../images/pinmap/con_power_aux.png" alt="">
  </div>	
  <div style="flex: 2; padding-left:20px;">
    <table>
    	<thead>
    		<td>Pin No</td>
    		<td>Description</td>
    	</thead>
    	<tr>
    		<td>1</td>
    		<td>PWR AUX (5V)</td>
    	</tr> 
    	<tr>
    		<td>2</td>
    		<td>Ground</td>
    	</tr>    	 	   	    	    	    	   	 	
    </table>
  </div>
</div>

## Power Control Port

<div style="display: flex; flex-direction: row;">
  <div style="flex: 1;">
  	<img src="../images/pinmap/con_power_control.png" alt="">
  </div>	
  <div style="flex: 2; padding-left:20px;">
    <table>
    	<thead>
    		<td>Pin No</td>
    		<td>Description</td>
    	</thead>
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
    </table>
  </div>
</div>

## Communications Port

<div style="display: flex; flex-direction: row;">
  <div style="flex: 1;">
  	<img src="../images/pinmap/con_comm.png" alt="">
  </div>	
  <div style="flex: 2; padding-left:20px;">
    <table>
    	<thead>
    		<td>Pin No</td>
    		<td>Description</td>
    	</thead>
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
    </table>
  </div>
</div>

## QWIC A Port

<div style="display: flex; flex-direction: row;">
  <div style="flex: 1;">
  	<img src="../images/pinmap/con_qwic_a.png" alt="">
  </div>	
  <div style="flex: 2; padding-left:20px;">
    <table>
    	<thead>
    		<td>Pin No</td>
    		<td>Description</td>
    	</thead>
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
    </table>
  </div>
</div>

## QWIC B Port

<div style="display: flex; flex-direction: row;">
  <div style="flex: 1;">
  	<img src="../images/pinmap/con_qwic_b.png" alt="">
  </div>	
  <div style="flex: 2; padding-left:20px;">
    <table>
    	<thead>
    		<td>Pin No</td>
    		<td>Description</td>
    	</thead>
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
    </table>
  </div>
</div>

## SPI Port

<div style="display: flex; flex-direction: row;">
  <div style="flex: 1;">
  	<img src="../images/pinmap/con_spi.png" alt="">
  </div>	
  <div style="flex: 2; padding-left:20px;">
    <table>
    	<thead>
    		<td>Pin No</td>
    		<td>Description</td>
    	</thead>
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
    </table>
  </div>
</div>

## Serial Port

<div style="display: flex; flex-direction: row;">
  <div style="flex: 1;">
  	<img src="../images/pinmap/con_serial.png" alt="">
  </div>	
  <div style="flex: 2; padding-left:20px;">
    <table>
    	<thead>
    		<td>Pin No</td>
    		<td>Description</td>
    	</thead>
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
    </table>
  </div>
</div>

## Power Connector

<div style="display: flex; flex-direction: row;">
  <div style="flex: 1;">
  	<img src="../images/pinmap/con_xt30.png" alt="">
  </div>	
  <div style="flex: 2; padding-left:20px;">
    <table>
    	<thead>
    		<td>Pin No</td>
    		<td>Description</td>
    	</thead>
    	<tr>
    		<td>1</td>
    		<td>Power Input</td>
    	</tr> 
    	<tr>
    		<td>2</td>
    		<td>Ground</td>
    	</tr>    	 	   	    	    	    	   	 	
    </table>
  </div>
</div>

## Battery

<div style="display: flex; flex-direction: row;">
  <div style="flex: 1;">
  	<img src="../images/pinmap/con_battery.png" alt="">
  </div>	
  <div style="flex: 2; padding-left:20px;">
  	Use standard CR1225 Battery
  </div>
</div>


__Next Chapter:__ [Connecting ROSRider to Host Computer](../03_CONNECT/README.md)