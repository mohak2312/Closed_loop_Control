`timescale 1ns / 1ps
// hardwarwe detection logic to detect signal of Hall sensor output

// Author:	Mohak Patel and Archit Tatwawadi
// Date: 12-May
//
// Description:
// ------------
// This hardware module provides a counter values of of high and low signal.
// input- Hall sensor signal, clock_3 and reset.
// outputs are 32 bit counters that provide time interval of high and low signal.
//
//It use the edge detection logic to count the time interval for high and low signal
////////////////////////////////////////////////////////////////////////////////////////////////

module  n4fpga_PWM_detection (
    // I/O port declaration				
	input 					clock_3,                //100Mhz clock		
	input 					SA,                     //  signal of Hall sensor output
	input                   Reset,		            // system reset to reset the counter
	output reg	[31:0]		high_counter,		    // counter to calculate the time for how long signal stay high
	output reg	[31:0]		low_counter             // counter to calculate the time for how long signal stay low
	);
	
	//inner variable		
	reg			[31:0]		counter1=32'b0;          //counter to calculate the time for how long signal stay high
	reg			[31:0]		counter2=32'b0;          //counter to calculate the time for how long signal stay low
	reg 					lastval; 		         //Store the previous PWM pulse for High or low pulse detection

always@(posedge clock_3) begin
if(Reset)begin                                       //check for system reset
    counter1<=32'b0;                                 // clear the high counter
    counter2<=32'b0;                                 // clear the low counter
    high_counter<=32'b0;                            // clear the output register of high counter
    low_counter<=32'b0;                             // clear the output register of low counter
    lastval<=1'b0;                                  // clear the previous PWM signal state
    end
    
	if (SA == 1'b1) begin                         //check for signal is high		
			if (lastval == 1'b0)                        // if last PWM signal is not equal to the current PWM 
			begin 
					high_counter<=counter1;               // Store the counter value of High signal
					counter1<=32'b0;                       //clear the high counter
					lastval<=1'b1;                         //store the current PWM signal to Previous PWM signal
			end
			else begin
					counter1 <= counter1 + 1'b1;           // else increment the high counter
					lastval<=1'b1;                         //store the current PWM signal to Previous PWM signal
			end

	end

	else if(SA == 1'b0)	begin                          //check for signal is low
			if(lastval==1'b1)                              // if last PWM signal is not equal to the current PWM 
			begin
				low_counter <= counter2;                    // Store the counter value of low signal
				counter2<=32'b0;                            //clear the low counter
				lastval<=1'b0;                              //store the current PWM signal to Previous PWM signal
			end
			else begin
				counter2=counter2+1'b1;                     // else increment the low counter
				lastval=1'b0;                               //store the current PWM signal to Previous PWM signal
			end	

		end
	
end	
	
endmodule