// FPGA Bot : Task 1 D : PWM Generator
/*
Instructions
-------------------
Students are not allowed to make any changes in the Module declaration.
This file is used to design a module which will scale down the 50Mhz Clock Frequency to 1Mhz and perform Pulse Width Modulation on it.

Recommended Quartus Version : 19.1
The submitted project file must be 19.1 compatible as the evaluation will be done on Quartus Prime Lite 19.1.

Warning: The error due to compatibility will not be entertained.
-------------------
*/

//PWM Generator
//Inputs : Clk, DUTY_CYCLE
//Output : PWM_OUT

//////////////////DO NOT MAKE ANY CHANGES IN MODULE//////////////////

module SM1068_PWM_Generator(
 
	input clk,             // Clock input
	//input [7:0]DUTY_CYCLE, // Input Duty Cycle
	output PWM_OUT         // Output PWM
);
 
////////////////////////WRITE YOUR CODE FROM HERE////////////////////
reg [15:0]clk_cnt=0;
reg div_out;
reg [15:0]on_time;
//reg [7:0] DUTY_CYCLE=50;

always @(*) begin
on_time<=100000;
end

always @(posedge clk) begin
clk_cnt<=(clk_cnt<102040)?clk_cnt+1:0;
div_out<=(clk_cnt<on_time)?0:1;

end

assign PWM_OUT=div_out;
////////////////////////YOUR CODE ENDS HERE//////////////////////////
endmodule
///////////////////////////////MODULE ENDS///////////////////////////