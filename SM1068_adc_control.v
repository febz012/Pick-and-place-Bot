// SM : Task 2 A : ADC
/*
Instructions
-------------------
Students are not allowed to make any changes in the Module declaration.
This file is used to design ADC Controller.

Recommended Quartus Version : 19.1
The submitted project file must be 19.1 compatible as the evaluation will be done on Quartus Prime Lite 19.1.

Warning: The error due to compatibility will not be entertained.
-------------------
*/

//ADC Controller design
//Inputs  : clk_50 : 50 MHz clock, dout : digital output from ADC128S022 (serial 12-bit)
//Output  : adc_cs_n : Chip Select, din : Ch. address input to ADC128S022, adc_sck : 2.5 MHz ADC clock,
//				d_out_ch5, d_out_ch6, d_out_ch7 : 12-bit output of ch. 5,6 & 7,
//				data_frame : To represent 16-cycle frame (optional)

//////////////////DO NOT MAKE ANY CHANGES IN MODULE//////////////////
module SM1068_adc_control(
	input  clk_50,				//50 MHz clock
	input  dout,				//digital output from ADC128S022 (serial 12-bit)
	output adc_cs_n,			//ADC128S022 Chip Select
	output din,					//Ch. address input to ADC128S022 (serial)
	output adc_sck,			//2.5 MHz ADC clock
	output [11:0] d_out_ch5,	//12-bit output of ch. 5 (parallel)
	output [11:0] d_out_ch6,	//12-bit output of ch. 6 (parallel)
	output [11:0] d_out_ch7,	//12-bit output of ch. 7 (parallel)
	output [1:0] data_frame,	//To represent 16-cycle frame (optional)
	output [2:0] direction
);
	
////////////////////////WRITE YOUR CODE FROM HERE////////////////////
parameter frame1=1,frame2=2,frame3=3;

reg adc_clk=1;
reg di=0;
reg[2:0] ch5=5,ch6=6,ch7=7;

reg[5:0] clk_cnt=0;

reg [5:0] counter=0;
reg [1:0] state=frame1;

reg adc_cs=0;
reg [1:0]dfra=0;
reg [2:0]dir_buf=0;
reg [11:0]d_ch7=0,df_ch7=0;
reg [11:0]d_ch6=0,df_ch6=0;
reg [11:0]d_ch5=0,df_ch5=0;

reg[21:0] en_counter=0;
reg cl_en = 1;

always @(posedge clk_50) begin
en_counter<=(en_counter<499999)?en_counter+1:500010;
cl_en<=(en_counter<499999)?1:0;
end

always @(negedge clk_50) begin
	adc_clk <= (counter < 10)?0:1;
	counter <= (counter < 19)?(counter+1):0;
end

assign adc_sck=adc_clk | cl_en;

always @(negedge adc_sck) begin
		clk_cnt <= (clk_cnt<16)?(clk_cnt+1):1;
      end
		
always@(negedge adc_sck) begin

  case(state)
	  
	  frame1:begin
					 dfra <= 1;
			       df_ch6<=d_ch6;
			        if(clk_cnt>4)begin
							di<=0;
							end 
					  else if(clk_cnt>1) begin
							di<=ch5[4-clk_cnt];
							end
					  else begin
							di<=0; 
							end
				end
				
	  frame2:begin 
					  dfra<=2;
					  df_ch7<=d_ch7;
				     if(clk_cnt >4)begin
							di<=0;
							end 
					  else if(clk_cnt >1)begin
							di<=ch6[4-clk_cnt];
							end 
					  else begin
							di<=0; 
							end
				end
			
	  frame3:begin
					  dfra<=3;
					  df_ch5 <= d_ch5;
					  if(clk_cnt > 4)begin
							di<=0;
							end 
					  else if(clk_cnt > 1)begin
							di <= ch7[4-clk_cnt];
							end 
					  else begin
							di <= 0; 
							end
				end 
	  
  endcase
end

always @(posedge adc_sck) begin
		case(state) 
			frame1:begin
				     if(clk_cnt == 16) begin
							d_ch7[16 - clk_cnt] <= dout;
							state <= frame2;
							end
					  else if(clk_cnt > 4) begin
							d_ch7[16 - clk_cnt] <= dout;						
							end
					 end
		   frame2:begin
				     if(clk_cnt == 16) begin
							d_ch5[16 - clk_cnt] <= dout;
							state <= frame3;
							end
					  else if(clk_cnt > 4) begin
							d_ch5[16 - clk_cnt] <= dout;						
							end
					 end
			frame3:begin
				     if(clk_cnt == 16) begin
							d_ch6[16 - clk_cnt] <= dout;
							state <= frame1;
							end
					  else if(clk_cnt > 4) begin
							d_ch6[16 - clk_cnt] <= dout;						
							end
					 end
	   endcase		
end

always@(*) begin
	if(df_ch7<200) begin
			dir_buf[0]<=1;
			end
	else begin
		  dir_buf[0]<=0;
	     end
	if(df_ch6<200) begin
			dir_buf[1]<=1;
			end
	else begin
		  dir_buf[1]<=0;
	     end
	if(df_ch5<200) begin
			dir_buf[2]<=1;
			end
	else begin
		  dir_buf[2]<=0;
	     end
end

assign din=di;
assign adc_cs_n=adc_cs;

assign data_frame=dfra;

assign direction=dir_buf;

assign d_out_ch7=df_ch7;
assign d_out_ch6=df_ch6;
assign d_out_ch5=df_ch5;
////////////////////////YOUR CODE ENDS HERE//////////////////////////
endmodule
///////////////////////////////MODULE ENDS///////////////////////////

