// SM : Task 2 B : UART
/*
Instructions
-------------------
Students are not allowed to make any changes in the Module declaration.
This file is used to design UART Transmitter.

Recommended Quartus Version : 19.1
The submitted project file must be 19.1 compatible as the evaluation will be done on Quartus Prime Lite 19.1.

Warning: The error due to compatibility will not be entertained.
-------------------
*/

//UART Transmitter design
//Input   : clk_50M : 50 MHz clock
//Output  : tx : UART transmit output

//////////////////DO NOT MAKE ANY CHANGES IN MODULE//////////////////
module SM1068_uart(
	input clk_50M,	//50 MHz clock
	output tx,	//UART transmit output
	input [7:0] data_in,
	input en_tri,
	input[1:0] buf_sel,
	input [3:0] node_num,
	input [1:0] p_num
	
);
////////////////////////WRITE YOUR CODE FROM HERE////////////////////
parameter IDLE=0,START=1,DATA=2,STOP=3;

parameter clocks_per_bit=434;
parameter data_size_in_bytes=4;

reg[13:0] clk_count;
reg[2:0] state;

reg[7:0] tx_data [11:0];
reg[7:0] pick_data [12:0];
reg[7:0] deposit_data [12:0];

reg en=0;
reg [16:0] uen_counter;
reg uen_rst=0;

reg [7:0] data;
reg [3:0] len;





always@(posedge clk_50M) begin
tx_data[8]<=data_in;
pick_data[9]<=data_in;
deposit_data[9]<=data_in;

end

reg[3:0] i;

reg tx_flag;

reg tx_act;
reg tx_inact;

reg tx_line;

reg [4:0]buf_cnt;

initial begin
tx_act=0;
tx_inact=0;
state=IDLE;
clk_count=0;

uen_counter=0;

tx_data[0]=83;
tx_data[1]=73;
tx_data[2]=45;
tx_data[3]=83;
tx_data[4]=73;
tx_data[5]=78;
tx_data[6]=49;
tx_data[7]=45;
tx_data[8]=68;
tx_data[9]=45;
tx_data[10]=35;
tx_data[11]=13;

pick_data[0]=83;
pick_data[1]=45;
pick_data[2]=80;
pick_data[3]=45;
pick_data[4]=68;
pick_data[5]=90;
pick_data[6]=78;
pick_data[7]=49;
pick_data[8]=45;
pick_data[9]=78;
pick_data[10]=45;
pick_data[11]=35;
pick_data[12]=13;

deposit_data[0]=83;
deposit_data[1]=45;
deposit_data[2]=68;
deposit_data[3]=45;
deposit_data[4]=68;
deposit_data[5]=90;
deposit_data[6]=78;
deposit_data[7]=49;
deposit_data[8]=45;
deposit_data[9]=78;
deposit_data[10]=45;
deposit_data[11]=35;
deposit_data[12]=13;

i=0;
tx_flag=0;
tx_line=1;
end


always@(*) begin
	case(buf_sel)
				0:begin
						len<=13;
					end
				1: begin
						len<=13;
					end
				2: begin
						len<=12;
					end
				3:begin
						len<=0;
					end
			endcase
end

always@(posedge clk_50M) begin
		uen_rst<=(buf_cnt < len)?0:1;
		end

always@(posedge en_tri or posedge uen_rst) begin
		if(uen_rst) begin
				en<=0;
				end
		else begin
				en<=1;
				end
end

always @(posedge clk_50M) begin
	if(~en &(~tx_act))begin
				if(buf_cnt==len) begin
					buf_cnt<=0;
					end
			end
	else if(tx_flag==1) begin			
				
				if(buf_sel==2'b10) begin
						data<=tx_data[buf_cnt];
					end
				else if(buf_sel==2'b00) begin
						data<=pick_data[buf_cnt];
					end
				else if(buf_sel==2'b01) begin
						data<=deposit_data[buf_cnt];
					end
		buf_cnt<=buf_cnt+1;
		end
	else begin
		data<=data;
		buf_cnt<=buf_cnt;
		end
end


always@(posedge clk_50M) begin

    case (state)
			IDLE:begin
				  tx_line<=1;
				  if(clk_count<clocks_per_bit-1) begin
					  		clk_count<=clk_count+1;
							state<=IDLE;
							
						  	end
				  else if(en) begin
				         state<=START;
							clk_count<=0;
							tx_act<=1;
							tx_inact<=0;
							end
				  else begin
							state<=IDLE;
					 		clk_count<=0;
					   	end
				  end
			START:begin
					tx_line<=0;
					if(clk_count<clocks_per_bit-1) begin
							clk_count<=clk_count+1;
							state<=START;
							end
					else begin
							state<=DATA;
							tx_flag<=1;
							clk_count<=0;
						   end
					end
			DATA:begin
					tx_flag<=0;
					tx_line<=data[i];
				  if(clk_count<clocks_per_bit-1) begin
							clk_count<=clk_count+1;
							state<=DATA;
							end
				  else begin 
						clk_count<=0;
						if(i<7) begin
						   i<=i+1;
							state<=DATA;
							
							end
						else begin
							i<=0;
							//data_count<=data_count+1;
							state<=STOP;
							
							
								end
						end
					
				  end
			STOP:begin
				  tx_line<=1;
				  
				  if(clk_count<clocks_per_bit-1) begin
					  		clk_count<=clk_count+1;
							state<=STOP;
							tx_act<=0;
							tx_inact<=1;
						  	end
				  else begin
							
							state<=IDLE;
							clk_count<=0;
							end
				  end
	 endcase

end

assign tx=tx_line;

////////////////////////YOUR CODE ENDS HERE//////////////////////////
endmodule
///////////////////////////////MODULE ENDS///////////////////////////