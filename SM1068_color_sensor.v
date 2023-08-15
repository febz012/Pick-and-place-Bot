module SM1068_color_sensor(
	input clk_color_50,
	input out,
	output s0,
	output s1,
	output s2,
	output s3,
	output [2:0]redl,
	output [2:0]greenl,
	output [2:0]bluel,
	output det,
	input clr_led,
	output[2:0] led_st,
	input all_led
);

parameter red=0,green=1,blue=2;
parameter red_led=0,green_led=1,blue_led=2,no_led=3,blink_led=4;
reg[2:0] led_state=no_led;
reg [23:0] led_counter=0;

reg [1:0] state=red;
reg [2:0] led_stb=0;
reg det_buf=0;
reg s0_buf=1;
reg s1_buf=1;
reg s2_buf=0;
reg s3_buf=0;

reg r=0;
reg g=1;
reg b=0;

reg [2:0]red_buf=0;
reg [2:0]green_buf=0;
reg [2:0]blue_buf=0;

reg s_clk;

reg[21:0] counter=0;
reg[21:0] rst_counter=0;
reg[21:0] out_counter=0;
reg rst=0;

reg[21:0] red_counter=0;
reg[21:0] green_counter=0;
reg[21:0] blue_counter=0;

reg fla=0;
reg[35:0] fla_cnt=0;

always@(posedge clk_color_50) begin
fla_cnt<=(fla_cnt<500000000)?fla_cnt+1:500001000;
fla<=(fla_cnt<500000000)?0:1;
end



always @(posedge clk_color_50) begin
counter<=(counter<499999)?counter+1:0;
s_clk<=(counter<250000)?1:0;

rst_counter<=(rst_counter<500000)?rst_counter+1:0;
rst<=(rst_counter<500000)?0:1;
end

always @(posedge out or posedge rst) begin
	if(rst==1) begin
			out_counter<=0;
			end
	else begin
		  out_counter<=out_counter+1;	
		  end
end

always @(posedge s_clk) begin

	case(state)
		red:begin
		    s2_buf<=0;
			 s3_buf<=0;
			 blue_counter<=out_counter;
			 state<=green;
			 end
		
		green:begin
				s2_buf<=1;
			   s3_buf<=1;
				red_counter<=out_counter;
				state<=blue;
			   end
		
		blue:begin
				s2_buf<=0;
			   s3_buf<=1;
				green_counter<=out_counter;
				state<=red;
			   end
		
		
	endcase

end

always @(posedge clk_color_50) begin

	case(led_state)
		red_led:begin
				red_buf<=1;
		    end
		
		green_led:begin
					green_buf<=2;
				end
		
		blue_led:begin
				 blue_buf<=4;
				end
		no_led:begin
				red_buf<=0;
				green_buf<=0;
				blue_buf<=0;
		 
				end
		blink_led:begin
				if(green_buf == 5 | red_buf == 5|blue_buf == 5)
							begin
								green_buf<=1;
								red_buf<=1;
								blue_buf<=1;
							end
					else if(led_counter>100000) begin
								green_buf<=green_buf+1;
								red_buf<=red_buf+1;
								blue_buf<=blue_buf+1;
								led_counter<=0;
							end
					else begin
							led_counter<=led_counter+1;
					end

				end
		
	endcase

end

always @(clk_color_50) begin
	if(clr_led) begin
		 det_buf<=0;
		 led_stb<=0;
		 led_state<=no_led;
		 
		 	
		 end
	else if(all_led) begin
				led_state<=blink_led;
					
			end
	
	else if(green_counter>107 & green_counter<115 &  red_counter>85 & red_counter<104 & blue_counter>158 & blue_counter<185 & b) begin
       //red_buf<=0;
		 //green_buf<=0;
		led_state<=blue_led;

		 
		 det_buf<=1;
		 
		 led_stb[0]<=1;
		 led_stb[1]<=0;
		 led_stb[2]<=0;
		 end
	
	
	else if(green_counter>78 & green_counter<93 &  red_counter>157 & red_counter<167 & blue_counter>97 & blue_counter<113 & r ) begin
       led_state<=red_led;

		 
		 //green_buf<=0;
		 //blue_buf<=0;
		 det_buf<=1;
		 led_stb[0]<=0;
		 led_stb[1]<=0;
		 led_stb[2]<=1;
		 
		 end
	
	else if(green_counter> 128 & green_counter<143 &  red_counter>115 & red_counter<125 & blue_counter>125 & blue_counter<133 & g) begin
       //red_buf<=0;
		 
		 led_state<=green_led;

		 //blue_buf<=0;
		 det_buf<=1;
		 led_stb[0]<=0;
		 led_stb[1]<=1;
		 led_stb[2]<=0;
		 end
	
	else begin
		 det_buf<=0;
		 
		 
		 if(led_stb[2]) begin
			r<=0;
			g<=1;
			b<=0;
			end 
		 else if(led_stb[1]) begin
			r<=0;
			g<=0;
			b<=0;
			end
		 else if(led_stb[0]) begin
			r<=0;
			g<=1;
			b<=0;
			end
		 else begin
			r<=0;
			g<=1;
			b<=0;
		 end
		 
       
		 end

end


assign s0=s0_buf;
assign s1=s1_buf;
assign s2=s2_buf;
assign s3=s3_buf;


assign redl=red_buf;
assign greenl=green_buf;
assign bluel=blue_buf;

assign det=det_buf;
assign led_st=led_stb;


endmodule
