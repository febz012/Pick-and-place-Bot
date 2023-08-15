module SM1068_motor(
output [3:0] hv,
input[2:0] state,
input clk_50
);
parameter front=0,stop=1,right=2,left=3,sleft=4,sright=5,back=6,speed=7;

reg [3:0]lv=0;

reg [16:0]clk_cnt=0;
reg div_out;
reg [16:0]on_time=66326;

reg [16:0]clk_cnt1=0;
reg div_out1;
reg [16:0]on_time1=89326;

reg [16:0]clk_cnt2=0;
reg div_out2;
reg [16:0]on_time2=79326;

always @(posedge clk_50) begin
on_time<=66326;
end

always @(posedge clk_50) begin
if(state==stop) begin
	clk_cnt<=0;
	end
else begin							
	clk_cnt<=(clk_cnt<102040)?clk_cnt+1:0;
	div_out<=(clk_cnt<on_time)?0:1;
	end
end

always @(posedge clk_50) begin
on_time1<=89326;
end

always @(posedge clk_50) begin
if(state==stop) begin
	clk_cnt1<=0;
	end
else begin							
	clk_cnt1<=(clk_cnt1<102040)?clk_cnt1+1:0;
	div_out1<=(clk_cnt1<on_time1)?0:1;
	end
end
always @(posedge clk_50) begin
on_time2<=79326;
end

always @(posedge clk_50) begin
if(state==stop) begin
	clk_cnt2<=0;
	end
else begin							
	clk_cnt2<=(clk_cnt2<102040)?clk_cnt2+1:0;
	div_out2<=(clk_cnt2<on_time2)?0:1;
	end
end


always@(*)begin
	
	case(state)
			front:begin
						
							lv[0]<=div_out;
							lv[1]<=1'b1;
							lv[2]<=1'b1;
							lv[3]<=div_out;
					 end
			
			right:begin
						
							lv[0]<=1'b0;
							lv[1]<=1'b1;
							lv[2]<=1'b0;
							lv[3]<=1'b0;
					 end
			 sright:begin
						
							lv[0]<=1'b0;
							lv[1]<=1'b1;
							lv[2]<=1'b0;
							lv[3]<=1'b0;
					 end
			left:begin
						
							lv[0]<=1'b0;
							lv[1]<=1'b0;
							lv[2]<=1'b1;
							lv[3]<=div_out1;
					 end
			 sleft:begin
						
							lv[0]<=1'b0;
							lv[1]<=1'b0;
							lv[2]<=1'b1;
							lv[3]<=1'b0;
					 end
			 back:begin
						
							lv[0]<=1'b1;
							lv[1]<=div_out1;
							lv[2]<=div_out1;
							lv[3]<=1'b1;
					 end
			 speed:begin
						
							lv[0]<=div_out2;
							lv[1]<=1'b1;
							lv[2]<=1'b1;
							lv[3]<=div_out2;
					 end
			stop:    begin
							lv[0]<=1'b0;
							lv[1]<=1'b0;
							lv[2]<=1'b0;
							lv[3]<=1'b0;
							
							end
			
	endcase				
end


assign hv[0] = lv[0];
assign hv[1] = lv[1];
assign hv[2] = lv[2];
assign hv[3] = lv[3];

endmodule
