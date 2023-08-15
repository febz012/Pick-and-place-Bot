module SM1068_bot_1068(
	
	input clk_50_master,
   input color_out,
   input  dout,
	output adc_cs_n,
	output din,
	output adc_sck,
	output s0,
	output s1,
	output s2,
	output s3,
	output pnum,
	output [2:0] redl,
	output [2:0]greenl,
	output [2:0]bluel,
	output [2:0] direction,
	output [4:0] ledn,
	output tx,
	output [2:0]state,
	output [3:0] hv,
	output clr_led,
	output [1:0] buf_sel,
	output [2:0] led_st,
	output [1:0] emo,
	output [7:0] tx_buf,
	output all_led,
	output [1:0] sl0,output [1:0] sl1,output [1:0] sl2,output [1:0] sl3,output [1:0] sl4,output [1:0] sl5,output [1:0] sl6,output [1:0] sl7,output [1:0] sl8,output [1:0] sl9,
output [1:0] sl10,output [1:0] sl11,output [1:0] sl12,output [1:0] sl13,output [1:0] sl14,output [1:0] sl15,output [1:0] sl16,output [1:0] sl17,output [1:0] sl18,output [1:0] sl19,
output [1:0] sl20,output [1:0] sl21,output [1:0] sl22,output [1:0] sl23,output [1:0] sl24,output [1:0] sl25,output [1:0] sl26,output [1:0] sl27,output [1:0] sl28,output [1:0] sl29,
output [3:0]len

,output [5:0] iend,output [5:0] istr,output enpl,output signal, output [1:0] empi,output[1:0] orientation

   );
	
parameter front=0,right=2,left=3,stop=1,sleft=4,sright=5,back=6,speed=7;

parameter PATH=0,TRAVERSE =1,END=2,WARE=3,PLACE=4,DELAY=5;

parameter PICK = 0, DROP = 1;


reg dep_comp=0;
reg all_ledb=0;
reg nut_count=0;
reg wat_count=0;
reg pest_count=0;
assign all_led=all_ledb;

reg [6:0]pest_ware[1:0];
reg [6:0]nut_ware[1:0];
reg [6:0]wat_ware[1:0];

initial begin
	nut_ware[0]=18;
	nut_ware[1]=16;
	
	pest_ware[0]=4;
	pest_ware[1]=2;
	
	wat_ware[0]=3;
	wat_ware[1]=19;
	end

reg clr_ledb;
reg[2:0] bot_state=PATH;
reg en_turn=1;
reg nod_turn=0;
reg [1:0] sim_b=0;
reg end_run=0;

reg[5:0] iendn=28;
reg[5:0] istrt=15;
reg enplan=0;

reg [1:0] orien=2;
assign orientation=orien;

assign enpl=enplan;
assign iend=iendn;
assign istr=istrt;

reg [30:0] del_counter=0;
wire det;
reg[3:0] i=0;
reg[3:0] j=0;
reg init_turn=0;
reg[3:0] mdir=stop;
reg[3:0] mot_dir=stop;
reg [1:0] map[36:0]; 
 
reg [4:0] ledb=0;
reg [7:0] tx_bu=0;

reg [1:0] dir;
reg u_en=0;

reg n_f=0;

reg trig=0;

reg [4:0] ng = 5'b00110;
assign pnum=sim_b;


SM1068_color_sensor col(.out(color_out),.s0(s0),.s1(s1),.s2(s2),.s3(s3),.redl(redl),.greenl(greenl),.bluel(bluel),.clk_color_50(clk_50_master),.det(det),.clr_led(clr_led),.led_st(led_st),.all_led(all_led));

SM1068_adc_control adc(.clk_50(clk_50_master),.dout(dout),.adc_cs_n(adc_cs_n),.din(din),.adc_sck(adc_sck),.direction(direction));

SM1068_uart urt(.clk_50M(clk_50_master),.data_in(tx_buf),.tx(tx),.en_tri(u_en),.buf_sel(buf_sel),.p_num(pnum));

SM1068_motor mot(.hv(hv),.state(state),.clk_50(clk_50_master));

SM1068_elec_mag ele(.state(signal),.clk(clk_50_master),.emo(empi ));

SM1068_path_planner pat(.sl0(sl0),.sl1(sl1),.sl2(sl2),.sl3(sl3),.sl4(sl4),.sl5(sl5),.sl6(sl6),.sl7(sl7),.sl8(sl8),.sl9(sl9),.sl10(sl10),.sl11(sl11),.sl12(sl12),.sl13(sl13),.sl14(sl14),.sl15(sl15),.sl16(sl16),.sl17(sl17),.sl18(sl18),.sl19(sl19),
                        .sl20(sl20),.sl21(sl21),.sl22(sl22),.sl23(sl23),.sl24(sl24),.sl25(sl25),.sl26(sl26),.sl27(sl27),.sl28(sl28),.sl29(sl29),.clk(clk_50_master),.iend(iend),.istr(istr),.enpl(enpl),.orientation(orientation),.len(len));

always @(posedge clk_50_master) begin
 map[0]<=sl0; map[1]<=sl1; map[2]<=sl2; map[3]<=sl3; map[4]<=sl4; map[5]<=sl5; map[6]<=sl6; map[7]<=sl7; map[8]<=sl8; map[9]<=sl9;
 map[10]<=sl10; map[11]<=sl11; map[12]<=sl12; map[13]<=sl13; map[14]<=sl14; map[15]<=sl15; map[16]<=sl16; map[17]<=sl17; map[18]<=sl18; map[19]<=sl19;
 map[20]<=sl20; map[21]<=sl21; map[22]<=sl22; map[23]<=sl23; map[24]<=sl24; map[25]<=sl25; map[26]<=sl26; map[27]<=sl27; map[28]<=sl28; map[29]<=sl29;
 

end

reg sigl=0;
assign signal=sigl;
reg [20:0] en_counter=0;
reg en_rst=0;
reg [20:0] b_counter=0;
reg b_rst=0;

reg[1:0] current_dir=0;

reg mot_en;


reg[1:0] buf_selb;

reg init_f=1;

reg [29:0] init_counter=0;
reg [29:0] end_counter=0;
reg b_trig=0;

reg [20:0] td_counter=0; //turn delay counter
reg td_rst=0;
reg en_td = 0;
reg pick_trig=0;
reg dep_trig=0;

always @(posedge clk_50_master) begin
		init_counter<=(init_counter<250000000)?init_counter+1:250000100;
		init_f<=(init_counter<230000000)?0:1;
		
		
		if(init_counter>250000000) begin
				mot_en<=1;
				init_f<=0;
				end
		else begin
				mot_en<=0;
				end
		
		end

always @(clk_50_master) begin
	trig<=det|dep_trig|pick_trig;
	end 

always@(*) begin 
		b_rst<=(b_counter<120)?0:1;
		end

always@(posedge trig or  posedge b_rst) begin
		if(b_rst) begin
				b_trig<=0;
				end
		else begin
				b_trig<=1;
				end
		end

always@(posedge clk_50_master or posedge b_rst) begin
		if(b_rst) begin
			b_counter<=0;
			end
		else begin
			  if(b_trig) begin
					b_counter<=b_counter+1;
					end
				end
		end


always@(posedge trig) begin
	if(det==1)begin
		buf_selb<=2;
		end
	else if(pick_trig==1) begin
		buf_selb<=0;
		end
	else if(dep_trig==1) begin
		buf_selb<=1;
		end
	else begin
		buf_selb<=3;
		end
	end

always@(*) begin
		en_rst<=(en_counter<120)?0:1;
		end

always@(posedge b_trig or  posedge en_rst) begin
		if(en_rst) begin
				u_en<=0;
				end
		else begin
				u_en<=1;
				end
      end

always@(posedge clk_50_master or posedge en_rst) begin
		if(en_rst) begin
			en_counter<=0;
			end
		else begin
			  if(u_en) begin
					en_counter<=en_counter+1;
					end
				end
		end

//counter for delay durig turn
always@(*) begin 
		td_rst<=(td_counter<10000000)?0:1;
		end

always@(posedge en_td or  posedge td_rst) begin
		if(td_rst) begin
				en_turn<=1;
				end
		else begin
				en_turn<=0;
				end
		end

always@(posedge clk_50_master or posedge td_rst) begin
		if(td_rst) begin
			td_counter<=0;
			end
		else begin
			  if(en_turn==0) begin
					td_counter<=td_counter+1;
					end
			  end
		end

//////////////////////////////////

always @(posedge n_f) begin
	   
   case (bot_state) 
	
			PATH: begin
					sigl<=DROP;
					//dep_comp<=0;
					pick_trig<=0;
					dep_trig<=0;
					enplan<=0;
					if(i==len) begin
						i<=0;
						 bot_state<=TRAVERSE;
						end
					else begin
							if(init_turn==1) begin
									i<=i+1;end
							else begin
								i<=i;
								end
						  j<=0;
						  end
					end
			TRAVERSE:begin
					//dep_comp<=0;
					
					pick_trig<=0;
					dep_trig<=0;
					sigl<=DROP;
					if(i==3) begin
						i<=0;
						bot_state<=WARE;
						orien<=0;
						enplan<=1;
						istrt<=28;
						if(led_st[1]==1) begin
								iendn<=nut_ware[nut_count];
								nut_count<=nut_count+1;
								end
						else if(led_st[0]==1) begin
								iendn<=wat_ware[wat_count];
								wat_count<=wat_count+1;
								end
						else if(led_st[2]==1) begin
								iendn<=pest_ware[pest_count];
								pest_count<=pest_count+1;
								end
						
						end
					else begin
					     i<=i+1;
						  j<=0;
						  end
					end
			WARE: begin
					
					//dep_comp<=0;
					if(i==len-1) begin
						i<=0;
						sigl<=PICK;
						pick_trig<=1;
						dep_trig<=0;
						bot_state<=PLACE;
						orien<=1;
						istrt<=18;
						iendn<=31;
						enplan<=1;
						end
					else begin
							enplan<=0;
							if(init_turn==1) begin
									i<=i+1;end
							else begin
								i<=i;
								end
						  j<=0;
						  end
					end
			PLACE: begin
					
					enplan<=0;
					pick_trig<=0;
					if(i==len-1) begin
						i<=0;
						sigl<=DROP;
						dep_trig<=1;
						bot_state<=END;
						//dep_comp<=1;
						end
					else begin
							if(init_turn==1) begin
									i<=i+1;end
							else begin
								i<=i;
								end
						  j<=0;
						  end
					end
			END: begin
						pick_trig<=0;
						dep_trig<=0;
						
						bot_state<=END;
						//dep_comp<=0;
						sigl<=DROP;
						
					end
			 
			endcase
	end

always@(posedge clk_50_master) begin

	if(mot_en==1) begin
	
		if(dep_comp==1) begin 
				clr_ledb<=1;
				all_ledb<=0;
		end
		else if(end_run == 1) begin 
				clr_ledb<=0;
				all_ledb<=1;
		end
		else begin
			clr_ledb<=0;
			all_ledb<=0;
		end
	end
	
	else begin
	clr_ledb<=1;
	all_ledb<=0;
	end

end

always @(posedge clk_50_master) begin

		case(bot_state)
			
				PATH: begin
						current_dir<=map[i];
						if(n_f==1 & map[i]==1) begin
						        mot_dir<=right;
							     end
							 else if(n_f==1 & map[i]==2'b11) begin
						        mot_dir <=left;
							     end
							 else if(n_f==1 & map[i]==2'b00) begin
						        mot_dir<=front;
							     end
						    else begin 
						       mot_dir<=stop;
						       end
							 end
			   
			   TRAVERSE: begin
								
								current_dir<=ng[i];
								if(n_f==1 & ng[i]==1) begin
								   
					
									mot_dir<=right;
							      end
						     else begin 
						         mot_dir<=front;
									dep_comp<=0;
								end_run<=0;
					
						       end
							 end
					  WARE: begin
								
					
								del_counter<=0;
								current_dir<=map[i];
						if(n_f==1 & map[i]==1) begin
						        mot_dir<=right;
							     end
							 else if(n_f==1 & map[i]==2'b11) begin
						        mot_dir <=left;
							     end
							 else if(n_f==1 & map[i]==2'b00) begin
						        mot_dir<=front;
							     end
						    else begin 
						       mot_dir<=stop;
						       end
							 end
					PLACE: begin
						
					
						if(i==len) begin
							mot_dir<=stop;
							dep_comp<=0;
							end_run<=0;
							end
					   else begin
							end_run<=0;
							dep_comp<=0;
							if(del_counter>25000000) begin
									current_dir<=map[i];
									if(n_f==1 & map[i]==1) begin
											mot_dir<=right;
											end
							      else if(n_f==1 & map[i]==2'b11) begin
						               mot_dir <=left;
							            end
									else if(n_f==1 & map[i]==2'b00) begin
											mot_dir<=speed;
											end
									else if(n_f==1 & map[i]==2'b10) begin
											mot_dir<=back;
											end
									
									else begin 
											mot_dir<=stop;
											end
										end
									else begin
											del_counter<=del_counter+1;
											mot_dir<=stop;
											end
							     end
							end
				  END:begin
							mot_dir<=stop;
							if(end_counter>50000000) begin
										dep_comp<=0;
										end_run<=1;
										end
							else if(end_counter>25000000) begin
										dep_comp<=1;
										end_run<=0;
										end_counter<=end_counter+1;
										end
							
							else begin
									end_counter<=end_counter+1;
									dep_comp<=0;
									end_run<=0;
									end
					
						end
					endcase
				end


always@(posedge clk_50_master) begin
				
		
		case(direction)
				3'b000:begin
				       n_f<=1;
						 
						 mdir<=mot_dir;
						 if(current_dir==2'b00) begin
								en_td<=0;
								nod_turn<=1;
								
								end
						 else if(current_dir==2'b01)begin
								en_td<=1;
								nod_turn<=0;
								end
						 else if(current_dir==2'b11)begin
								en_td<=1;
								nod_turn<=0;
								end
						  else if(current_dir==2'b10)begin
								en_td<=0;
								nod_turn<=0;
								end
						 else begin
								nod_turn<=1;
								en_td<=0;
								end
						 end
				
				3'b110:begin
							if(nod_turn) begin
				      	mdir<=right;
							end
							
											
						 end
				3'b011:begin
								if(nod_turn) begin
				      		mdir<=left;
								end
											
						 end
				3'b101:begin
				            if(mot_en==1) begin
									
									if(init_turn==0 & map[i]==1) begin
										mdir<=sright;
										n_f<=1;
										en_td<=0;
										nod_turn<=0;
										end
									else if(init_turn==0 & map[i]==2'b11) begin
										mdir<=sleft;
										n_f<=1;
										en_td<=0;
										nod_turn<=0;
										end
									
									else begin
										en_td<=0;
										n_f<=0;
										if(current_dir==2'b10) begin
												mdir<=back;
												nod_turn<=0;
												end
										else if(mot_dir==speed) begin
												mdir<=speed;
												nod_turn<=1;
												end
											else begin
													nod_turn<=1;
													mdir<=front;
													end
										init_turn<=1;
										
										end
									end
								else begin
									  mdir<=stop;
									  n_f<=1;
									  nod_turn<=1;
								 	  end
						 end
			   3'b001:begin
								if(nod_turn) begin
						 
								mdir<=left;
								end
						  end
						 
			   3'b100:begin
						     if(nod_turn) begin
						
								mdir<=right;
								end
						
						  end
			
				3'b111: begin 
							if(mot_en==0) begin
							init_turn<=0;
							end
							else begin
							 init_turn<=1;
								
									if(current_dir==1) begin
										mdir<=right;
										end
									else if(current_dir==2'b11) begin
										mdir <=left;
										end
									else if(current_dir==2'b00) begin
										mdir<=front;
										end
									else if(current_dir==2'b10) begin
										mdir<=back;
										end
									
									else begin
										mdir<=mdir;
								   end
								  end
							  
							end
				default:begin
							en_td<=0;
							end
		endcase
end



always @(posedge b_trig) begin
  
   if(led_st[1]) begin
		tx_bu<=78;
		sim_b<=1;		
		end
	else if(led_st[0]) begin
		tx_bu<=87;
		sim_b<=3;
		end
	else if(led_st[2]) begin
		tx_bu<=80;
		sim_b<=1;
		end
	else begin
		tx_bu<=70;
		sim_b<=0;
		end
end

assign buf_sel=buf_selb;
assign ledn=ledb;
assign state=mdir;
assign clr_led=clr_ledb;
assign tx_buf=tx_bu;



endmodule 