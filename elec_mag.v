module elec_mag(
input state,
output [1:0] emo

);
 
parameter PICK = 0, DROP = 1;
reg [1:0]em ;
//reg stat=PICK;


	always @(*) begin
		case(state)
			PICK : begin
					 em[0] <= 1'b1 ;
					 em[1] <= 1'b0 ;
			       end
			
			DROP : begin
				    em[0] <= 1'b0 ;
					 em[1] <= 1'b0 ;
				    end
	   endcase
	end
	assign emo = em;
	
endmodule

