`default_nettype none
module address_decoder(
    input wire [31:0]address,
    output reg [7:0]psel_selected,
    output reg decode_err
    );

    always @(*) begin
	psel_selected = 8'b0;
	decode_err = 1'b0;     
	case (1'b1)
		(address >= 32'h0000_0000) && (address < 32'h0000_1000): psel_selected[0] = 1'b1; 
		(address >= 32'h0000_1000) && (address < 32'h0000_2000): psel_selected[1] = 1'b1;
		(address >= 32'h0000_2000) && (address < 32'h0000_3000): psel_selected[2] = 1'b1;
		(address >= 32'h0000_3000) && (address < 32'h0000_4000): psel_selected[3] = 1'b1;
		(address >= 32'h0000_4000) && (address < 32'h0000_5000): psel_selected[4] = 1'b1;
		(address >= 32'h0000_5000) && (address < 32'h0000_6000): psel_selected[5] = 1'b1;
		(address >= 32'h0000_6000) && (address < 32'h0000_7000): psel_selected[6] = 1'b1;
		(address >= 32'h0000_7000) && (address < 32'h0000_8000): psel_selected[7] = 1'b1;
		default: decode_err = 1'b1;
	endcase
end
endmodule    