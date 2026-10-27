`default_nettype none
//This module checks HADDR and determines which APB slave should receive the transaction.

module ADDRESS_DECODER 
#(
  parameter ADDR_WIDTH = 32, //address bit width 
  parameter N_SLAVES   = 8,  // number of acceptable slaves 
  // Base address of each slave peripheral (4KB aligned )
  parameter [ADDR_WIDTH-1:0] SLAVE0_BASE = 32'h0000_0000,
  parameter [ADDR_WIDTH-1:0] SLAVE1_BASE = 32'h0000_1000,
  parameter [ADDR_WIDTH-1:0] SLAVE2_BASE = 32'h0000_2000,
  parameter [ADDR_WIDTH-1:0] SLAVE3_BASE = 32'h0000_3000,
  parameter [ADDR_WIDTH-1:0] SLAVE4_BASE = 32'h0000_4000,
  parameter [ADDR_WIDTH-1:0] SLAVE5_BASE = 32'h0000_5000,
  parameter [ADDR_WIDTH-1:0] SLAVE6_BASE = 32'h0000_6000,
  parameter [ADDR_WIDTH-1:0] SLAVE7_BASE = 32'h0000_7000,

  // Uniform size (address space range) allocated per slave's address window
  parameter [ADDR_WIDTH-1:0] SLAVE_SIZE  = 32'h0000_1000   // 4KB each

)(
  input  wire [ADDR_WIDTH-1:0] addr, // captured address from AHB capture
  output reg  [N_SLAVES-1:0]   slave_sel, // tells us which slave is selcted
  output reg                   decode_err // if no valid address is given which does not map to any slave we will get an error
);

// decoder logic 
always @(*) begin
	slave_sel  = {N_SLAVES{1'b0}}; // default none of the slaves are selected
	decode_err = 1'b0;             // no error

	case (1'b1)
    // validating the address by checking in which window it falls and then we are enabling that slave associated to that address.
		(addr >= SLAVE0_BASE) && (addr < SLAVE0_BASE + SLAVE_SIZE): slave_sel[0] = 1'b1; 
		(addr >= SLAVE1_BASE) && (addr < SLAVE1_BASE + SLAVE_SIZE): slave_sel[1] = 1'b1;
		(addr >= SLAVE2_BASE) && (addr < SLAVE2_BASE + SLAVE_SIZE): slave_sel[2] = 1'b1;
		(addr >= SLAVE3_BASE) && (addr < SLAVE3_BASE + SLAVE_SIZE): slave_sel[3] = 1'b1;
		(addr >= SLAVE4_BASE) && (addr < SLAVE4_BASE + SLAVE_SIZE): slave_sel[4] = 1'b1;
		(addr >= SLAVE5_BASE) && (addr < SLAVE5_BASE + SLAVE_SIZE): slave_sel[5] = 1'b1;
		(addr >= SLAVE6_BASE) && (addr < SLAVE6_BASE + SLAVE_SIZE): slave_sel[6] = 1'b1;
		(addr >= SLAVE7_BASE) && (addr < SLAVE7_BASE + SLAVE_SIZE): slave_sel[7] = 1'b1;
		default: decode_err = 1'b1; // if address does not match to any slave's range we will output it as an error.
	endcase
end
endmodule