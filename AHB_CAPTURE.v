`default_nettype none

/* The AHB Capture module is designed to capture and hold an AHB-Lite transaction 
   from the HCLK domain before sending it across the CDC boundary to the APB/PCLK domain.*/

module AHB_CAPTURE #( 
    parameter DATA_WIDTH = 32, //parameterized data width
    parameter ADDR_WIDTH = 32  // parameterized address width
)(
    input  wire                       HCLK,   // fast ahb global clock with active posedge 
    input  wire                       HRESETn, // active low global reset signal for system and bus

    input  wire                       HSEL, // means the AHB master is attempting to access your bridge/slave, when high
    input  wire [ADDR_WIDTH-1:0]      HADDR, // AHB address bus, tells the AHB slave which address the master wants to access.
    input  wire [1:0]                 HTRANS, // tells an AHB-Lite slave what type of transfer is happening on the bus.(idle, busy, nonseq, seq) 
    input  wire                       HWRITE, // write transfer when high else read transfer
    input  wire [2:0]                 HSIZE,  // indicate the size of the transfer that is typically byte, halfword or word.
    input  wire [DATA_WIDTH-1:0]      HWDATA, // it is the AHB write-data bus. It carries the data that the AHB master wants to write into the selected slave.
    input  wire                       HREADY, // inform master and all slaves that previous trans is complete, when high.
         
    //input wire [2:0]                   HBURST,   // by default using single burst .
    output wire                       HREADYOUT, //tells whether your AHB bridge is ready to finish the current transfer or needs more time, when high transfer is done.
    
    input  wire                       resp_done, //Internal indication that the APB/CDC transaction has completed
    output reg                        req_valid, //tells the next module that AHB_CAPTURE has successfully captured a valid AHB transaction to be send to cdc. 
    output reg  [ADDR_WIDTH-1:0]      req_addr, // valid address being captured
    output reg                        req_write, // valid HWRITE signal captured
    output reg  [DATA_WIDTH-1:0]      req_wdata, // valid HWDATA bus captured
    output reg  [(DATA_WIDTH/8)-1:0]  req_strb // indicates which bytes of HWDATA are valid during an AHB write
);

// transfer type
localparam [1:0] TR_IDLE   = 2'b00; // no data transfer is required , slave provides zero wait state
localparam [1:0] TR_BUSY   = 2'b01; // enable masters to insert idle cycle in the middle of a burst.
localparam [1:0] TR_NONSEQ = 2'b10; // indicate a single transfer or the first transfer of a burst.
localparam [1:0] TR_SEQ    = 2'b11; // remaining transfer in a burst are seq and the address is related to the previous transfer.

//indicates that the current AHB bus cycle contains a valid transfer,
wire ahb_valid = HSEL && HREADY && (HTRANS == TR_NONSEQ || HTRANS == TR_SEQ);

// state declaration
localparam [1:0] IDLE            = 2'b00;
localparam [1:0] WAIT_WRITE_DATA = 2'b01;
localparam [1:0] WAIT_RESPONSE   = 2'b10;

reg [1:0] state, nextstate;

// state when reset
always@(posedge HCLK or negedge HRESETn)begin
	if (!HRESETn) state <= IDLE;
	else          state <= nextstate;
end

// state transition to nextstate logic 
always@(*)begin
	case (state)
		IDLE: begin
			if (ahb_valid && HWRITE)       nextstate = WAIT_WRITE_DATA;
			else if (ahb_valid && !HWRITE) nextstate = WAIT_RESPONSE;
			else                           nextstate = IDLE;
		end
		WAIT_WRITE_DATA: nextstate = WAIT_RESPONSE;
		WAIT_RESPONSE:   nextstate = resp_done ? IDLE : WAIT_RESPONSE;
		default:         nextstate = IDLE;
	endcase
end

// when on idle no transaction is going on , and when on WAIT_RESPONSE it will look for the apb transaction is done or not.   
assign HREADYOUT = (state == IDLE)? 1'b1:((state == WAIT_RESPONSE)?resp_done :1'b0); 

// output at each state
always @(posedge HCLK or negedge HRESETn) begin
	if (!HRESETn) begin // reset all the values to 0
		req_addr  <= {ADDR_WIDTH{1'b0}};
		req_write <= 1'b0;
		req_wdata <= {DATA_WIDTH{1'b0}};
		req_strb  <= {(DATA_WIDTH/8){1'b0}};
		req_valid <= 1'b0;
	end else begin
		req_valid <= 1'b0;
		case (state)
			IDLE: begin
				if (ahb_valid) begin // capture the data when transaction is valid
					req_addr  <= HADDR;
					req_write <= HWRITE;                      
					req_strb  <= make_strb(HSIZE, HADDR[1:0]);
 					if (!HWRITE) begin // request if on read
          	req_wdata <= {DATA_WIDTH{1'b0}};
            req_valid <= 1'b1; // read request fully captured
          end
        end
      end
      WAIT_WRITE_DATA: begin // when the transaction is for write
      	req_wdata <= HWDATA; // capture the valid HWDATA bus
        req_valid <= 1'b1;   // write request fully captured
      end
      default:begin // hold the value elsewise
      	req_wdata <= req_wdata;
				req_addr <= req_addr; 
				req_valid <=1'b0; 
				req_strb <= req_strb; 
				req_write <= req_write; 
			end 
		endcase
	end
end

// this function is used to generate strb signal which will tell us valid data
function automatic [(DATA_WIDTH/8)-1:0] make_strb;
	input [2:0] hsize;
	input [1:0] addr_lsb;
	begin
		make_strb = {(DATA_WIDTH/8){1'b0}};
    case (hsize)
    	3'b000: begin // BYTE
              case (addr_lsb)
              	2'b00: make_strb = 4'b0001;
                2'b01: make_strb = 4'b0010;
                2'b10: make_strb = 4'b0100;
                2'b11: make_strb = 4'b1000;
              endcase
              end
      3'b001: begin // HALFWORD
              case (addr_lsb)
               	2'b00: make_strb = 4'b0011;
                2'b10: make_strb = 4'b1100;
                default: make_strb = 4'b0000;
              endcase
              end
      3'b010: begin // WORD
                make_strb = (addr_lsb == 2'b00) ? 4'b1111 : 4'b0000;
              end
      default: make_strb = 4'b0000; // unsupported size
    endcase
	end
endfunction
endmodule
