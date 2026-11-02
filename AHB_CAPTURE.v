`default_nettype none

/* The AHB Capture module is designed to capture and hold an AHB-Lite transaction 
   from the HCLK domain before sending it across the CDC boundary to the APB/PCLK domain.*/

module AHB_CAPTURE #( 
    parameter DATA_WIDTH = 32, //parameterized data width
    parameter ADDR_WIDTH = 32,  // parameterized address width
    parameter N_SLAVES = 8
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
         
    input wire [2:0]                  HBURST,   
    input wire                        dec_err, // when unmapped address it reject the transaction in ahb side itself.
    input wire [N_SLAVES-1:0]         slave_sel, 
    output wire                       HREADYOUT, //tells whether your AHB bridge is ready to finish the current transfer or needs more time, when high transfer is done.
    output wire [DATA_WIDTH-1:0]      HRDATA,
    output wire                       HRESP,
    input  wire                       resp_done, //Internal indication that the APB/CDC transaction has completed
    input wire                        resp_err,
    input wire [DATA_WIDTH-1:0]       resp_rdata,
    output wire                        apb_start, //tells the next module that AHB_CAPTURE has successfully captured a valid AHB transaction to be send to cdc. 
    output wire  [ADDR_WIDTH-1:0]      req_addr, // valid address being captured
    output wire                        req_write, // valid HWRITE signal captured
    output wire  [DATA_WIDTH-1:0]      req_wdata, // valid HWDATA bus captured
    output wire [N_SLAVES-1:0]        req_slave_sel,
    output wire  [(DATA_WIDTH/8)-1:0]  req_strb // indicates which bytes of HWDATA are valid during an AHB write
);
localparam BYTES = DATA_WIDTH/8;
// transfer type
localparam [1:0] TR_IDLE   = 2'b00; // no data transfer is required , slave provides zero wait state
localparam [1:0] TR_BUSY   = 2'b01; // enable masters to insert idle cycle in the middle of a burst.
localparam [1:0] TR_NONSEQ = 2'b10; // indicate a single transfer or the first transfer of a burst.
localparam [1:0] TR_SEQ    = 2'b11; // remaining transfer in a burst are seq and the address is related to the previous transfer.

// state declaration
localparam [2:0] S_IDLE  = 3'd0,
                 S_WDATA = 3'd1,
                 S_WAIT  = 3'd2,
                 S_ERR1  = 3'd3,
                 S_ERR2  = 3'd4;

reg [2:0]            state;
reg [ADDR_WIDTH-1:0] addr_q;
reg                  write_q;
reg [2:0]            size_q;
reg [N_SLAVES-1:0]   sel_q;
reg                  bad_q;
reg                  start_q;
reg [DATA_WIDTH-1:0] hwdata_reg;


wire [31:0] nbytes_now     = 32'd1 << HSIZE;
wire        size_bad_now   = (nbytes_now > BYTES);
wire        misaligned_now = ((HADDR & (nbytes_now - 1)) != 0);
wire        bad_now        = size_bad_now || misaligned_now || dec_err;

wire resp_ok  = resp_done && !resp_err;
wire resp_bad = resp_done &&  resp_err;

wire can_take = (state == S_IDLE) ||(state == S_ERR2) ||(state == S_WAIT && resp_ok);

wire take = HSEL && HREADY && can_take && (HTRANS == TR_NONSEQ || HTRANS == TR_SEQ);

always @(posedge HCLK or negedge HRESETn) begin
    if (!HRESETn) begin
        state      <= S_IDLE;
        addr_q     <= {ADDR_WIDTH{1'b0}};
        write_q    <= 1'b0;
        size_q     <= 3'b0;
        sel_q      <= {N_SLAVES{1'b0}};
        bad_q      <= 1'b0;
        start_q    <= 1'b0;
        hwdata_reg <= {DATA_WIDTH{1'b0}};
    end else begin
        start_q <= 1'b0;                     

        case (state)
            S_WDATA: begin
                hwdata_reg <= HWDATA;         
                start_q    <= 1'b1;           
                state      <= S_WAIT;
            end
            S_WAIT: begin
                if (resp_bad)      state <= S_ERR2;   
                else if (resp_ok)  state <= S_IDLE;   
            end
            S_ERR1: state <= S_ERR2;
            S_ERR2: state <= S_IDLE;           
            default: ;
        endcase

        if (take) begin
            addr_q  <= HADDR;
            write_q <= HWRITE;
            size_q  <= HSIZE;
            sel_q   <= slave_sel;
            bad_q   <= bad_now;

            if (bad_now)
                state <= S_ERR1;
            else if (HWRITE)
                state <= S_WDATA;
            else begin
                state   <= S_WAIT;
                start_q <= 1'b1;             
            end
        end
    end
end

assign HREADYOUT = (state == S_IDLE) ? 1'b1 :(state == S_ERR2) ? 1'b1 :(state == S_WAIT) ? resp_ok : 1'b0;   

assign HRESP     = (state == S_ERR1) || (state == S_ERR2) || (state == S_WAIT && resp_bad);

assign HRDATA    = resp_rdata;

assign apb_start     = start_q;
assign req_addr      = addr_q;
assign req_write     = write_q;
assign req_wdata     = hwdata_reg;
assign req_slave_sel = sel_q;
assign req_strb      = (write_q && !bad_q) ? make_strb(size_q, addr_q) : {BYTES{1'b0}};

function automatic [(DATA_WIDTH/8)-1:0] make_strb;
    input [2:0] hsize;
    input [ADDR_WIDTH-1:0] addr;
    integer nbytes, offs;
    reg [(DATA_WIDTH/8)-1:0] mask;
    begin
        nbytes = 1 << hsize;
        if (nbytes > BYTES) nbytes = BYTES;
        mask = {BYTES{1'b1}} >> (BYTES - nbytes);
        offs = addr & (BYTES-1);
        make_strb = mask << offs;
    end
endfunction

endmodule