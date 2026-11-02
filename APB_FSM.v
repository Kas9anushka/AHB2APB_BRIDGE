`default_nettype none

module APB_FSM #(
    parameter DATA_WIDTH = 32, // APB data width 
    parameter ADDR_WIDTH = 32, // apb address width
    parameter N_SLAVES   = 8,  // number of APB slaves
    parameter STRB_WIDTH = DATA_WIDTH/8, // PSTRB width based on data width 
    parameter TIMEOUT_CYCLES = 16 // max wait cycles in access phase, 0 disables timeout
)(
    input  wire                     PCLK, // apb clock
    input  wire                     PRESETn, // active low apb reset

    // from cdc_handshake
    input  wire                     pclk_start, // one cycle pulse indicating a new request
    input  wire [ADDR_WIDTH-1:0]    pclk_addr,
    input  wire                     pclk_write,
    input  wire [DATA_WIDTH-1:0]    pclk_wdata,
    input  wire [STRB_WIDTH-1:0]    pclk_strb, // byte lane strobes
    input  wire [N_SLAVES-1:0]      pclk_slave_sel, // selected apb slave

    // to cdc_handshake
    output reg                      pclk_done,   // 1-cycle pulse indicating apb transaction completion
    output reg  [DATA_WIDTH-1:0]    pclk_rdata,  // apb read data returned to ahb side
    output reg                      pclk_err,  // apb error indicationa

    // APB4 master interface
    output reg  [ADDR_WIDTH-1:0]    PADDR,
    output reg  [N_SLAVES-1:0]      PSEL,
    output reg                      PENABLE, // apb access phase enable
    output reg                      PWRITE,
    output reg  [DATA_WIDTH-1:0]    PWDATA,
    output reg  [STRB_WIDTH-1:0]    PSTRB,
    input  wire [DATA_WIDTH-1:0]    PRDATA, //apb read data from slave
    input  wire                     PREADY, // apb slave completion indication
    input  wire                     PSLVERR // apb slave error indication
);
  
    // IDLE: no transaction is active
    // setup: apb setup phase
    // access_st: apb access phase, waiting for pready
    localparam [1:0] IDLE   = 2'b00;
    localparam [1:0] SETUP  = 2'b01;
    localparam [1:0] ACCESS_ST = 2'b10;

// state registers
reg [1:0] state, nextstate;
always @(posedge PCLK or negedge PRESETn)begin
	if (!PRESETn) 
		state <= IDLE;
	else          
		state <= nextstate;
end

// timeout counter, counts wait cycles in the access phase
localparam CNT_W = $clog2(TIMEOUT_CYCLES + 2);
reg  [CNT_W-1:0] to_cnt;
wire timeout_hit = (TIMEOUT_CYCLES != 0) && (state == ACCESS_ST) && !PREADY && (to_cnt == TIMEOUT_CYCLES-1);

always @(posedge PCLK or negedge PRESETn) begin
	if (!PRESETn)
		to_cnt <= {CNT_W{1'b0}};
	else if (state == ACCESS_ST && !PREADY)
		to_cnt <= to_cnt + 1'b1;
	else
		to_cnt <= {CNT_W{1'b0}};
end

// nextstate logic
always @(*) begin
	case (state)
   IDLE: nextstate = (pclk_start)? SETUP: IDLE;
   SETUP: nextstate = ACCESS_ST;
   ACCESS_ST: nextstate = (PREADY || timeout_hit)? ((pclk_start)? SETUP: IDLE): ACCESS_ST;
  default: nextstate = IDLE;
  endcase
end

// request registers, these signals will remain stable during SETUP and ACCESS phases
reg [ADDR_WIDTH-1:0] addr_q;
reg                  write_q;
reg [DATA_WIDTH-1:0] wdata_q;
reg [STRB_WIDTH-1:0] strb_q;
reg [N_SLAVES-1:0]   sel_q;

always @(posedge PCLK or negedge PRESETn) begin
	if (!PRESETn) begin
    addr_q  <= {ADDR_WIDTH{1'b0}};
    write_q <= 1'b0;
    wdata_q <= {DATA_WIDTH{1'b0}};
    strb_q  <= {STRB_WIDTH{1'b0}};
    sel_q   <= {N_SLAVES{1'b0}};
  end else if (pclk_start && (state == IDLE || (state == ACCESS_ST && (PREADY || timeout_hit)))) begin // a new request has arrived
    addr_q  <= pclk_addr;
    write_q <= pclk_write;
    wdata_q <= pclk_wdata;
    strb_q  <= pclk_strb;
    sel_q   <= pclk_slave_sel;
  end
end

//output logic
always @(*) begin
  PADDR      = addr_q;
  PWRITE     = write_q;
  PWDATA     = wdata_q;
  PSTRB      = strb_q;
  PSEL       = {N_SLAVES{1'b0}};
  PENABLE    = 1'b0; // penable must be low outside the apb access phase
  pclk_done  = 1'b0;
  pclk_rdata = {DATA_WIDTH{1'b0}};
  pclk_err   = 1'b0;

  case (state)
  IDLE: begin
    PSEL    = {N_SLAVES{1'b0}};
    PENABLE = 1'b0;
  end
  SETUP: begin
    PSEL    = sel_q;
    PENABLE = 1'b0;
  end
  ACCESS_ST: begin
    PSEL    = sel_q;
    PENABLE = 1'b1;
    if (PREADY) begin
      pclk_rdata = PRDATA;
      pclk_err   = PSLVERR;
      pclk_done  = 1'b1; // inform REQUEST_CDC that the APB transaction has completed.
    end else if (timeout_hit) begin
      pclk_err   = 1'b1;
      pclk_done  = 1'b1;
    end
  end
  default: begin
    PSEL    = {N_SLAVES{1'b0}};
    PENABLE = 1'b0;
  end
  endcase
end
endmodule