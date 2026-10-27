`default_nettype none

module APB_FSM #(
    parameter DATA_WIDTH = 32, // APB data width 
    parameter ADDR_WIDTH = 32, // apb address width
    parameter N_SLAVES   = 8,  // number of APB slaves
    parameter STRB_WIDTH = DATA_WIDTH/8 // PSTRB width based on data width 
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
    input  wire                     pclk_decode_err, // invalid address indication

    // to cdc_handshake
    output reg                      pclk_done,   // 1-cycle pulse indicating apb transaction completion
    output reg  [DATA_WIDTH-1:0]    pclk_rdata,  // apb read data returned to ahb side
    output reg                      pclk_err,  // apb error/ timeout indication

    // APB4 master interface
    output reg  [ADDR_WIDTH-1:0]    PADDR,
    output reg  [N_SLAVES-1:0]      PSEL,
    output reg                      PENABLE, // apb access phase enable
    output reg                      PWRITE,
    output reg  [DATA_WIDTH-1:0]    PWDATA,
    output reg  [STRB_WIDTH-1:0]    PSTRB,
    output reg  [2:0]               PPROT,  // apb protection attributes
    input  wire [DATA_WIDTH-1:0]    PRDATA, //apb read data from slave
    input  wire                     PREADY, // apb slave completion indication
    input  wire                     PSLVERR, // apb slave error indication

    // timeout monitor
    output wire                     apb_ACCESS_ST_active, // tells the timeout monitor that fsm is currently waiting fro pready
    input  wire                     timeout_hit // tells that timeout limit has been reached
);
  
    // IDLE: no transaction is active
    // setup: apb setup phase
    // access_st: apb access phase, waiting for pready
    // decerr: decode error state for invalid address
    localparam [1:0] IDLE   = 2'b00;
    localparam [1:0] SETUP  = 2'b01;
    localparam [1:0] ACCESS_ST = 2'b10;
    localparam [1:0] DECERR = 2'b11;

// state registers
reg [1:0] state, nextstate;
always @(posedge PCLK or negedge PRESETn)begin
	if (!PRESETn) 
		state <= IDLE;
	else          
		state <= nextstate;
end

// nextstate logic
always @(*) begin
	case (state)
  // waiting for a new request from cdc module, if no request is available remain in IDLE else if decode error go to DECERR otherwise normal arb transction in setup
	IDLE:       nextstate = (!pclk_start) ? IDLE : (pclk_decode_err ? DECERR : SETUP);
  SETUP:      nextstate = ACCESS_ST;
  // if apb transaction completed normally or apb slave did not respond within the allowed cycle,so transaction is terminated 
  ACCESS_ST:  nextstate = (PREADY || timeout_hit) ? IDLE : ACCESS_ST;
  // no valid slave was selected 
  DECERR:     nextstate = IDLE;
  default: nextstate = IDLE;
  endcase
end

// timeout monitor should count only when in ACCESS_ST and  PREADY is not asserted
assign apb_ACCESS_ST_active = (state == ACCESS_ST) && !PREADY;

//output logic
always @(posedge PCLK or negedge PRESETn) begin
	if (!PRESETn) begin
    PADDR      <= {ADDR_WIDTH{1'b0}};
    PSEL       <= {N_SLAVES{1'b0}};
    PENABLE    <= 1'b0;
    PWRITE     <= 1'b0;
    PWDATA     <= {DATA_WIDTH{1'b0}};
    PSTRB      <= {STRB_WIDTH{1'b0}};
    PPROT      <= 3'b000;
    pclk_done  <= 1'b0;
    pclk_rdata <= {DATA_WIDTH{1'b0}};
    pclk_err   <= 1'b0;
  end else begin
  pclk_done <= 1'b0;
  case (state)
  IDLE: begin
    PENABLE <= 1'b0; // penable must be low outside the apb access phase

    if (pclk_start && !pclk_decode_err) begin // a new valid request has arrived 
      // these signals will remain stable during SETUP and ACCESS phases
      PADDR  <= pclk_addr;
      PWRITE <= pclk_write;
      PWDATA <= pclk_wdata;
      PSTRB  <= pclk_strb;
      PPROT  <= 3'b000; // default 
      PSEL   <= pclk_slave_sel; 
      
    end else if (!pclk_start) begin // no new request is available deassert all apb slave select signals 
      PSEL <= {N_SLAVES{1'b0}};
    end
  end
  SETUP: begin
    PENABLE <= 1'b1; // ACCESS_ST begins next cycle, psel is 0
  end
  ACCESS_ST: begin
    if (PREADY || timeout_hit) begin
      pclk_rdata <= PRDATA;
      pclk_err   <= PSLVERR | timeout_hit;
      pclk_done  <= 1'b1; // inform REQUEST_CDC that the APB transaction has completed.
      PSEL       <= {N_SLAVES{1'b0}};
      PENABLE    <= 1'b0;
    end
  end
  DECERR: begin
    pclk_rdata <= {DATA_WIDTH{1'b0}}; // no valid APB read data is available 
    pclk_err   <= 1'b1; // report a decode error to HCLK domain
    pclk_done  <= 1'b1; // report that the transaction has completed
  end
  default:begin pclk_done <=1'b0; end
  endcase
  end
end
endmodule 