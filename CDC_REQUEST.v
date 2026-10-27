`default_nettype none

module CDC_REQUEST #(
    parameter DATA_WIDTH = 32, // parameterized data width 
    parameter ADDR_WIDTH = 32, // parameterized address width 
    parameter N_SLAVES   = 8, // number of slaves in apb side
    parameter STRB_WIDTH = DATA_WIDTH/8 // PSTRB WIDTH with respect to data width 
)(
  // AHB SIDE signals 
    input  wire                      HCLK, // global ahb side active posedge clk
    input  wire                      HRESETn, // global active low reset in ahb side

    input  wire                      req_valid, // valid transaction signal from ahb capture
    input  wire [ADDR_WIDTH-1:0]     req_addr,  // valid captured address
    input  wire                      req_write, // valid captured write signal
    input  wire [DATA_WIDTH-1:0]     req_wdata, // valid captured HWDATA 
    input  wire [STRB_WIDTH-1:0]     req_strb, // pstrb signal generated in ahb captured
    input  wire [N_SLAVES-1:0]       slave_sel, // which slave is active 
    input  wire                      decode_err, // invalid address for slave mapping 

    // signals coming back from pclk side to hclk side 
    output reg                       rsp_valid, // indicates that the APB transaction has completed and a valid response is available
    output reg [DATA_WIDTH-1:0]      rsp_rdata, //contains the read data returned by the APB slave (PRDATA), which eventually becomes HRDATA.
    output reg                       rsp_err,  // indicates that the APB transaction ended with an error (PSLVERR), which eventually becomes HRESP = ERROR.


 // APB SIDE signals 
    input  wire                      PCLK, // global APB side active posedge clk
    input  wire                      PRESETn, // global active low reset in apb side

    output reg                       pclk_start, // internal control signal which tells to start processing the captured AHB request now.
    // AHB transaction information after it has reached the PCLK domain
    output reg [ADDR_WIDTH-1:0]      pclk_addr,
    output reg                       pclk_write,
    output reg [DATA_WIDTH-1:0]      pclk_wdata,
    output reg [STRB_WIDTH-1:0]      pclk_strb,
    output reg [N_SLAVES-1:0]        pclk_slave_sel,
    output reg                       pclk_decode_err,

    input  wire                      pclk_done, // Indicates the APB transaction has completed
    input  wire [DATA_WIDTH-1:0]     pclk_rdata, // Contains the read data received from the APB slave (PRDATA)
    input  wire                      pclk_err  // Indicates the APB slave reported an error (PSLVERR = 1)
);

reg req_toggle; // indicates a new AHB request has been generated and needs to be processed in the PCLK domain.

//the internal hreq_* signals because they act as a stable holding register for the AHB transaction while the request crosses from HCLK to PCLK.
reg [ADDR_WIDTH-1:0] hreq_addr;
reg                  hreq_write;
reg [DATA_WIDTH-1:0] hreq_wdata;
reg [STRB_WIDTH-1:0] hreq_strb;
reg [N_SLAVES-1:0]   hreq_slave_sel;
reg                  hreq_decode_err;

reg req_data_pending; //one-cycle HCLK-domain staging flag that ensures the request data is captured and stable before req_toggle announces the new request

reg rsp_toggle; //indicates the APB transaction has completed and a response is available for the HCLK domain.
reg [DATA_WIDTH-1:0] hrsp_rdata; // holds the APB response data stable while rsp_toggle crosses from PCLK to HCLK.
reg                  hrsp_err; // holds the APB error status stable while the response crosses back to HCLK

reg rsp_data_pending; //PCLK-domain flag that tells your CDC logic that the APB response has been captured, but rsp_toggle has not yet been changed.
reg req_meta; // 1st synchronizer stage receives the asynchronous req_toggle signal from hclk
reg req_sync; // 2nd synchronizer stage give metastability to settle before signal is used in pclk logic
reg req_sync_d; // stores previous value so that you can detect a change

//  request toggle synchronizer from hclk to pclk domain
always @(posedge PCLK or negedge PRESETn) begin
	if (!PRESETn) begin
		req_meta   <= 1'b0;
    req_sync   <= 1'b0;
    req_sync_d <= 1'b0;
  end
  else begin
    req_meta   <= req_toggle;
    req_sync   <= req_meta;
    req_sync_d <= req_sync;
  end
end

wire new_request;
assign new_request = req_sync ^ req_sync_d; // if present and previous value are same no new request

//  response toggle synchronizer from pclk to hclk domain
// rsp_toggle is generated in the pclk domain and crosses into the hclk domain through 2 ff synchronizer
reg rsp_meta;
reg rsp_sync;
reg rsp_sync_d;

always @(posedge HCLK or negedge HRESETn)begin
	if (!HRESETn) begin
		rsp_meta   <= 1'b0;
		rsp_sync   <= 1'b0;
    rsp_sync_d <= 1'b0;
	end
  else begin
    rsp_meta   <= rsp_toggle;
    rsp_sync   <= rsp_meta;
    rsp_sync_d <= rsp_sync;
  end
end

// a change in the synchronized response toggle indicates that a new response has arrived in the hclk domain
wire response_seen;
assign response_seen = rsp_sync ^ rsp_sync_d;

// holding registers before the request toggle is changed, they must be stable while the request is crossing from hclk to pclk
always @(posedge HCLK or negedge HRESETn) begin
	if (!HRESETn) begin
		req_toggle     <= 1'b0;
  	req_data_pending <= 1'b0;
		hreq_addr       <= {ADDR_WIDTH{1'b0}};
    hreq_write      <= 1'b0;
    hreq_wdata      <= {DATA_WIDTH{1'b0}};
    hreq_strb       <= {STRB_WIDTH{1'b0}};
    hreq_slave_sel  <= {N_SLAVES{1'b0}};
    hreq_decode_err <= 1'b0;
	end
	else begin
    // accept a new AHB request only when its valid, previous request has already completed and no request data is currently waiting to be announced
		if (req_valid &&(req_toggle ==rsp_sync) && !req_data_pending)begin 
			hreq_addr       <= req_addr;
      hreq_write      <= req_write;
      hreq_wdata      <= req_wdata;
      hreq_strb       <= req_strb;
      hreq_slave_sel  <= slave_sel;
      hreq_decode_err <= decode_err;
      // delay the request toggle by one HCLK cycle to become stabel before crossing.
			req_data_pending <= 1'b1;
    end
		else if(req_data_pending)begin // now can accept a new reuest
			req_toggle      <= ~req_toggle; // now req_toggle is not equal to rsp_sync, and request is outstanding. during this my hreq* must not change
      req_data_pending <= 1'b0;
    end
	end
end


// pclk domain receives AHB request
// detects the synchronized request toggle , the stable hclk domain registers and sample into pclk domain registers.
always @(posedge PCLK or negedge PRESETn) begin
	if (!PRESETn) begin
		pclk_start      <= 1'b0;
    pclk_addr       <= {ADDR_WIDTH{1'b0}};
    pclk_write      <= 1'b0;
    pclk_wdata      <= {DATA_WIDTH{1'b0}};
    pclk_strb       <= {STRB_WIDTH{1'b0}};
    pclk_slave_sel  <= {N_SLAVES{1'b0}};
    pclk_decode_err <= 1'b0;
	end
  else begin
		pclk_start <= 1'b0;
		if (new_request) begin
      // capture the stable transaction information from HCLK domain holding registers.
    	pclk_addr       <= hreq_addr;
      pclk_write      <= hreq_write;
      pclk_wdata      <= hreq_wdata;
      pclk_strb       <= hreq_strb;
      pclk_slave_sel  <= hreq_slave_sel;
      pclk_decode_err <= hreq_decode_err;
			pclk_start <= 1'b1; // generated for one PCLK cycle to tell the APB side logic to start processing the tranaction
    end
  end
end

// captures the apb response data and error status into holding registers before changing rsp_toggle
always @(posedge PCLK or negedge PRESETn) begin
	if (!PRESETn) begin
		rsp_toggle       <= 1'b0;
  	rsp_data_pending <= 1'b0;
		hrsp_rdata <= {DATA_WIDTH{1'b0}};
  	hrsp_err   <= 1'b0;
	end
  else begin
    // apb transaction has completed and no response is currently waiting to be announced.
		if (pclk_done && !rsp_data_pending) begin
			hrsp_rdata <= pclk_rdata;
      hrsp_err   <= pclk_err;
      // wait one PCLK cycle before toggling rsp_toggle to make response data stable
			rsp_data_pending <= 1'b1;
    end
		else if (rsp_data_pending) begin
			rsp_toggle       <= ~rsp_toggle; // telss that new response is available
			rsp_data_pending <= 1'b0;

    end
	end
end

// hclk domain receives APB response
// once response is detected the stable response data and error status are transferred to the hclk omain
always @(posedge HCLK or negedge HRESETn) begin
	if (!HRESETn) begin
		rsp_valid <= 1'b0;
    rsp_rdata <= {DATA_WIDTH{1'b0}};
    rsp_err   <= 1'b0;
	end
  else begin
		rsp_valid <= 1'b0;
		if(response_seen)begin
			rsp_rdata <= hrsp_rdata;
      rsp_err   <= hrsp_err;
			rsp_valid <= 1'b1; // tells the ahb side that valid response is available
    end
	end
end
endmodule
