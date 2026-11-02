`default_nettype none

module top #(
    parameter DATA_WIDTH     = 32,
    parameter ADDR_WIDTH     = 32,
    parameter N_SLAVES       = 8,
    parameter STRB_WIDTH     = DATA_WIDTH/8,
    parameter TIMEOUT_CYCLES = 16
)(
    input  wire                     HCLK,
    input  wire                     HRESETn,
    input  wire                     HSEL,
    input  wire [ADDR_WIDTH-1:0]    HADDR,
    input  wire                     HWRITE,
    input  wire [1:0]               HTRANS,
    input  wire [2:0]               HSIZE,
    input  wire [2:0]               HBURST,
    input  wire [DATA_WIDTH-1:0]    HWDATA,
    input  wire                     HREADY,
    output wire                     HREADYOUT,
    output wire                     HRESP,
    output wire [DATA_WIDTH-1:0]    HRDATA,

    input  wire                     PCLK,
    input  wire                     PRESETn,
    output wire [ADDR_WIDTH-1:0]    PADDR,
    output wire [N_SLAVES-1:0]      PSEL,
    output wire                     PENABLE,
    output wire                     PWRITE,
    output wire [DATA_WIDTH-1:0]    PWDATA,
    output wire [STRB_WIDTH-1:0]    PSTRB,
    input  wire [DATA_WIDTH-1:0]    PRDATA,
    input  wire                     PREADY,
    input  wire                     PSLVERR
);

    // address decoder
    wire [N_SLAVES-1:0]   slave_sel;
    wire                  dec_err;

    wire                  apb_start;
    wire [ADDR_WIDTH-1:0] req_addr;
    wire                  req_write;
    wire [DATA_WIDTH-1:0] req_wdata;
    wire [STRB_WIDTH-1:0] req_strb;
    wire [N_SLAVES-1:0]   req_slave_sel;
    wire                  rsp_valid;
    wire [DATA_WIDTH-1:0] rsp_rdata;
    wire                  rsp_err;

    wire                  pclk_start;
    wire [ADDR_WIDTH-1:0] pclk_addr;
    wire                  pclk_write;
    wire [DATA_WIDTH-1:0] pclk_wdata;
    wire [STRB_WIDTH-1:0] pclk_strb;
    wire [N_SLAVES-1:0]   pclk_slave_sel;
    wire                  pclk_done;
    wire [DATA_WIDTH-1:0] pclk_rdata;
    wire                  pclk_err;

    ADDRESS_DECODER #(
        .ADDR_WIDTH(ADDR_WIDTH),
        .N_SLAVES  (N_SLAVES)
    ) u_decoder (
        .addr      (HADDR),
        .slave_sel (slave_sel),
        .decode_err(dec_err)
    );

    AHB_CAPTURE #(
        .DATA_WIDTH(DATA_WIDTH),
        .ADDR_WIDTH(ADDR_WIDTH),
        .N_SLAVES  (N_SLAVES)
    ) u_ahb_capture (
        .HCLK         (HCLK),
        .HRESETn      (HRESETn),
        .HSEL         (HSEL),
        .HADDR        (HADDR),
        .HTRANS       (HTRANS),
        .HWRITE       (HWRITE),
        .HSIZE        (HSIZE),
        .HWDATA       (HWDATA),
        .HREADY       (HREADY),
        .HBURST       (HBURST),
        .dec_err      (dec_err),
        .slave_sel    (slave_sel),
        .HREADYOUT    (HREADYOUT),
        .HRDATA       (HRDATA),
        .HRESP        (HRESP),
        .resp_done    (rsp_valid),
        .resp_err     (rsp_err),
        .resp_rdata   (rsp_rdata),
        .apb_start    (apb_start),
        .req_addr     (req_addr),
        .req_write    (req_write),
        .req_wdata    (req_wdata),
        .req_strb     (req_strb),
        .req_slave_sel(req_slave_sel)
    );

    CDC_REQUEST #(
        .DATA_WIDTH(DATA_WIDTH),
        .ADDR_WIDTH(ADDR_WIDTH),
        .N_SLAVES  (N_SLAVES),
        .STRB_WIDTH(STRB_WIDTH)
    ) u_cdc (
        .HCLK           (HCLK),
        .HRESETn        (HRESETn),
        .apb_start      (apb_start),
        .req_addr       (req_addr),
        .req_write      (req_write),
        .req_wdata      (req_wdata),
        .req_strb       (req_strb),
        .slave_sel      (req_slave_sel),
        .rsp_valid      (rsp_valid),
        .rsp_rdata      (rsp_rdata),
        .rsp_err        (rsp_err),
        .PCLK           (PCLK),
        .PRESETn        (PRESETn),
        .pclk_start     (pclk_start),
        .pclk_addr      (pclk_addr),
        .pclk_write     (pclk_write),
        .pclk_wdata     (pclk_wdata),
        .pclk_strb      (pclk_strb),
        .pclk_slave_sel (pclk_slave_sel),
        .pclk_done      (pclk_done),
        .pclk_rdata     (pclk_rdata),
        .pclk_err       (pclk_err)
    );

    APB_FSM #(
        .DATA_WIDTH    (DATA_WIDTH),
        .ADDR_WIDTH    (ADDR_WIDTH),
        .N_SLAVES      (N_SLAVES),
        .STRB_WIDTH    (STRB_WIDTH),
        .TIMEOUT_CYCLES(TIMEOUT_CYCLES)
    ) u_apb_fsm (
        .PCLK          (PCLK),
        .PRESETn       (PRESETn),
        .pclk_start    (pclk_start),
        .pclk_addr     (pclk_addr),
        .pclk_write    (pclk_write),
        .pclk_wdata    (pclk_wdata),
        .pclk_strb     (pclk_strb),
        .pclk_slave_sel(pclk_slave_sel),
        .pclk_done     (pclk_done),
        .pclk_rdata    (pclk_rdata),
        .pclk_err      (pclk_err),
        .PADDR         (PADDR),
        .PSEL          (PSEL),
        .PENABLE       (PENABLE),
        .PWRITE        (PWRITE),
        .PWDATA        (PWDATA),
        .PSTRB         (PSTRB),
        .PRDATA        (PRDATA),
        .PREADY        (PREADY),
        .PSLVERR       (PSLVERR)
    );

endmodule
