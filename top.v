`default_nettype none

module top #(
    parameter DATA_WIDTH     = 32,
    parameter ADDR_WIDTH     = 32,
    parameter N_SLAVES       = 8,
    parameter TIMEOUT_CYCLES = 16,
    parameter STRB_WIDTH     = DATA_WIDTH/8
)(
    // ------------- AHB-Lite slave -------------
    input  wire                     HCLK,
    input  wire                     HRESETn,
    input  wire                     HSEL,
    input  wire [ADDR_WIDTH-1:0]    HADDR,
    input  wire [1:0]               HTRANS,
    input  wire                     HWRITE,
    input  wire [2:0]               HSIZE,
    input  wire [DATA_WIDTH-1:0]    HWDATA,
    input  wire                     HREADY,
    output wire                     HREADYOUT,
    output wire [DATA_WIDTH-1:0]    HRDATA,
    output wire                     HRESP,

    // ------------- APB4 master -------------
    input  wire                     PCLK,
    input  wire                     PRESETn,
    output wire [ADDR_WIDTH-1:0]    PADDR,
    output wire [N_SLAVES-1:0]      PSEL,
    output wire                     PENABLE,
    output wire                     PWRITE,
    output wire [DATA_WIDTH-1:0]    PWDATA,
    output wire [STRB_WIDTH-1:0]    PSTRB,
    output wire [2:0]               PPROT,
    input  wire [DATA_WIDTH-1:0]    PRDATA,
    input  wire                     PREADY,
    input  wire                     PSLVERR
);

    // ---- HCLK-domain request bundle (ahb_capture -> addr_decoder/cdc) ----
    wire                     req_valid;
    wire [ADDR_WIDTH-1:0]    req_addr;
    wire                     req_write;
    wire [DATA_WIDTH-1:0]    req_wdata;
    wire [STRB_WIDTH-1:0]    req_strb;
    wire                     resp_done;

    wire [N_SLAVES-1:0]      slave_sel;
    wire                     decode_err;

    // ---- HCLK-domain response bundle (cdc -> error_response) ----
    wire                     rsp_valid;
    wire [DATA_WIDTH-1:0]    rsp_rdata;
    wire                     rsp_err;

    // ---- PCLK-domain request bundle (cdc -> apb_fsm) ----
    wire                     pclk_start;
    wire [ADDR_WIDTH-1:0]    pclk_addr;
    wire                     pclk_write;
    wire [DATA_WIDTH-1:0]    pclk_wdata;
    wire [STRB_WIDTH-1:0]    pclk_strb;
    wire [N_SLAVES-1:0]      pclk_slave_sel;
    wire                     pclk_decode_err;

    // ---- PCLK-domain response bundle (apb_fsm -> cdc) ----
    wire                     pclk_done;
    wire [DATA_WIDTH-1:0]    pclk_rdata;
    wire                     pclk_err;

    wire                     apb_access_active;
    wire                     timeout_hit;

    AHB_CAPTURE #(
        .DATA_WIDTH(DATA_WIDTH), .ADDR_WIDTH(ADDR_WIDTH)
    ) u_ahb_capture (
        .HCLK(HCLK), .HRESETn(HRESETn),
        .HSEL(HSEL), .HADDR(HADDR), .HTRANS(HTRANS), .HWRITE(HWRITE),
        .HSIZE(HSIZE), .HWDATA(HWDATA),
        .HREADY(HREADY), .HREADYOUT(HREADYOUT),
        .resp_done(resp_done),
        .req_valid(req_valid), .req_addr(req_addr), .req_write(req_write),
        .req_wdata(req_wdata), .req_strb(req_strb)
    );

    ADDRESS_DECODER #(
        .ADDR_WIDTH(ADDR_WIDTH), .N_SLAVES(N_SLAVES)
    ) u_addr_decoder (
        .addr(req_addr),
        .slave_sel(slave_sel),
        .decode_err(decode_err)
    );

    CDC_REQUEST #(
        .DATA_WIDTH(DATA_WIDTH), .ADDR_WIDTH(ADDR_WIDTH), .N_SLAVES(N_SLAVES), .STRB_WIDTH(DATA_WIDTH/8)
    ) u_cdc (
        .HCLK(HCLK), .HRESETn(HRESETn),
        .req_valid(req_valid), .req_addr(req_addr), .req_write(req_write),
        .req_wdata(req_wdata), .req_strb(req_strb),
        .slave_sel(slave_sel), .decode_err(decode_err),
        .rsp_valid(rsp_valid), .rsp_rdata(rsp_rdata), .rsp_err(rsp_err),

        .PCLK(PCLK), .PRESETn(PRESETn),
        .pclk_start(pclk_start), .pclk_addr(pclk_addr), .pclk_write(pclk_write),
        .pclk_wdata(pclk_wdata), .pclk_strb(pclk_strb),
        .pclk_slave_sel(pclk_slave_sel), .pclk_decode_err(pclk_decode_err),
        .pclk_done(pclk_done), .pclk_rdata(pclk_rdata), .pclk_err(pclk_err)
    );

    APB_FSM #(
        .DATA_WIDTH(DATA_WIDTH), .ADDR_WIDTH(ADDR_WIDTH), .N_SLAVES(N_SLAVES),.STRB_WIDTH(DATA_WIDTH/8)
    ) u_apb_fsm (
        .PCLK(PCLK), .PRESETn(PRESETn),
        .pclk_start(pclk_start), .pclk_addr(pclk_addr), .pclk_write(pclk_write),
        .pclk_wdata(pclk_wdata), .pclk_strb(pclk_strb),
        .pclk_slave_sel(pclk_slave_sel), .pclk_decode_err(pclk_decode_err),
        .pclk_done(pclk_done), .pclk_rdata(pclk_rdata), .pclk_err(pclk_err),
        .PADDR(PADDR), .PSEL(PSEL), .PENABLE(PENABLE), .PWRITE(PWRITE),
        .PWDATA(PWDATA), .PSTRB(PSTRB), .PPROT(PPROT),
        .PRDATA(PRDATA), .PREADY(PREADY), .PSLVERR(PSLVERR),
        .apb_ACCESS_ST_active(apb_access_active), .timeout_hit(timeout_hit)
    );

    TIMEOUT_MONITOR #(
        .TIMEOUT_CYCLES(TIMEOUT_CYCLES)
    ) u_timeout (
        .PCLK(PCLK), .PRESETn(PRESETn),
        .apb_access_active(apb_access_active),
        .timeout_hit(timeout_hit)
    );

    RESPONSE_ERROR_MAPPER #(
        .DATA_WIDTH(DATA_WIDTH)
    ) u_error_response (
        .HCLK(HCLK), .HRESETn(HRESETn),
        .rsp_valid(rsp_valid), .rsp_rdata(rsp_rdata), .rsp_err(rsp_err),
        .HRDATA(HRDATA), .HRESP(HRESP), .resp_done(resp_done)
    );

endmodule
