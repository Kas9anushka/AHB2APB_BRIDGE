`default_nettype none
module ahb_slave#(
    parameter ADDR_WIDTH = 32,
    parameter DATA_WIDTH = 32,
    parameter SLAVE_COUNT = 8
)(
    input wire PCLK,
    input wire PRESETn,
    input wire valid_transfer,
    // signals from ahb slave
    input wire [ADDR_WIDTH-1:0] addr_in,
    input wire [DATA_WIDTH-1:0] wdata_in,
    input wire write_in,
    input wire [2:0]size_in,
    input wire [SLAVE_COUNT-1:0]psel_in,

    // apb master interface
    output reg [ADDR_WIDTH-1:0]PADDR,
    output reg [DATA_WIDTH-1:0] PWDATA,
    output reg PWRITE,
    output reg [SLAVE_COUNT-1:0]PSEL,
    output reg PENABLE,
    output reg [DATA_WIDTH/8-1:0]PSTRB,
    input wire [DATA_WIDTH-1:0]PRDATA,
    input wire PREADY,
    input wire PSLVERR  
);

localparam [2:0] 
    s_IDLE = 3'd0,
    S_SETUP = 3'd1,
    S_ENABLE = 3'd2,
    S_WAIT = 3'd3,
    S_WB = 3'd4;

// byte strobe generation
function [DATA_WIDTH/8-1:0]gen_strobe;
    input [2:0]size;
    input [1:0]addr_lsb;
    begin
        case(size)
            3'b000: 
                case(addr_lsb)
                    2'b00: gen_strobe = 4'b0001;
                    2'b01: gen_strobe = 4'b0010;
                    2'b10: gen_strobe = 4'b0100;
                    2'b11: gen_strobe = 4'b1000;
                endcase
            3'b001: 
                case(addr_lsb[1])
                    1'b0: gen_strobe = 4'b0011;
                    1'b1: gen_strobe = 4'b1100;
                endcase
                default: gen_strobe = 4'b1111;
        endcase
    end
endfunction

reg [2:0]state, nextstate;

// fsm state register
always @(posedge PCLK or negedge PRESETn)begin
    if(!PRESET_n)
        state <= S_IDLE;
    else
        state <= nextstate;
end

always @(*)begin
    nextstate = state;
    hready_out = 1'b1;
    hresp_next = 1'b0;
    PSEL = 8'b0;
    PENABLE = 1'b0;
    PADDR = addr_in;
    PWRITE = write_in;
    PWDATA = wdata_in;
    PSTRB = gen_strobe(size_in, addr_in[1:0]);
    case(state)
        S_IDLE: begin
            hready_out = 1'b1;
            if(valid_transfer)begin
                hready_out = 1'b0;
                nextstate = S_SETUP;
            end
        end
        S_SETUP: begin
            PSEL = psel_in;
            PENABLE = 1'b0;
            PADDR = addr_in;
            PWRITE = write_in;
            PWDATA = wdata_in;
            PSTRB = gen_strobe(size_in, addr_in[1:0]);
            hready_out = 1'b0;
            nextstate = S_ENBALE;
        end
        S_ENABLE: begin
            PSEL = psel_in;
            PENABLE = 1'b1;
            hready_out = 1'b0;
            if(PREADY)begin
                if(PSLVERR)begin
                    hresp_next = 1'b1;
                    hready_out = 1'b0;
                    nextstate = S_WB;
                end else begin
                    hready_out = 1'b1;
                    if(valid_transfer)
                        nextstate = S_SETUP;
                    else 
                        nextstate = S_IDLE;
                end
            end else begin
                nextstate = S_WAIT;
            end
        end
        S_WAIT: begin
            PSEL = psel_in;
            PENABLE = 1'b1;
            hready_out = 1b0;
            if(PREADY)begin
                if(PSLVERR)begin
                    hresp_next = 1'b1;
                    hready_out = 1'b0;
                    nextstate = S_WB;
                end else begin
                    hready_out = 1'b1;
                    if(valid_transfer)
                        nextstate = S_SETUP;
                    else 
                        nextstate = S_IDLE;
                end
            end else begin
                nextstate = S_WAIT;
            end
        end
        S_WB: begin
            hready_out = 1'b1;
            hresp_next = 1'b1;
            nextstate = S_IDLE;
        end
        default: nextstate = S_IDLE;
        endcase
end
endmodule