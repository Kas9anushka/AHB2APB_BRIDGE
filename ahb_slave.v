'default_nettype none
module ahb_slave #(
    parameter DATA_WIDTH = 32,
    parameter ADDR_WIDTH = 32
    )(
    input wire HCLK,
    input wire HRESET_n,
    input wire [ADDR_WIDTH-1:0]HADDR,
    input wire [DATA_WIDTH-1:0]HWDATA,
    input wire HWRITE,
    input wire [1:0]HTRANS,
    input wire [2:0]HSIZE,
    input wire [2:0]HBURST,
    input wire HSEL,
    output reg [DATA_WIDTH-1:0]HRDATA,
    output wire HREADYOUT,
    output wire HRESP,

    // APB master interface 
    output reg [ADDR_WIDTH-1:0] paddr,
    output reg [DATA_WIDTH-1:0] pwdata,
    output reg pwrite,
    output reg psel,
    output reg penable,
    output reg [DATA_WIDTH/8-1:0]pstrb,
    input wire [DATA_WIDTH-1:0]prdata,
    input wire pready,
    input wire pslverr    
);

// state encoding
localparam [2:0] 
    s_IDLE = 3'd0,
    S_SETUP = 3'd1,
    S_ENABLE = 3'd2,
    S_WAIT = 3'd3,
    S_WB = 3'd4;

// htrans encoding 
localparam [1:0]
    T_IDLE = 2'd0,
    T_BUSY = 2'd1,
    T_NONSEQ = 2'd2,
    T_SEQ = 2'd3;

// hburst encoding
localparam [2:0]
    B_SINGLE = 3'b000,
    B_INCR = 3'd1,
    B_WRAP4 = 3'd2,
    B_INCR4 = 3'd3,
    B_WRAP8 = 3'd4,
    B_INCR8 = 3'd5,
    B_WRAP16 = 3'd6,
    B_INCR16 = 3'd7;

// internal signals
reg [2:0]state, nextstate;

// address phase latches
reg [ADDR_WIDTH-1:0] latch_addr; 
reg [DATA_WIDTH-1:0] latch_wdata;
reg latch_write;
reg [2:0] latch_size;
reg [2:0]latch_burst;

// burst_tracking

reg [4:0] beat_count;
reg burst_active;

// registered response signal
reg hready_out;
reg hresp_reg;
reg hresp_next;

// output assignment 
assign HREADYOUT = hready_out;
assign HRESP = {1'b0, hresp_reg};

// transfer validation

wire valid_transfer = HSEL && (HTRANS == T_NONSEQ || HTRANS == T_SEQ);

// burst length decode
function [4:0]burst_beats;
    input [2:0]hburst;
    begin
        case(hburst)
            B_WRAP4, B_INCR4 : burst_beats = 5'd3;
            B_WRAP8, B_INCR8 : burst_beats = 5'd7;
            B_WRAP16, B_INCR16 : burst_beats = 5'd15;
            default : burst_beats = 5'd0;
        endcase
    end
endfunction

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

// fsm state register
always @(posedge HCLK or negedge HRESETn)begin
    if(!HRESET_n)
        state <= S_IDLE;
    else
        state <= nextstate;
end

always @(posedge HCLK or negedge HRESETn)begin
    if(!HRESETn)
        hresp_reg <= 1'b0;
    else
        hresp_reg <= hresp_next;
end

// ahb slave fsm with address phase and data phase
always @(posedge HCLK or negedge HRESETn)begin
    if(!HRESETn)begin
        latch_addr <= 0;
        latch_wdata <= 0;
        latch_write <= 0;
        latch_size <= 0;
        latch_burst <= B_SINGLE;
        beat_count <= 0;
        burst_active <= 0;
    end else begin
        if(!HSEL && burst_active)begin
            burst_active <=0;
            beat_count <=0;
        end
        if(hready_out)begin
            case(HTRANS)
                T_NONSEQ: begin
                    if(HSEL)begin
                        latch_addr <= HADDR;
                        latch_write <= HWRITE;
                        latch_size <= HSIZE;
                        latch_burst <= HBURST;
                        beat_count <= burst_beats(HBURST);
                        burst_active <= (HBURST != BURST_SINGLE);
                    end
                end
                T_SEQ: begin
                    if(HSEL)begin
                        latch_addr <= HADDR;
                        latch_write <= HWRITE;
                        latch_size <= HSIZE;
                        // latch burst unchanged (same burst)
                        if(beat_count != 5'd0)
                            beat_count <= beat_count -5'd1;
                        burst_active <= (beat_count > 5'd1) || (latch_burst == B_INCR);
                    end
                end
                T_BUSY: begin
                end
                T_IDLE: begin
                    burst_active <= 1'b0;
                    beat_count <= 5'd0;
                end
                default: ;
            endcase
            if(valid_transfer)
                latch_wdata <= HWDATA;
        end
    end
end
always @(*)begin
    nextstate = state;
    hready_out = 1'b1;
    hresp_next = 1'b0;
    psel = 1'b0;
    penbale = 1'b0;
    paddr = latch_addr;
    pwrite = latch_write;
    pwdata = latch_wdata;
    pstrb = gen_strobe(latch_size, latch_addr[1:0]);
    case(state)
        S_IDLE: begin
            hready_out = 1'b1;
            if(valid_transfer)begin
                hready_out = 1'b0;
                nextstate = S_SETUP;
            end
        end
        S_SETUP: begin
            psel = 1'b1;
            penable = 1'b0;
            paddr = latch_addr;
            pwrite = latch_write;
            pwdata = latch_wdata;
            pstrb = gen_strobe(latch_size, latch_addr[1:0]);
            hready_out = 1'b0;
            nextstate = S_ENBALE;
        end
        S_ENABLE: begin
            psel = 1'b1;
            penable = 1'b1;
            hready_out = 1'b0;
            if(pready)begin
                if(pslverr)begin
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
            psel = 1'b1;
            penable = 1'b1;
            hready_out = 1b0;
            if(pready)begin
                if(pslverr)begin
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
// capture prdata into hrdata when apb read completes
always @(posedge HCLK or negedge HRESETn)begin
    if(!HRESETn)begin
        HRDATA <= 0;
    end else begin
        case (state)
            S_ENABLE: begin
                if(pready && !pslverr && !latch_write)
                    HRDATA <= prdata;
            end
            S_WAIT: begin
                if(pready && !pslverr && !latch_write)
                    HRDATA <= prdata;
            end
            default : ;
        endcase
    end
end
endmodule















