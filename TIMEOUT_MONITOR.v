`default_nettype none

module TIMEOUT_MONITOR 
#(
    parameter TIMEOUT_CYCLES = 16 // max number of cycle PCLK allowed to remain in access phase 
)(
    input  wire PCLK, // clock from slave domain of apb
    input  wire PRESETn, // active low reset
    input  wire apb_access_active, // APB is in ACCESS phase and PREADY signal is low
    output reg  timeout_hit        // 1-cycle pulse indicating that the APB ha exceeded the programmed timeout limit.
);  

// count width.
localparam CW = (TIMEOUT_CYCLES <= 1) ? 1 : $clog2(TIMEOUT_CYCLES + 1);
// counter register 
reg [CW-1:0] cnt;


//sequential block that will dictate timeout_hit signal

always @(posedge PCLK or negedge PRESETn) begin
	if (!PRESETn) begin
     cnt         <= {CW{1'b0}};
		 timeout_hit <= 1'b0;
	end else begin
		 timeout_hit <= 1'b0;
		 if (!apb_access_active) begin
				cnt <= {CW{1'b0}};
		 end else if (cnt == TIMEOUT_CYCLES - 1) begin
				timeout_hit <= 1'b1;
				cnt         <= {CW{1'b0}};
		 end else begin
				cnt <= cnt + 1'b1;
		 end
	end
end
endmodule
