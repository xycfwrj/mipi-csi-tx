
//HS_CLK always ON
//assume data number is always N = 4*M, 4 lanes have synchronized SOT&EOT
//data structure, when receiving FS, transmit a frame of data via CSI:
// [short packet: FS(frame start) fno][long packet: #1(64KB)][long packet: #2(64KB)]
//...........[LgP: #lnum-1(64KB)][short packet: FE fno]

//in packet, Header CRC is fixed
//payload CRC is calculated
module dsi_tx_phy (
	input         byteclk   , // test 25MHz
	input         bitclk    , // test 100MHz
	input         rstn      , // Asynchronous reset active low
	input         FS        , // start transmit a frame
	input  [15:0] fno       ,
	input  [ 7:0] lnum      , //transmit line number each frame
	input  [31:0] data      ,
	input  [31:0] CRC       ,
	output        hs_clk_o  , //diff pair P
	output [ 3:0] hs_d_o    , //diff pairs P
	output        lp_clk_p_o,
	output        lp_clk_n_o,
	output [ 3:0] lp_p_o    ,
	output [ 3:0] lp_n_o
);

	reg        lp_oe, lp_clk_oe;
	reg  [3:0] lp_p, lp_n;
	reg        lp_clk_p, lp_clk_n;
	wire       hs_clk,coreclk;
	wire [3:0] hs_dat  ;

	//LP output
	genvar I;
	generate
		for(I=0; I<4;I=I+1) begin
			assign lp_p_o[I] = (lp_p[I] == '1' && lp_oe == '1') ? 1'b1: 1'bz;
			assign lp_n_o[I] = (lp_n[I] == '1' && lp_oe == '1') ? 1'b1: 1'bz;
		end
	endgenerate
	assign lp_clk_p_o = (lp_clk_p == '1' && lp_oe == '1') ? 1'b1: 1'bz;
	assign lp_clk_n_o = (lp_clk_n == '1' && lp_oe == '1') ? 1'b1: 1'bz;
	//HS output -- Serdes Factor 8, alt soft lvds
	reg        areset = 1;
	reg [31:0] datao     ;
	serdesx8 csi_4lane_serd (  //MSB first, need reverse datao !!!
		.pll_areset  (areset  ), //assert for at least 10ns, then wait pll_lock(wait 4us)
		.tx_in       (datao   ),
		.tx_inclock  (byteclk ),
		.tx_out      (hs_d_o  ),
		.tx_outclock (hs_clk_o),
		.tx_coreclock(coreclk )  //core clock to register data
	);
	reg [7:0] acnt;
	always@(posedge coreclk)
		if(areset || !rstn) begin
			acnt   <= 8'd0;
			areset <= 1'b1;
		end else begin
			if(acnt < 8'd100) begin
				areset <= 1'b1;
				acnt   <= acnt + 1'b1;
			end else
			areset <= 1'b0;

		end

	//FSMD-- TI snla303 Fig2
	//LP-11()->LP-01(50ns)->LP-00(40+4ui,85+6ui)->HS-Zero(105+6ui)->HS-Sync8b->
	//Hd32b->Payload->CRC16b->HS-TRAIL(60+4UI)->LP-11
	parameter INIT    = 4'h0, LP11=4'h1,LP01=4'h2, LP00=4'h3, HS0=4'h4, HSSYNC=4'h8, HSHD = 4'h9, HSPAYLOAD = 4'ha, HSCRC = 4'hb, HSTRAIL = 4'hc, HSEXIT = 4'hd;
	reg [3:0] dphy_st                                                                                                                           ;
	reg       FSr,FSrr;
	reg payload;
	always@(posedge coreclk)
		if(areset || !rstn) begin
			dphy_st <= INIT;
			FSr     <= 1'b0;
			FSrr    <= 1'b0;
		end else begin
			FSr  <= FS;
			FSrr <= FSrr;  //cross clock domain
		end

	reg [16:0] cnt            ;
	reg [31:0] CSIHD          ;
	reg [31:0] lastbitsflipped;
	always@(posedge coreclk)
		if(areset || !rstn) begin
			dphy_st <= INIT;
			FSr     <= 1'b0;
			FSrr    <= 1'b0;
			datao   <= 32'h00;
			CSIHD   <= 32'hffffffff;
			lp_clk_p <= 1'b0;
			lp_clk_n <= 1'b0;
			lp_p    <= 1'b1;
			lp_n    <= 1'b1;
			lp_oe   <= 1'b1;			
		end else begin
			case(dphy_st)
				INIT : begin
					dphy_st <= LP11;
					lp_p    <= 1'b1;
					lp_n    <= 1'b1;
					lp_oe   <= 1'b1;
					cnt     <= 17'd0;
				end
				LP11 : if(FSrr) begin
					dphy_st <= LP01;
					lp_p    <= 1'b0;
					lp_n    <= 1'b1;
					lp_oe   <= 1'b1;
					cnt     <= 17'd0;
				end
				LP01 : if(cnt < 20)
					cnt <= cnt + 1'b1;
				else begin
					dphy_st <= LP00;
					lp_p    <= 1'b0;
					lp_n    <= 1'b0;
					lp_oe   <= 1'b1;
					cnt     <= 17'd0;
				end
				LP00 : if(cnt < 2) begin //80us, need change for high speed
					cnt <= cnt + 1'b1;
				end else begin
					dphy_st <= HS0;
					lp_p    <= 1'b0;
					lp_n    <= 1'b0;
					lp_oe   <= 1'b1;  //
					cnt     <= 17'd0;
					datao   <= 32'h00;
				end
				HS0 : if(cnt<2) begin
					cnt <= cnt + 1'b1;
				end else begin
					dphy_st <= HSSYNC;
					datao   <= 32'hB8B8B8B8;
				end
				HSSYNC : begin
					dphy_st <= HSHD;
					dphy_st <= CSIHD;
					cnt     <= 17'd0;
				end
				HSHD : begin
					dphy_st <= HSPAYLOAD;
					datao   <= {16'd543a, cnt[15:0]}; //data;  //test data
					cnt     <= 17'd0;
					//generate FIFO signals here to output data, FWFT
				end
				HSPAYLOAD : if(cnt<=17'd16384) begin  //in test, a line has 65536B
					cnt   <= cnt + 1'b1;
					datao <= data;
					payload <= data;
				end else begin
					dphy_st <= HSCRC;
					datao   <= 32'habcd0123;  //CRC
				end
				HSCRC : begin
					dphy_st <= HSTRAIL;
					datao   <= {8{~payload[31]},8{~payload[23]},8{~payload[15]},8{~payload[7]}};  //last bit of payload
					cnt <= 17'd0;
				end
				HSTRAIL : if(cnt<2) begin
					cnt <= cnt + 1'b1;
				end else begin
					dphy_st <= HSEXIT;
					lp_p    <= 1'b1;
					lp_n    <= 1'b1;
					lp_oe   <= 1'b1;
					cnt <= 17'd0;
				end
				HSEXIT : if(cnt < 25) begin
					cnt <= cnt + 1'b1;
				end else begin
					dphy_st <= INIT;
				end 
				default : dphy_st <= INIT;
			endcase;
		end

endmodule