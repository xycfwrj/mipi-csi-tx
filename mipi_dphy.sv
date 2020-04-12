module mipi_blvds (
        //Reset and Clocks
        input        max10_resetn       ,
        input        clk50m_max10       ,
        //User IO: LED/PB/DIPSW
        inout  [9:0] user_io            ,
        output [4:0] user_led           ,
        input  [3:0] user_pb            ,
        /*** Tx MIPI Channel ***/
        //input         FS        , // start transmit a frame
        //input  [31:0] CRC       ,
        /* Low power lane data */
        output       mipi_tx_clk_lp_p   ,
        output       mipi_tx_data_0_lp_p,
        output       mipi_tx_data_1_lp_p,
        output       mipi_tx_data_2_lp_p,
        output       mipi_tx_data_3_lp_p,
        output       mipi_tx_clk_lp_n   ,
        output       mipi_tx_data_0_lp_n,
        output       mipi_tx_data_1_lp_n,
        output       mipi_tx_data_2_lp_n,
        output       mipi_tx_data_3_lp_n,
        /* Differential lane data  */
        output       mipi_tx_clk_hs_p   ,
        output       mipi_tx_data_0_hs_p
        //output          mipi_tx_data_1_hs_p,
        //output          mipi_tx_data_2_hs_p,
        //output          mipi_tx_data_3_hs_p,
        //output          mipi_tx_clk_hs_n,
        //output          mipi_tx_data_0_hs_n,
        //output          mipi_tx_data_1_hs_n,
        //output          mipi_tx_data_2_hs_n,
        //output          mipi_tx_data_3_hs_n
);

        wire       locked         ;
        wire       byteclk,bitclk,bitclk90;
        reg  [7:0] data    = 8'hb8;
        reg  [1:0] data2b         ;
        wire       coreclk        ;
        reg  [7:0] acnt           ;
        reg  [1:0] bcnt           ;
        reg        areset         ;
        reg        hsoe           ;
        reg  [3:0] lp_p, lp_n;
        reg  [3:0] dphy_st        ;
        
        reg        st             ;

        wire FS;

        assign FS = ~user_pb[3];

        //50mhz in, 12.5mhz ,50mhz,50mhz 90degree
        pllm pllm_inst (clk50m_max10,byteclk,bitclk,bitclk90,locked);
        //gpio lite: pseudo diff, oe, ddr,
        oddr dphy_hstl (bitclk,data2b,mipi_tx_data_0_hs_p,mipi_tx_data_0_hs_n,hsoe);
        oddr dphy_hstl_c (bitclk90,2'b10,mipi_tx_clk_hs_p,mipi_tx_clk_hs_n,hsoe);

        assign mipi_tx_clk_lp_p    = 1'bz;
        assign mipi_tx_data_0_lp_p = (hsoe) ? 1'bz : lp_p[0];
        assign mipi_tx_data_1_lp_p = 1'bz;
        assign mipi_tx_data_2_lp_p = 1'bz;
        assign mipi_tx_data_3_lp_p = 1'bz;
        assign mipi_tx_clk_lp_n    = 1'bz;
        assign mipi_tx_data_0_lp_n = (hsoe) ? 1'bz : lp_n[0];
        assign mipi_tx_data_1_lp_n = 1'bz;
        assign mipi_tx_data_2_lp_n = 1'bz;
        assign mipi_tx_data_3_lp_n = 1'bz;
        assign user_io[0]          = byteclk;  //pin1
        assign user_io[5]          = dphy_st[0];  //pin2
        assign user_io[1]          = bcnt[1];  //pin3
        assign user_io[6]          = locked ;  //pin4
        assign user_io[2]          = dphy_st[1];
        assign user_led            = 5'b11110;

        //FSMD-- TI snla303 Fig2
        //LP-11()->LP-01(50ns)->LP-00(40+4ui,85+6ui)->HS-Zero(105+6ui)->HS-Sync8b->
        //Hd32b->Payload->CRC16b->HS-TRAIL(60+4UI)->LP-11
        localparam [3:0] 
                INIT = 4'he, 
                LP11=4'h1,
                LP01=4'h2, 
                LP00=4'h3, 
                HS0=4'h4, 
                HSSYNC=4'h8, 
                HSHD = 4'h9, 
                HSPAYLOAD = 4'ha, 
                HSCRC = 4'hb, 
                HSTRAIL = 4'hc, 
                HSEXIT = 4'hd;

        reg        FSr,FSrr;
        reg lockedr,lockedrr;
        always@(posedge bitclk) 
        begin
                lockedr <= locked;
                lockedrr <= lockedr;
                FSr  <= FS;
                FSrr <= FSr;  //cross clock domain
        end

        reg [16:0] cnt            ;
        reg [31:0] CSIHD          ;
        reg [31:0] datao, payload;
        reg [31:0] lastbitsflipped;
        always@(posedge bitclk)
                if(~lockedrr) begin
                        dphy_st <= INIT;
                        datao   <= 32'h00;
                        CSIHD   <= 32'hffffffff;
                        //lp_clk_p <= 1'b0;
                        //lp_clk_n <= 1'b0;
                        lp_p    <= 1'b1;
                        lp_n    <= 1'b1;
                        hsoe    <= 1'b0;
                        bcnt    <= 2'd0;
                        data    <= 8'hb8;
                end else begin
                        bcnt <= bcnt + 1'b1;
                        data <= 8'hb8;
                        if(bcnt == 2'b00) begin
                                //data   <= datao[7:0];
                                data2b <= {data[1], data[0]};
                        end else if(bcnt == 2'b01) begin
                                data2b <= {data[3], data[2]};
                        end else if(bcnt == 2'b10) begin
                                data2b <= {data[5], data[4]};
                        end else begin
                                data2b <= {data[7], data[6]};
                        end

                        if(bcnt == 2'b00) begin
                                case(dphy_st)
                                        INIT : begin
                                                dphy_st <= LP11;
                                                lp_p    <= 1'b1;
                                                lp_n    <= 1'b1;
                                                hsoe    <= 1'b0;
                                                cnt     <= 17'd0;
                                        end
                                        LP11 : if(FSrr) begin
                                                dphy_st <= LP01;
                                                lp_p    <= 1'b0;
                                                lp_n    <= 1'b1;
                                                hsoe    <= 1'b0;
                                                cnt     <= 17'd0;
                                        end
                                        LP01 : if(cnt < 20)
                                                cnt <= cnt + 1'b1;
                                        else begin
                                                dphy_st <= LP00;
                                                lp_p    <= 1'b0;
                                                lp_n    <= 1'b0;
                                                hsoe    <= 1'b0;
                                                cnt     <= 17'd0;
                                        end
                                        LP00 : if(cnt < 2) begin //80us, need change for high speed
                                                cnt <= cnt + 1'b1;
                                        end else begin
                                                dphy_st <= HS0;
                                                lp_p    <= 1'b0;
                                                lp_n    <= 1'b0;
                                                hsoe    <= 1'b1;  //
                                                cnt     <= 17'd0;
                                                datao   <= 32'h00;
                                        end
                                        HS0 : if(cnt<2) begin
                                                cnt  <= cnt + 1'b1;
                                        end else begin
                                                dphy_st <= HSSYNC;
                                                datao   <= 32'hB8B8B8B8;
                                                st      <= 1'b1;
                                        end
                                        HSSYNC : begin
                                                dphy_st <= HSHD;
                                                datao <= CSIHD;
                                                cnt     <= 17'd0;
                                        end
                                        HSHD : begin
                                                dphy_st <= HSPAYLOAD;
                                                datao   <= {16'h543a, 16'h96a3}; //cnt[15:0]}; //data;  //test data
                                                cnt     <= 17'd0;
                                                //generate FIFO signals here to output data, FWFT
                                        end
                                        HSPAYLOAD : if(cnt < 17'd255) begin  //in test, a line has 65536B
                                                cnt     <= cnt + 1'b1;
                                                datao   <= 32'haabbcca3; //;
                                                payload <= 32'haabbcca3; //;
                                        end else begin
                                                dphy_st <= HSCRC;
                                                datao   <= 32'habcd0123;  //CRC
                                        end
                                        HSCRC : begin
                                                dphy_st <= HSTRAIL;
                                                //datao   <= {8{~payload[31]},8{~payload[23]},8{~payload[15]},8{~payload[7]}};  //last bit of payload
                                                cnt     <= 17'd0;
                                        end
                                        HSTRAIL : if(cnt<2) begin
                                                cnt <= cnt + 1'b1;
                                        end else begin
                                                dphy_st <= HSEXIT;
                                                lp_p    <= 1'b1;
                                                lp_n    <= 1'b1;
                                                hsoe    <= 1'b0;
                                                cnt     <= 17'd0;
                                        end
                                        HSEXIT : if(cnt < 25) begin
                                                cnt  <= cnt + 1'b1;
                                        end else begin
                                                dphy_st <= INIT;
                                        end
                                        default : dphy_st <= INIT;
                                endcase;
                        end
                end

endmodule
