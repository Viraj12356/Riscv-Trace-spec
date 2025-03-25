// Code your testbench here
// or browse Examples
`timescale 1ns/1ps

module tb_riscv_trace_interface();
    parameter MAX_BITS = 80;
    parameter blocks_p = 1;
    parameter itype_width_p = 4;
    parameter iaddress_width_p = 32;
    parameter iretire_width_p = 2;
    parameter ilastsize_width_p = 1;
    parameter privilege_width_p = 2;
    parameter ecause_width_p = 4;
    parameter context_width_p = 1;
    parameter iaddress_lsb_p = 1;
	parameter time_width_p = 1;

    logic clk;
    logic rst_n;
    logic [0:0][3:0] itype;
    logic [ecause_width_p-1:0] cause;
    logic [iaddress_width_p-1:0] tval;
    logic [privilege_width_p-1:0] priv;
    logic [0:0][iaddress_width_p-1:0] iaddr;
    logic [context_width_p-1:0] context_;
    logic [0:0][iretire_width_p-1:0] iretire;
    logic [0:0][ilastsize_width_p-1:0] ilastsize;
    control::tr_te_control_t ctrl;
	//Packet outputs
	logic [MAX_BITS-1:0] bit_array;


    // ================== Redefine Packet Structs (Matching DUT) ==================
    typedef struct packed {
        logic [1:0] fmt;
        logic [1:0] sfmt;
        logic branch;
        logic [privilege_width_p-1:0] priv;
        logic [context_width_p-1:0] context_;
        logic [iaddress_width_p-iaddress_lsb_p-1:0] addr;
      logic [40:0] reserved;
    } fmt3_sync_pkt_t;

    typedef struct packed {
        logic [1:0] fmt;
        logic [1:0] sfmt;
        logic branch;
        logic [privilege_width_p-1:0] priv;
        logic [ecause_width_p-1:0] ecause;
        logic interrupt;
        logic thaddr;
        logic [iaddress_width_p-iaddress_lsb_p-1:0] addr;
        logic [iaddress_width_p-1:0] tval;
        logic [3:0] reserved; 
    } fmt3_trap_pkt_t;

    typedef struct packed {
        logic [1:0] fmt;
        logic [4:0] branches;
        logic [30:0] branch_map;
        logic [iaddress_width_p-iaddress_lsb_p-1:0] addr;
        logic notify;
        logic updiscon;
        logic irreport;
        logic [3:0] irdepth;
        logic [3:0] reserved; 
    } fmt1_branch_pkt_t;




    // Instantiate DUT
    riscv_trace_interface #(
        .MAX_BITS(MAX_BITS),
		.blocks_p(blocks_p),
        .iaddress_width_p(iaddress_width_p),
        .iretire_width_p(iretire_width_p),
        .ilastsize_width_p(ilastsize_width_p),
        .privilege_width_p(privilege_width_p),
        .ecause_width_p(ecause_width_p),
        .context_width_p(context_width_p),
        .iaddress_lsb_p(iaddress_lsb_p),
		.time_width_p(time_width_p)
    ) dut (
        .clk(clk),
        .rst_n(rst_n),
        .itype(itype),
        .cause(cause),
        .tval(tval),
        .priv(priv),
        .iaddr(iaddr),
        .context_(context_),
        .iretire(iretire),
        .ilastsize(ilastsize),
        .ctrl(ctrl),
        .bit_array(bit_array)
    );

    // Clock generation
    initial begin
        clk = 0;
        forever #5 clk = ~clk;
    end

    // Test sequence
    initial begin
        // Initialize inputs
        rst_n = 0;
        ctrl = '0;
        ctrl.trTeActive = 1;
        ctrl.trTeEnable = 1;
        ctrl.trTeInstSyncMax = 4; // resync_max = 4 (1 << (4+4) = 256)


        // Reset sequence
        #10;
        rst_n = 1;

        // Inject inputs (from encoder_input lines 38-47)
		
		
        // Cycle 1: UNINFERABLE_JUMP @80000154 (Line 38)
        itype = 6; // UNINFERABLE_JUMP
        iaddr = 32'h80000154;
        iretire = 1;
        ilastsize = 2;
        priv = 3;
        context_ = 0;
        cause = 0;
        tval = 0;
        #10;

        // Cycle 2: ITYPE_NONE @80000104 (Line 39)
        itype = 0;
        iaddr = 32'h80000104;
        iretire = 1;
        ilastsize = 4;
        priv = 3;
        context_ = 0;
        cause = 0;
        tval = 0;
        #10;

        // Cycle 3: ITYPE_NONE @80000108 (Line 40)
        itype = 0;
        iaddr = 32'h80000108;
        iretire = 1;
        ilastsize = 4;
        priv = 3;
        context_ = 0;
        cause = 0;
        tval = 0;
        #10;

        // Cycle 4: UNINFERABLE_JUMP @8000010c (Line 41)
        itype = 6;
        iaddr = 32'h8000010c;
        iretire = 1;
        ilastsize = 2;
        priv = 3;
        context_ = 0;
        cause = 0;
        tval = 0;
        #10;

        // Cycle 5: EXCEPTION @80000222 (Line 42)
        itype = 1; // EXCEPTION
        iaddr = 32'h80000222;
        iretire = 0; // NOT RETIRED (exc_only)
        cause = 2;
        priv = 3;
        context_ = 0;
        tval = 0;
        #10;

        // Cycle 6: ITYPE_NONE @800001b0 (Line 43)
        itype = 0;
        iaddr = 32'h800001b0;
        iretire = 1;
        ilastsize = 2;
        priv = 3;
        context_ = 0;
        cause = 0;
        tval = 0;
        #10;

        // Wait for packets to flush
        #100;
        $finish;
    end


  /*  always @(posedge clk) begin
        if (packet_valid) begin
            packet_union_t pkt_union;
            pkt_union.raw = packet_data;
            $display("\n=== Packet Detected ===");
            $display(pkt_union.raw[1:0]);
          $display(pkt_union.raw[3:2]);
          $display(packet_data);
            // Decode format
          case (pkt_union.raw[1:0])
            
                2'b11: begin // Format 3
                  case (pkt_union.raw[3:2])
                        2'b00: begin // Sync Start
                            $display("Type: FMT3_SYNC (Start)");
                            $display("  - branch: %b", pkt_union.sync_pkt.branch);
                            $display("  - priv: %d", pkt_union.sync_pkt.priv);
                            $display("  - context: %d", pkt_union.sync_pkt.context_);
                            $display("  - addr: 0x%h", pkt_union.sync_pkt.addr << iaddress_lsb_p);
                        end
                        2'b01: begin // Trap
                            $display("Type: FMT3_TRAP");
                            $display("  - ecause: 0x%h", pkt_union.trap_pkt.ecause);
                            $display("  - interrupt: %b", pkt_union.trap_pkt.interrupt);
                            $display("  - thaddr: %b", pkt_union.trap_pkt.thaddr);
                            $display("  - addr: 0x%h", pkt_union.trap_pkt.addr << iaddress_lsb_p);
                            $display("  - tval: 0x%h", pkt_union.trap_pkt.tval);
                        end
                        2'b10: begin // Context
                            $display("Type: FMT3_CONTEXT");
                            // Add field extraction as needed
                        end
                        2'b11: begin // Support
                            $display("Type: FMT3_SUPPORT");
                            // Add field extraction as needed
                        end
                    endcase
                end
                2'b01: begin // Format 1 (Branch)
                    $display("Type: FMT1_BRANCH");
                    $display("  - branches: %d", pkt_union.branch_pkt.branches);
                    $display("  - branch_map: 0b%b", pkt_union.branch_pkt.branch_map);
                    $display("  - addr: 0x%h", pkt_union.branch_pkt.addr << iaddress_lsb_p);
                end
                2'b10: begin // Format 2 (Address)
                    $display("Type: FMT2_ADDR");
                    $display("  - addr: 0x%h", pkt_union.raw[38:8] << iaddress_lsb_p);
                end
                default: $display("Unknown Packet Type");
            endcase
        end
    end


    // Packet checker
    /*always @(posedge clk) begin
        if (packet_valid) begin
            $display("[Packet] Data: 0x%h", packet_data);
            // Check against expected packets
            case (packet_data[1:0]) // Check format
                2'b01: begin // Format 1 (Branch)
                    if (packet_data[7:2] != 5'h1) 
                        $error("Format 1: Expected branches=1, got %d", packet_data[7:2]);
                    if (packet_data[37:8] != 30'h0) 
                        $error("Format 1: Expected branch_map=0, got 0x%h", packet_data[37:8]);
                end
                2'b11: begin // Format 3
                    case (packet_data[3:2]) // Subformat
                        2'b01: begin // Trap
                            if (packet_data[16] != 0) // thaddr=0
                                $error("Trap: Expected thaddr=0, got %d", packet_data[16]);
                            if (packet_data[24:17] != 8'h02) // ecause=2
                                $error("Trap: Expected ecause=2, got 0x%h", packet_data[24:17]);
                        end
                        2'b00: begin // Start
                            if (packet_data[39:8] != 32'h800001b0 >> iaddress_lsb_p) 
                                $error("Start: Incorrect address");
                        end
                    endcase
                end
            endcase
        end
    end*/
endmodule