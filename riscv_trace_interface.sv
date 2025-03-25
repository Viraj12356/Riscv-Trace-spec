// Code your design here
//Single instruction retirement

package control;
typedef struct packed {
    logic [31:27] reserved1; // RW, Reset 0
    logic [26:24] trTeFormat; // WARL, Reset Under
    logic [23:20] trTeInstSyncMax; // WARL, Reset Under
    logic [19:18] reserved2; // RW, Reset 0
    logic [17:16] trTeInstSyncMode; // WARL, Reset Under
    logic trTeInhibitSrc; // WARL, Reset Under
    logic reserved3; // RW, Reset 0
    logic trTeInstStallEna; // WARL, Reset Under
    logic trTeInstStallOrOverflow; // RW1C, Reset Under
    logic trTeInstTrigEnable; // WARL, Reset Under
    logic reserved4; // RW, Reset 0
    logic trTeContext; // WARL, Reset Under
    logic [8:7] reserved5; // RW, Reset 0
    logic [6:4] trTeInstMode; // WARL, Reset Under
    logic trTeEmpty; // RO, Reset 1
    logic trTeInstTracing; // RW, Reset Under
    logic trTeEnable; // RW, Reset 0
    logic trTeActive; // RW, Reset 0
} tr_te_control_t;




endpackage

module riscv_trace_interface #(
    parameter MAX_BITS = 80,
    //parameter FIFO_DEPTH = 8,
    parameter blocks_p = 1,
    parameter itype_width_p = 4,
    parameter iaddress_width_p = 32,
    parameter iretire_width_p = 2,
    parameter ilastsize_width_p = 1,
    parameter privilege_width_p = 2,
    parameter ecause_width_p = 4,
    parameter context_width_p = 1,
    parameter iaddress_lsb_p = 1,
	parameter time_width_p = 1
)(
    input logic clk,
    input logic rst_n,
    // Core Interface
    input logic [blocks_p-1:0][itype_width_p-1:0] itype,
    input logic [ecause_width_p-1:0] cause,
    input logic [iaddress_width_p-1:0] tval,
    input logic [privilege_width_p-1:0] priv,
    input logic [blocks_p-1:0][iaddress_width_p-1:0] iaddr,
    input logic [context_width_p-1:0] context_,
    input logic [blocks_p-1:0][iretire_width_p-1:0] iretire,
    input logic [blocks_p-1:0][ilastsize_width_p-1:0] ilastsize,
    // Control
    input control::tr_te_control_t ctrl,
	
	//Packet outputs
	output logic [MAX_BITS-1:0] bit_array
);

// ================== Packet Structures ==================
typedef struct packed {
    logic [1:0] fmt; // 11
    logic [1:0] sfmt; // 00
    logic branch; 
    logic [privilege_width_p-1:0] priv;
    logic [context_width_p-1:0] context_;
	logic [time_width_p-1:0] time_;
    logic [(iaddress_width_p-iaddress_lsb_p)-1:0] addr;
} fmt3_sync_pkt_t;

typedef struct packed {
    logic [1:0] fmt; // 11
    logic [1:0] sfmt; // 01
    logic branch;
    logic [privilege_width_p-1:0] priv;
    logic [ecause_width_p-1:0] ecause;
    logic interrupt;
    logic thaddr;
    logic [(iaddress_width_p-iaddress_lsb_p)-1:0] addr;
    logic [iaddress_width_p-1:0] tval;
	logic [context_width_p-1:0] context_;
	logic [time_width_p-1:0] time_;
} fmt3_trap_pkt_t;

typedef struct packed {
    logic [1:0] fmt; // 11
    logic [1:0] sfmt; // 10
    logic [privilege_width_p-1:0] priv;
    logic [context_width_p-1:0] context_;
	logic [time_width_p-1:0] time_;
} fmt3_context_pkt_t;

typedef struct packed {
    logic [1:0] fmt;    // 11
    logic [1:0] sfmt;   // 11 (SUPPORT)
    logic ienable;
    logic encoder_mode;
    logic [1:0] qual_status;
    logic [4:0] ioptions;
    logic denable;      // Fake data trace fields
    logic dloss;
    logic [3:0] doptions;
} fmt3_support_pkt_t;

typedef struct packed {
    logic [1:0] fmt; // 01
    logic [4:0] branches;
    logic [30:0] branch_map;
    logic [iaddress_width_p-1:0] addr;
    logic notify;
    logic updiscon;
    logic irreport;
    logic [3:0] irdepth;
} fmt1_branch_pkt_t;

typedef struct packed {
    logic [1:0] fmt; // 10
    logic [iaddress_width_p-1:0] addr;
    logic notify;
    logic updiscon;
    logic irreport;
    logic [3:0] irdepth;
} fmt2_addr_pkt_t;


// ================== Pipeline Stages ==================
typedef struct packed {
    logic [itype_width_p-1:0] itype;
    logic [iaddress_width_p-1:0] iaddr;
    logic [iretire_width_p-1:0] iretire;
    logic [ilastsize_width_p-1:0] ilastsize;
    logic [privilege_width_p-1:0] priv;
    logic [context_width_p-1:0] context_;
    logic [ecause_width_p-1:0] cause;
    logic [iaddress_width_p-1:0] tval;
    logic [1:0] ctype;
} pipeline_stage_t;

pipeline_stage_t prev_stage, curr_stage, next_stage;


// ================== Algorithm State ==================
logic [4:0] branch_count;
logic [30:0] branch_map;
logic [iaddress_width_p-1:0] last_reported_addr;
logic [3:0] call_counter;
logic [4:0] sync_counter;
logic first_packet_after_enable;
logic [1:0] resync_reason;

// ================== FIFO Implementation ==================
/*localparam FIFO_ADDR_WIDTH = $clog2(FIFO_DEPTH);
logic [FIFO_ADDR_WIDTH:0] fifo_wptr, fifo_rptr;
logic [PACKET_WIDTH-1:0] packet_fifo [0:FIFO_DEPTH-1];*/

// ================== Condition Logic ==================
logic addr_config,exc_only, prev_reported, rpt_br, cci, ppccd, ppccd_br, exc_only_next, er_n,trap_reported, updiscon_prev,updiscon_curr;

always_comb begin : condition_logic

	//Updiscon
	updiscon_prev = (prev_stage.itype inside {6,8,10,14});
    
    updiscon_curr = (curr_stage.itype inside {6,8,10,14});

    // Exception without retirement
  exc_only = (curr_stage.iretire == 0) && (curr_stage.itype inside {1,2});

  
    // Previous exception reported with thaddr=0
  prev_reported = (prev_stage.itype inside {1,2}) && trap_reported;

    // Need to report branches
    rpt_br = (branch_count >= 5'd31);

    // Imprecise context change
    cci = (curr_stage.context_ != prev_stage.context_) &&
  (curr_stage.ctype == 2'd2);

    // Precise privilege/context change
    ppccd = (curr_stage.priv != prev_stage.priv) || 
           ((curr_stage.context_ != prev_stage.context_) && 
            ((curr_stage.ctype == 2'b10) || updiscon_prev));

    // Resync required with pending branches
  ppccd_br = ((branch_count > 0) && (next_stage.priv != curr_stage.priv)) || ((next_stage.context_ != curr_stage.context_) && (next_stage.ctype == 2'd2 || updiscon_curr));
  
  exc_only_next = (next_stage.iretire == 0) && (next_stage.itype inside {1,2});


    // Exception + retirement or notify trigger
  er_n = (curr_stage.iretire > 0) &&(curr_stage.itype inside {1,2});
          
end

// ================== Packet Buffer Management ==================
/*task enqueue_packet(input logic [PACKET_WIDTH-1:0] pkt);
    if ((fifo_wptr[FIFO_ADDR_WIDTH-1:0] != fifo_rptr[FIFO_ADDR_WIDTH-1:0]) || 
       (fifo_wptr[FIFO_ADDR_WIDTH] == fifo_rptr[FIFO_ADDR_WIDTH])) begin
        packet_fifo[fifo_wptr[FIFO_ADDR_WIDTH-1:0]] <= pkt;
        fifo_wptr <= fifo_wptr + 1;
    end else begin
        packet_error <= 1'b1;
    end
endtask*/

//-------------------------------------------------------------------------
  // bit stream and stats
  //-------------------------------------------------------------------------
  // a counter to track the current bit count.
  integer            bit_count;
  integer 			 branch_bits;
  

// ================== Packet Generation Tasks ==================
  task automatic send_fmt3_trap(logic thaddr, logic [ecause_width_p-1:0] cause, logic[iaddress_width_p-1:0] tval);
    fmt3_trap_pkt_t pkt = '0;
    pkt.fmt = 2'b11;
    pkt.sfmt = 2'b01;
    pkt.thaddr = thaddr;
    pkt.ecause = cause;
    pkt.tval = tval;
	pkt.branch = (branch_count > 0) ? branch_map[branch_count] : 1'b1;
    pkt.addr = (addr_config == 1'b1) ? curr_stage.iaddr >> iaddress_lsb_p:(curr_stage.iaddr-last_reported_addr) >> iaddress_lsb_p;
    pkt.interrupt = (curr_stage.itype == 2);
	pkt.priv = curr_stage.priv;
	pkt.context_ = curr_stage.context_;

	bit_array = '0;
    bit_count = 0;
	
    add_bits(2, pkt.fmt);
    add_bits(2, pkt.sfmt);
	add_bits(1, pkt.branch);
    add_bits(privilege_width_p, pkt.priv);
    //add_bits(time_width_p, pkt.time_);
	add_bits(iaddress_width_p, pkt.tval);
    add_bits(ecause_width_p, pkt.ecause);
	add_bits(1, 1'b0);//time
	add_bits(context_width_p , pkt.context_);
	add_bits(1, pkt.interrupt);
    add_bits(1, pkt.thaddr);
	add_bits((iaddress_width_p - iaddress_lsb_p),pkt.addr);

	output_packet();
	  
	trap_reported <= (thaddr == 0);

endtask


  task automatic send_fmt3_sync();
    fmt3_sync_pkt_t pkt='0;
    pkt.fmt = 2'b11;
    pkt.sfmt = 2'b00;
    pkt.branch = (branch_count > 0) ? branch_map[branch_count] : 1'b1;
    pkt.priv = curr_stage.priv;
    pkt.context_ = curr_stage.context_;
    pkt.addr = (addr_config == 1'b1) ? curr_stage.iaddr >> iaddress_lsb_p:(curr_stage.iaddr-last_reported_addr) >> iaddress_lsb_p;
	pkt.priv = curr_stage.priv;
	pkt.context_ = curr_stage.context_;
	//pkt.time_ = curr_stage.time_; 
    
    bit_array = '0;
    bit_count = 0;
	
    add_bits(2, pkt.fmt);
    add_bits(2, pkt.sfmt);
	add_bits(1, pkt.branch);
	add_bits(privilege_width_p, pkt.priv);
	add_bits(iaddress_width_p, 1'b0);
	add_bits((iaddress_width_p-iaddress_lsb_p), pkt.addr);
	add_bits(1, 1'b0);//time
    add_bits(context_width_p , pkt.context_);
	output_packet();

	trap_reported <= 0;
	
endtask

task automatic send_fmt3_context();
    fmt3_context_pkt_t pkt = '0;
    pkt.fmt = 2'b11;
    pkt.sfmt = 2'b10;
    pkt.priv = curr_stage.priv;
	pkt.context_ = curr_stage.context_;
	//pkt.time_ = curr_stage.time_; 
   	bit_array = '0;
    bit_count = 0;
  
    add_bits(2, pkt.fmt);
    add_bits(2, pkt.sfmt); 
	add_bits(privilege_width_p, pkt.priv);
    //add_bits(time_width_p : 0, pkt.time_);
    add_bits(context_width_p , pkt.context_);
	output_packet();

endtask


task automatic send_fmt3_support();
    fmt3_support_pkt_t pkt = '0;
    pkt.fmt = 2'b11;
    pkt.sfmt = 2'b11;
    pkt.ienable = 1'b1; // Active tracing
    pkt.ioptions = 5'b10101; // Replace with actual config bits
	pkt.qual_status = 2'b00; //No change to qualified instruction
  

	bit_array = '0;
    bit_count = 0;

    add_bits(2, pkt.fmt);
    add_bits(2, pkt.sfmt);
	add_bits(1, pkt.ienable);
    add_bits(1, pkt.encoder_mode);
    add_bits(2, pkt.qual_status);
    add_bits(5, pkt.ioptions);
	output_packet();

	
endtask

task automatic send_fmt1_branch(logic updiscon);
    fmt1_branch_pkt_t pkt ='0;
    pkt.fmt = 2'b01;
    pkt.branches = branch_count;
    pkt.branch_map = branch_map;
    pkt.addr = (addr_config == 1'b1) ? curr_stage.iaddr >> iaddress_lsb_p:(curr_stage.iaddr-last_reported_addr) >> iaddress_lsb_p;
    pkt.notify = 1'b0;
	pkt.updiscon = updiscon;
	pkt.irreport = 0;
	pkt.irdepth = 4'd0;
	
	bit_array = '0;
    bit_count = 0;
	
    // Start by adding the fields to packet.
    add_bits(2, pkt.fmt);
	add_bits(5, pkt.branches);
	branch_map_bits(pkt.branches, branch_bits);
    add_bits(branch_bits, pkt.branch_map);
	add_bits((iaddress_width_p-iaddress_lsb_p), pkt.addr);
    add_bits(1, pkt.notify);
    add_bits(1, pkt.updiscon);
    add_bits(1, pkt.irreport);
    add_bits(4, pkt.irdepth);
	output_packet();
	
	branch_count <= '0;
	branch_map <= '0;

endtask

  task automatic send_fmt2_addr(logic updiscon);
    fmt2_addr_pkt_t pkt ='0;
    pkt.fmt = 2'b10;
    pkt.addr = (addr_config == 1'b1) ? curr_stage.iaddr >> iaddress_lsb_p:(curr_stage.iaddr-last_reported_addr) >> iaddress_lsb_p;
    pkt.notify = 1'b0;
    pkt.updiscon = updiscon;
    pkt.irreport = 0;
	pkt.irdepth = 4'd0;
	
	bit_array = '0;
    bit_count = 0;
	

    add_bits_lsb(2, pkt.fmt);
	add_bits_lsb((iaddress_width_p-iaddress_lsb_p), pkt.addr);
    add_bits_lsb(1, pkt.notify);
    add_bits_lsb(1, pkt.updiscon);
    add_bits_lsb(1, pkt.irreport);
    //add_bits_lsb(1, pkt.irdepth);
		
	output_packet();

endtask

// ================== Address Calculation Functions ==================
/*function logic [iaddress_width_p-1:0] calc_last_addr(
    input logic [iaddress_width_p-1:0] base_addr,
    input logic [iretire_width_p-1:0] iretire,
    input logic [ilastsize_width_p-1:0] ilastsize
);
    return base_addr + (iretire - (1 << ilastsize)) * 2;
endfunction*/



  


// ================== Main Algorithm Implementation ==================
logic [8:0] sync_max_count;
assign sync_max_count = (1 << (ctrl.trTeInstSyncMax+4)); 	

always_ff @(posedge clk or negedge rst_n) begin : main_algorithm
    if (!rst_n) begin
        prev_stage <= '0;
        curr_stage <= '0;
        next_stage <= '0;
        last_reported_addr <= '0;
        branch_count <= '0;
        branch_map <= '0;
        call_counter <= '0;
        sync_counter <= '0;
        resync_reason <= 2'b00;
        //fifo_wptr <= '0;
		addr_config <= 1'b1;

	    //send_fmt3_support();
    end else begin
        // Pipeline shift


     prev_stage <= curr_stage;
        curr_stage <= next_stage;
        next_stage <= '{
            itype: itype,
            iaddr: iaddr,
            iretire: iretire,
            ilastsize: ilastsize,
            priv: priv,
            context_: context_,
            cause: cause,
            tval: tval,
            ctype: (context_ != curr_stage.context_) ? ((itype[0] inside {8,9,13}) ? 2'b11 : 2'b10) : 2'b00
        };


        if (ctrl.trTeActive && prev_stage != '0) begin
			if(ctrl.trTeEnable) begin
            // ===== Diamond 1: Qualified Instruction? =====
            if (1)begin//curr_stage.itype != 0) begin
                // ===== Diamond 2: Branch Instruction? =====
                if (curr_stage.itype inside {4, 5}) begin
                    branch_map[branch_count] = (curr_stage.itype == 5);
                    branch_count <= branch_count + 1;
                end

                // ===== Diamond 3: Previous Exception? =====
              if (prev_stage.itype inside {1,2}) begin
                    // Sub Diamond 3a: exc_only?
                    if (exc_only) begin
       send_fmt3_trap(0,prev_stage.cause,prev_stage.tval);
                      last_reported_addr <= curr_stage.iaddr;
					  sync_counter <= '0;
                    end
                    // Sub Diamond 3b: Reported?
                    else if (prev_reported) begin
                      	send_fmt3_sync();
						//$display("reported");
                        last_reported_addr <= curr_stage.iaddr;
                        sync_counter <= '0;
                    end else begin
					  send_fmt3_trap(1,prev_stage.cause,prev_stage.tval);
                      last_reported_addr <= curr_stage.iaddr;
					  sync_counter <= '0;
                    end
                end
              // ===== Diamond 4: ppccd or > max_sync? =====
              else if (ppccd || sync_counter > sync_max_count) begin
                    send_fmt3_sync();
                    last_reported_addr <= curr_stage.iaddr;
                	sync_counter <= '0;
                end
              // ===== Diamond 5: Updiscon previous?====
              else if(updiscon_prev) begin
                if(exc_only) begin
                  send_fmt3_trap(0,curr_stage.cause,curr_stage.tval);
				  //$display("exc_only");
                  last_reported_addr <= curr_stage.iaddr;
                  sync_counter <= '0;
                end else begin
				  if (branch_count > 0) begin
					send_fmt1_branch(updiscon_prev);
                  end else begin
				  //$display("updiscon_prev");
				  send_fmt2_addr(updiscon_prev);
                  end
				  last_reported_addr <= curr_stage.iaddr;
                end
              end
              //=== Diamond 6: resync_br or er_n?===
              else if(sync_counter == sync_max_count || branch_count > 0 || er_n) begin
                if (branch_count > 0) begin
					send_fmt1_branch(updiscon_curr);
                end else begin
					send_fmt2_addr(updiscon_curr);
					//$display("sync_max_count");
				end
                last_reported_addr <= curr_stage.iaddr;
               
              end
              //====Diamond 7: exc_only_next or ppccd_br or unqualified
              else if(exc_only_next || ppccd_br) begin
                if (branch_count > 0) begin
					send_fmt1_branch(updiscon_curr);
                end else begin
				send_fmt2_addr(updiscon_curr);
				//$display("exc_only_next");
				end
				last_reported_addr <= curr_stage.iaddr;
              end
              // ===== Diamond 8: rpt_br? =====
                else if (rpt_br) begin
                        send_fmt1_branch(updiscon_curr);
                  	    last_reported_addr <= curr_stage.iaddr;
                end
                // ===== Diamond 5: cci? =====
                else if (cci) begin
                        send_fmt3_context();						            
						last_reported_addr <= curr_stage.iaddr;
                end
                
                // ===== Box 9: Continue accumulation =====
                else begin
                    sync_counter <= sync_counter + 1;
                end
            end
        end else if (!ctrl.trTeActive) begin
			send_fmt3_support();
		end
    end
end 
end

// ================== Packet Output Logic ==================
/*always_ff @(posedge clk or negedge rst_n) begin : output_logic
    if (!rst_n) begin
        packet_valid <= 1'b0;
        packet_data <= '0;
        fifo_rptr <= '0;
    end else begin
        packet_valid <= (fifo_rptr != fifo_wptr);
        if (fifo_rptr != fifo_wptr) begin
            packet_data <= packet_fifo[fifo_rptr[FIFO_ADDR_WIDTH-1:0]];
            fifo_rptr <= fifo_rptr + 1;
        end
    end
end*/


task add_bits_lsb(input int width, input logic [31:0] value);
  begin
    //$display("%d %h", width, value);
    bit_array = (bit_array << width) | value;
    //$display("%h", bit_array);
    bit_count = bit_count + width;
  end
endtask

  //-------------------------------------------------------------------------
  // Task: add_bits
  // Appends a field (given its width and value) to the bit_array.
  // The implementation shifts left and ORs in the new field.
  //-------------------------------------------------------------------------
task add_bits(input int width, input logic [32-1:0] value);
  begin
    //$display("%d %h", width, value);
    // Prepend new bits: shift new value to the left by the current bit count
    bit_array = (value << bit_count) | bit_array;
    //$display("%h", bit_array);
    bit_count = bit_count + width;
  end
endtask

  //-------------------------------------------------------------------------
  // Task: branch_map_bits
  // Given the number of branches, returns (via output arguments) the number
  // of bits for the branch map and whether an address field is required.
  //-------------------------------------------------------------------------
  task branch_map_bits(input int branches,
                       output int branch_bits);
    begin
      if (branches == 0) begin
        branch_bits = 31;
      end else if (branches == 1) begin
        branch_bits = 1;
      end else if (branches <= 3) begin
        branch_bits = 3;
      end else if (branches <= 7) begin
        branch_bits = 7;
      end else if (branches <= 15) begin
        branch_bits = 15;
      end else if (branches < 32) begin
        branch_bits = 31;
      end else begin
        $display("Error: branches out of range: %0d", branches);
        branch_bits = 31;
      end
    end
  endtask


  task output_packet;

    //output logic [FULL_WIDTH-1:0] out_val,
    //int out_bytes;
    // Temporary vector for reversed bytes.
    logic [MAX_BITS-1:0] rev;
    int i;
    //int effective_width;
	//int trim_index;
	//int extra;
	
	begin
    // Step 1: Reverse the byte order.
    // For each byte in the input, copy it into the reversed order.
    for (i = 0; i < (MAX_BITS/8); i++) begin
      rev[i*8 +: 8] = bit_array[((MAX_BITS/8)-1 - i)*8 +: 8];
    end

	$display("Rev: %h",rev);
    // Step 2: Calculate effective width (number of significant bits)
    // without using a break statement.
   /* $display("MSB bit_array: %h",bit_array[MAX_BITS-1]);
    effective_width = 0;
    // Scan from the most-significant bit (index FULL_WIDTH-1) to 0.
    for (int j = MAX_BITS-1; j >= 0; j--) begin
      // When a 1 is found and effective_width is still zero, record j+1.
      if (bit_array[j] == 1'b1 && effective_width == 0) begin
        effective_width = (j + 1);
      end
    end
    // If no bits are set, effective width is defined as 1.
    if (effective_width == 0)
      effective_width = 1;

    // Step 3: Trim trailing zero bytes from the reversed vector.
    // (These come from the original high-order zeros.)
	
	
	
	$display("Effective Width: %h",effective_width);
   
    //trim_index = (MAX_BITS/8) - 1;
    //while (trim_index >= (MAX_BITS - effective_width) && bit_array[trim_index*8 +: 8] == 8'h00)
    //  trim_index--;
    
    // If all bytes are zero, force one byte as output.
    //if (trim_index < 0) begin
    //  out_bytes = 1;
    //  out_val = { {(MAX_BITS-8){1'b0}}, 8'h00 };
    //  return;
    //end else begin
    out_bytes = trim_index + 1;
    //end

    // Step 4: Adjust the top (most-significant) byte.
    // If the effective width is not an even multiple of 8, then the top byte
    // contains extra unused bits. Calculate the number of extra bits.
    
    if (effective_width % 8 == 0)
      extra = 0;
    else
      extra = 8 - (effective_width % 8);
	  
	$display("extra %h", extra);

    // Right-shift the top byte (last byte in rev) by the extra bits.
    rev[(effective_width+extra)*8 +: 8] = rev[(effective_width+extra)*8 +: 8] >> extra;

    // The compressed value is the lower (out_bytes*8) bits of rev.
	//for(int k=0 ; k<=out_bytes; k++)
		$display("%h",rev);*/
	end
  endtask


endmodule