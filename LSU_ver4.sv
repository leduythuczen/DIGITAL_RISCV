module LSU(
    input logic i_clk,                // Global clock, active on the rising edge
    input logic i_rst,                // Global reset
    input logic [31:0] i_lsu_addr,    // Address for data read/write
    input logic [31:0] i_st_data,     // Data to be stored
    input logic i_lsu_wren,           // Write enable signal (1 if writing)
    input logic [2:0] mem_op_i,       // Memory operation (byte, half-word, word)

    // I/O connections
    input logic [31:0] i_io_sw,       // Input for switches
    input logic [3:0] i_io_btn,       // Input for buttons
    
    output logic [31:0] o_ld_data,    // Data read from memory
    output logic [31:0] o_io_ledr,    // Output for red LEDs
    output logic [31:0] o_io_ledg,    // Output for green LEDs
    output logic [31:0] o_io_lcd,     // Output for the LCD register
    output logic [31:0] io_hex0_o,    // Output for 7-segment display 0
    output logic [31:0] io_hex1_o,    // Output for 7-segment display 1
    output logic [31:0] io_hex2_o,    // Output for 7-segment display 2
    output logic [31:0] io_hex3_o,    // Output for 7-segment display 3
    output logic [31:0] io_hex4_o,    // Output for 7-segment display 4
    output logic [31:0] io_hex5_o,    // Output for 7-segment display 5
    output logic [31:0] io_hex6_o,    // Output for 7-segment display 6
    output logic [31:0] io_hex7_o,    // Output for 7-segment display 7
//    output logic o_insn_vld           // Instruction valid flag
);

// Internal memory and I/O registers
logic [31:0] data_mem [2047:0];        // 8KiB Data Memory
logic [31:0] instruction_mem [2047:0]; // 8KiB Instruction Memory
logic [31:0] output_peripherals[63:0]; // Output peripheral space (0x7000 onwards)
logic [31:0] input_peripherals;        // Input peripherals

// Address decoding signals
logic is_instr_mem, is_data_mem, is_output_peri, is_input_peri;

// Address range checks
assign is_instr_mem = (i_lsu_addr >= 32'h0000) && (i_lsu_addr <= 32'h1FFF); // 0x0000 - 0x1FFF
assign is_data_mem  = (i_lsu_addr >= 32'h2000) && (i_lsu_addr <= 32'h3FFF);
assign is_output_peri = (i_lsu_addr >= 32'h7000) && (i_lsu_addr <= 32'h703F);
assign is_input_peri  = (i_lsu_addr >= 32'h7800) && (i_lsu_addr <= 32'h781F);

// Mask for byte, half-word, word access
logic [3:0] mask;
always_comb begin
    case (mem_op_i[1:0])
        2'b00: mask = 4'b0001 << i_lsu_addr[1:0]; // Byte
        2'b01: mask = (i_lsu_addr[1] == 1'b0) ? 4'b0011 : 4'b1100; // Half-word
        2'b10: mask = 4'b1111; // Word
        default: mask = 4'b0000;
    endcase
end

// Memory read logic (combinational)
always_comb begin
    if (is_instr_mem)
        o_ld_data = instruction_mem[i_lsu_addr[12:2]]; // Read from instruction memory
    else if (is_data_mem)
        o_ld_data = data_mem[i_lsu_addr[12:2]];       // Read from data memory
    else if (is_output_peri)
        o_ld_data = output_peripherals[i_lsu_addr[7:2]]; // Read from output peripherals
    else if (is_input_peri) begin
        // Map input peripherals (switches and buttons)
        case (i_lsu_addr[3:2])
            2'b00: o_ld_data = i_io_sw;             // Switches at 0x7800
            2'b01: o_ld_data = {28'b0, i_io_btn};   // Buttons at 0x7810
            default: o_ld_data = 32'b0;
        endcase
    end else
        o_ld_data = 32'b0;
end

// Memory write logic (sequential)
always_ff @(posedge i_clk or posedge i_rst) begin
    if (i_rst) begin
        output_peripherals <= '{default: 32'b0};
 //       o_insn_vld <= 1'b0;
    end else if (i_lsu_wren) begin
        if (is_data_mem) begin
            // Write to data memory
            for (int i = 0; i < 4; i++) begin
                if (mask[i]) data_mem[i_lsu_addr[12:2]][8*i +: 8] <= i_st_data[8*i +: 8];
            end
        end else if (is_output_peri) begin
            // Write to output peripherals
            for (int i = 0; i < 4; i++) begin
                if (mask[i])
		 output_peripherals[i_lsu_addr[7:2]][8*i +: 8] <= i_st_data[8*i +: 8];
            end
        end
    end
end

// Output peripheral assignments
assign o_io_ledr  = output_peripherals[5'h00]; // Red LEDs at 0x7000
assign o_io_ledg  = output_peripherals[5'h01]; // Green LEDs at 0x7010
assign o_io_lcd   = output_peripherals[5'h02]; // LCD control at 0x7030

// Assign each 7-segment display to its respective output
assign io_hex0_o = output_peripherals[5'h03]; // 7-segment display 0 at 0x7020
assign io_hex1_o = output_peripherals[5'h04]; // 7-segment display 1 at 0x7024
assign io_hex2_o = output_peripherals[5'h05]; // 7-segment display 2 at 0x7028
assign io_hex3_o = output_peripherals[5'h06]; // 7-segment display 3 at 0x702C
assign io_hex4_o = output_peripherals[5'h07]; // 7-segment display 4 at 0x7030
assign io_hex5_o = output_peripherals[5'h08]; // 7-segment display 5 at 0x7034
assign io_hex6_o = output_peripherals[5'h09]; // 7-segment display 6 at 0x7038
assign io_hex7_o = output_peripherals[5'h0A]; // 7-segment display 7 at 0x703C


endmodule
