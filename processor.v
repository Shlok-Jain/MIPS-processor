// Code your design here
`timescale 1ns / 1ps

module DistributedMemory (
 input wire [9:0] addr_in, input wire[9:0] read_addr , input write_enable , input clk ,
 input wire [31 : 0] data_in , output wire [31 : 0] data_out
);
    reg [31:0] mem_cells [1023:0];
    assign data_out = mem_cells[read_addr];
    always @(negedge clk) begin
        if(write_enable) begin
            mem_cells[addr_in] = data_in;
        end
        else begin
            mem_cells[addr_in] = mem_cells[addr_in];
        end
    end
endmodule

//Reads at positive edge and writes at negative edge
module RegBank (
 input wire [4:0] dest_reg, input wire [4:0] src_reg1, input wire [4:0] src_reg2, input clk, input write_enable, input wire [31:0] data_in,
 output [31:0] src1_data, output [31:0] src2_data, input wire reset
);
    reg [31:0] reg_array [31:0];
    integer index;

    // Read operation
    assign src2_data = reg_array[src_reg2];
    assign src1_data = reg_array[src_reg1];

    // Write operation
    always @(posedge clk) begin
        if(reset) begin
            for (index = 0; index < 32; index = index + 1) begin
                reg_array[index] = 0;
            end
        end
        else if(write_enable) begin
            reg_array[dest_reg] = data_in;
        end
    end
endmodule

module InstructionParser (
 input wire [31:0] instr, output wire [4:0] field1 , output wire [4:0] field2 , output wire [4:0] field3 ,
 output wire [5:0] opcode , output wire [4:0] shift_amt , output wire [5:0] op_type , output wire [15:0] imm_value,
 output wire [25:0] jump_target
);
    assign op_type = instr[31:26];
    assign field3 = instr[25:21];
    assign field2 = instr[20:16];
    assign field1 = instr[15:11];
    assign shift_amt = instr[10:6];
    assign op_field = instr[5:0];
    assign imm_value = instr[15:0];
    assign jump_target = instr[25:0];
endmodule
/*
op_select = 0 => ADD
1 == Sub (val1 - val2) == src1 - src2
2 == and
3 == xor
4 == or
5 == not
6 == left shift
7 == right shift
8 == set not equal
9 == set equal
10 == set less than
11 == set less than equal
12 == set greater than
13 == set greater than equal
*/
module ArithLogicUnit (
 input wire [31:0] val1, input wire [31:0] val2 , input wire [4:0] op_select , input wire clk ,
 output reg [31:0] result , input wire [4:0] shift_amount
);
    wire[63:0] mult_result;
    assign mult_result = $signed(val1) * $signed(val2);
    always @(val1 or val2 or op_select or shift_amount or mult_result) begin
        case (op_select)
            0: result = val1 + val2;
            1: result = val1 - val2;
            2: result = val1 & val2;
            3: result = val1 ^ val2;
            4: result = val1 | val2;
            5: result = ~val1;
            6: result = val1 << shift_amount;
            7: result = val1 >> shift_amount;
            8: result = ($signed(val1) != $signed(val2));
            9: result = ($signed(val1) == $signed(val2));
            10: result = ($signed(val1) < $signed(val2));
            11: result = ($signed(val1) <= $signed(val2));
            12: result = ($signed(val1) > $signed(val2));
            13: result = ($signed(val1) >= $signed(val2));
            14: result = val2 << 16;
            15: result = mult_result[31:0];
            16: result = mult_result[63:32];
            default: result = 32'b0;
        endcase
    end
endmodule

module FloatToIntConverter (
    input [31:0] float_val , output reg [31:0] int_out
);
    reg sign_bit;
    reg [32:0] mantissa_bits;
    reg [7 : 0] exp_bits;
    reg [7:0] offset;
    integer iter;
    always @(float_val) begin
        {sign_bit, exp_bits, mantissa_bits[31:9]} = float_val;
        mantissa_bits[32] = 1;
        mantissa_bits[8:0] = 0;
        int_out = 0;
        offset = exp_bits - 127;
        if (offset > 30) begin
            int_out = 0;
        end
        else if(offset < 0) begin
            int_out = 0;
        end
        else begin
            for(iter = offset ; iter > -1 ; iter = iter - 1) begin
                int_out[iter] = mantissa_bits[32 + (iter - (offset))];
            end
        end
    end
endmodule

module IntToFloatConverter (
    input [31:0] int_val , output reg[31:0] float_out
);
    reg[22:0] mantissa_val;
    reg sign_val;
    reg [7:0] exp_val;
    reg[4:0] idx;
    integer pos;
    always @(int_val) begin
        sign_val = 0;
        exp_val = 31;
        if (int_val == 0) begin
            float_out = 32'h00000000;
        end
        else begin
          for (idx = 31; int_val[idx] == 0; idx = idx - 1) begin
            exp_val = idx;
          end

            exp_val = exp_val - 1;

            mantissa_val = 0;
            if (idx >= 23) begin
                for (pos = 0; pos < 23; pos = pos + 1) begin
                    mantissa_val[22 - pos] = int_val[idx - 1 - pos];
                end
            end 
            else begin
                for(pos = exp_val - 1 ; pos >= 0 ; pos = pos - 1) begin
                    mantissa_val[22 + (pos - (exp_val - 1))] = int_val[pos];
                end
            end
            exp_val = exp_val + 127;
            float_out = {sign_val , exp_val, mantissa_val};
        
        end
    end
endmodule

module FloatingPointAdder (
    input wire [31:0] operand1, input [31:0] operand2, output reg[31:0] result
);
    reg sign_a, sign_b;
    reg [7:0] exp_a, exp_b;
    reg [7:0] diff_val;
    reg [23:0] mant_a_temp, mant_b_temp;
    reg[24:0] sum_val;
    reg[23:0] mant_a, mant_b;
    integer pos;
    
    always @(operand1 or operand2) begin
        sign_a = operand1[31]; sign_b = operand2[31];
        exp_a = operand1[30:23]; exp_b = operand2[30:23];
        mant_a = {1'b1, operand1[22:0]}; mant_b = {1'b1, operand2[22:0]};
        
        if(sign_a == sign_b) begin
            if(exp_a > exp_b) begin
                diff_val = exp_a - exp_b;
                mant_b_temp = {1'b1, operand2[22:0]} >> diff_val;
                sum_val = {1'b0, mant_a} + {1'b0, mant_b_temp};
                
                if(sum_val[24] == 1) begin
                    sum_val = sum_val >> 1;
                    exp_a = exp_a + 1;
                end
                result = {sign_a, exp_a, sum_val[22:0]};
            end
            else begin
                diff_val = exp_b - exp_a;
                mant_a_temp = {1'b1, operand1[22:0]} >> diff_val;
                sum_val = {1'b0, mant_a_temp} + {1'b0, mant_b};
                
                if(sum_val[24] == 1) begin
                    sum_val = sum_val >> 1;
                    exp_b = exp_b + 1;
                end
                result = {sign_b, exp_b, sum_val[22:0]};
            end
        end
        else begin
            if (exp_a == exp_b && mant_a == mant_b) begin
                result = 32'b0;
            end
            else begin
            // Different signs calculation
            if(operand1[30:0] > operand2[30:0]) begin
                diff_val = exp_a - exp_b;
                mant_b_temp = mant_b >> diff_val;
                mant_a = mant_a - mant_b_temp;
                
                // Normalize
                for (pos = 22; pos >= 0; pos = pos - 1) begin
                    if (mant_a[pos] == 1) begin
                        exp_a = exp_a - (23 - pos);
                        mant_a = mant_a << (23 - pos);
                        pos = -1;
                    end
                end
                
                result = {sign_a, exp_a, mant_a[22:0]};
            end 
            else begin
                diff_val = exp_b - exp_a;
                mant_b_temp = mant_a >> diff_val;
                mant_a = mant_b - mant_b_temp;
                exp_a = exp_b;
                
                // Normalize
                for (pos = 22; pos >= 0; pos = pos - 1) begin
                    if (mant_a[pos] == 1) begin
                        exp_a = exp_a - (23 - pos);
                        mant_a = mant_a << (23 - pos);
                        pos = -1;
                    end
                end
                
                result = {sign_b, exp_a, mant_a[22:0]};
            end
            end
        end
    end
endmodule

module SignFlipper (
    input flip_enable , input [31:0] input_val , output [31:0] output_val
);
    assign output_val = (flip_enable)? input_val ^ 32'h80000000 : input_val;
endmodule

//Put src_reg2 into val1
module FloatingPointUnit (
    input wire [31:0] val1, input wire [31:0] val2, output reg [31:0] fp_result,
    input wire [5:0] op_type, output reg condition_flag
);
    wire [31:0] adder_result;
    wire flip_enable;
    assign flip_enable = (op_type == 6'b100011) || (op_type == 6'b100100) || 
                   (op_type == 6'b100101) || 
                   (op_type == 6'b100110) || 
                   (op_type == 6'b100111) || 
                   (op_type == 6'b101000);
                   
    wire [31:0] mod_val2;
    SignFlipper sign_modifier(.flip_enable(flip_enable), .input_val(val2), .output_val(mod_val2));
    FloatingPointAdder fp_adder(.operand1(val1), .operand2(mod_val2), .result(adder_result));
    
    wire is_zero_result;
    assign is_zero_result = (adder_result[30:0] == 31'b0);

    always @(val1 or val2 or op_type or adder_result or is_zero_result) begin
        case (op_type)
            6'b100010: fp_result = adder_result; // add.s
            6'b100011: fp_result = adder_result; // sub.s
            
            // Comparison operations
            6'b100100: condition_flag = is_zero_result;
            6'b100101: condition_flag = adder_result[31] || is_zero_result;
            6'b100110: condition_flag = adder_result[31] && !is_zero_result;
            6'b100111: condition_flag = !adder_result[31] || is_zero_result;
            6'b101000: condition_flag = !adder_result[31] && !is_zero_result;
            
            6'b101001: fp_result = val1; // mov.s.cc
            default: fp_result = fp_result;
        endcase
    end
endmodule

module ProgramCounter (
 input clk , output reg [9:0] counter_val, input reset , input jump_enable , input [25:0] jump_addr , input branch_enable , input [15:0] branch_offset
);
    always @(posedge clk ) begin
        if(reset) begin
            counter_val = 0;
        end
        else begin
            if(jump_enable) begin
                counter_val = jump_addr[9:0];
            end
            else if(branch_enable) begin
                counter_val = counter_val + 1 + branch_offset;
            end
            else begin
                counter_val = counter_val + 1;
            end
        end
    end
endmodule

module OperationSelector (
 input wire [5:0] op_type , input wire[5:0] opcode , output reg [4:0] op_select
);
    always @(op_type or opcode) begin
        if(op_type == 6'b000111 || op_type == 6'b001000) begin // lw or sw
            op_select = 0;
        end
        else if(op_type == 1) begin // addi
            op_select = 0;
        end
        else if(op_type == 2) begin // andi
            op_select = 2;
        end
        else if(op_type == 3) begin // ori
            op_select = 4;
        end
        else if(op_type == 4) begin // xori
            op_select = 3;
        end
        else if(op_type == 5) begin // addui
            op_select = 0;
        end
        else if(op_type == 6'b001001 || op_type == 6'b010100 || op_type == 6'b010110) begin // slti and ble , bleu
            op_select = 10;
        end
        else if(op_type == 6'b001010 || op_type == 6'b010000) begin // seq, beq
            op_select = 9;
        end
        else if(op_type == 6'b010001) begin // bne
            op_select = 8;
        end
        else if(op_type == 6'b010010 || op_type == 6'b010111) begin // bgt ,bgtu
            op_select = 12;
        end
        else if(op_type == 6'b010011) begin // bgte
            op_select = 13;
        end
        else if(op_type == 6'b010101) begin // bleq
            op_select = 11;
        end
        else if(op_type == 6'b001011) begin//lui
            op_select = 14;
        end
        else begin
            if(op_type == 6'b000000) begin
                case (opcode)
                    0: op_select = 0;
                    1: op_select = 1;
                    2: op_select = 2;
                    3: op_select = 4;
                    4: op_select = 5;
                    5: op_select = 3;
                    6: op_select = 0;
                    7: op_select = 1;
                    8: op_select = 10;
                    9: op_select = 6;
                    10: op_select = 7;
                    11: op_select = 16;
                    12: op_select = 15;
                    default: op_select = 31;
                endcase
            end
            else begin
                op_select = 5'b11111;
            end
        end
    end
endmodule

module ImmediateExtender (
 input wire[15:0] imm_in , output wire [31:0] extended_out
);
    assign extended_out = {{16{imm_in[15]}}, imm_in};
endmodule

module Selector #(
 parameter width = 32
) (
 input wire [width - 1 : 0] in0 , input wire [width - 1 : 0] in1, input sel , output wire[width - 1 : 0] out_val
);
 assign out_val = (sel)? in0 : in1;
endmodule

module ControlUnit (
 input clk , input wire [5:0] op_type , output reg reg_write , output reg mem_write ,
 output reg imm_sel , output reg jump_en, output reg branch_en , output reg link_en , output reg select_src,
 input reset , output reg move_fp_to_cpu
);
  always @(op_type or reset) begin
    if(reset) begin
      imm_sel = 0;
      jump_en = 0;
      branch_en = 0;
      select_src = 0;
      mem_write = 0;
      reg_write = 0;
      link_en = 0;
      move_fp_to_cpu = 0;
    end
    else if(op_type == 0) begin // Rtype instruction
      mem_write = 0;
      reg_write = 1;
      imm_sel = 0;
      select_src = 0;
      branch_en = 0;
      jump_en = 0;
      link_en = 0;
      move_fp_to_cpu = 0;
    end
    else if(op_type == 1 ||op_type == 2 ||op_type == 3 ||op_type == 4 ||op_type == 5) begin //addi, andi , xori, ori , addiu
      mem_write = 0;
      reg_write = 1;
      imm_sel = 1;
      select_src = 0;
      branch_en = 0;
      jump_en = 0;
      link_en = 0;
      move_fp_to_cpu = 0;
    end
    else if(op_type == 7) begin // lw
      reg_write = 1;
      mem_write = 0;
      imm_sel = 1;
      select_src = 1;
      branch_en = 0;
      jump_en = 0;
      link_en = 0;
      move_fp_to_cpu = 0;
    end
    else if(op_type == 8) begin // sw
      reg_write = 0;
      mem_write = 1;
      imm_sel = 1;
      select_src = 0;
      branch_en = 0;
      jump_en = 0;
      link_en = 0;
      move_fp_to_cpu = 0;
    end
    else if(op_type == 9 || op_type == 10) begin // slti , seq
      reg_write = 1;
      mem_write = 0;
      imm_sel = 1;
      select_src = 0;
      branch_en = 0;
      jump_en = 0;
      link_en = 0;
      move_fp_to_cpu = 0;
    end
    else if(op_type == 6'b001011) begin //lui
      reg_write = 1;
      mem_write = 0;
      imm_sel = 1;
      select_src = 0;
      branch_en = 0;
      jump_en = 0;
      link_en = 0;
      move_fp_to_cpu = 0;
    end
    else if(op_type == 16 || op_type == 17 ||op_type == 18 || op_type == 19 || op_type == 20 || op_type == 21 || op_type == 22 || op_type == 23) begin //all branches
      reg_write = 0;
      mem_write = 0;
      imm_sel = 0;
      select_src = 0;
      branch_en = 1;
      jump_en = 0;
      link_en = 0;
      move_fp_to_cpu = 0;
    end
    else if(op_type == 6'b011000 || op_type == 6'b011001) begin // All jump except jal
      reg_write = 0;
      mem_write = 0;
      imm_sel = 0;
      select_src = 0;
      branch_en = 0;
      jump_en = 1;
      link_en = 0;
      move_fp_to_cpu = 0;
    end
    else if(op_type == 6'b011010) begin // jal handled separately
      reg_write = 1;
      mem_write = 0;
      imm_sel = 0;
      select_src = 0;
      branch_en = 0;
      jump_en = 1;
      link_en = 1;
      move_fp_to_cpu = 0;
    end
    else if(op_type == 6'b100000) begin//mfc1
      reg_write = 1;
      mem_write = 0;
      imm_sel = 0;
      select_src = 0;
      branch_en = 0;
      jump_en = 0;
      link_en = 0;
      move_fp_to_cpu = 1;
    end
    else begin
      reg_write = 0;
      mem_write = 0;
      imm_sel = 0;
      select_src = 0;
      branch_en = 0;
      jump_en = 0;
      link_en = 0;
      move_fp_to_cpu = 0;
    end
  end
endmodule

module FPRegController (
    input [5:0] op_type, output fp_reg_write , output move_cpu_to_fp
);
  assign fp_reg_write = (op_type == 6'b100001 || op_type == 6'b100010 || op_type == 6'b100011 || op_type == 6'b101001);
  assign move_cpu_to_fp = (op_type == 6'b100001);
endmodule

module ProcessorCore (
 input reset , input clk , input wire [31:0] instr_data ,
 input wire [9:0] addr_in , input wire write_instr , input wire write_data_en,
 output wire [31:0] src1_output
);
 wire [9:0] pc_val;
 wire [31:0] current_instr;
 wire [31:0] alu_result;
 wire [9:0] mem_addr;
 wire [31:0] src2_data , src1_data , mem_write_data , mem_read_data , alu_src2_input, extended_imm , fp_src2_data, fp_src1_data , fp_result , int_converted , float_converted , reg_data_in , fp_reg_data_in;
 wire [4:0] field1 , field2 , field3 , shift_amt;
 wire [5:0] op_type , opcode;
 wire [15:0] imm_value;
 wire [25:0] jump_target;
 wire branch_en , jump_en , link_en , mem_write_en , imm_sel, select_src, reg_write_en, fp_reg_write_en , move_cpu_to_fp , move_fp_to_cpu;
 wire [4:0] src1_addr;
 wire cond_flag;
 wire [4:0] op_select;
 assign src1_output = src1_data;
 assign src1_addr = (branch_en | mem_write_en)? field3 : field2;
 
 wire[4:0] dest_reg_addr;
 assign dest_reg_addr = (link_en)? 5'd3 : field3;
 wire [31:0] reg_write_data , pre_move_data;
 assign pre_move_data = (link_en)? {22'b0 , pc_val} : reg_write_data;
 assign reg_data_in = (move_fp_to_cpu)? int_converted : pre_move_data;
 assign alu_src2_input = (imm_sel)? extended_imm : src2_data;
 ImmediateExtender imm_extender(.imm_in(imm_value) , .extended_out(extended_imm));
 assign reg_write_data = (select_src)? mem_read_data : alu_result;
 DistributedMemory instr_memory(.addr_in(addr_in), .data_in(instr_data), .read_addr(pc_val), .clk(clk), .write_enable(write_instr), .data_out(current_instr));

 assign mem_addr = (write_data_en)? addr_in : alu_result[9:0];
 assign mem_write_data = (write_data_en)? instr_data : src2_data;
 DistributedMemory data_memory(.addr_in(mem_addr) , .data_in(mem_write_data) , .read_addr(alu_result[9:0]) , .clk(clk) , .write_enable(mem_write_en | write_data_en) , .data_out(mem_read_data));

 InstructionParser decoder(.instr(current_instr) , .field1(field1) , .field2(field2) , .field3(field3) , .shift_amt(shift_amt), .opcode(opcode), .op_type(op_type) , .imm_value(imm_value) , .jump_target(jump_target));
 ControlUnit controller(.clk(clk) , .op_type(op_type) , .reg_write(reg_write_en) , .mem_write(mem_write_en) ,
 .imm_sel(imm_sel) , .jump_en(jump_en) , .branch_en(branch_en) , .link_en(link_en) , .select_src(select_src) ,
 .reset(reset) , .move_fp_to_cpu(move_fp_to_cpu));
 RegBank register_bank(.dest_reg(dest_reg_addr) , .src_reg1(src1_addr) , .src_reg2(field1) , .clk(clk) , .write_enable(reg_write_en) , .data_in(reg_data_in) , .reset(reset) , .src2_data(src2_data) , .src1_data(src1_data));

 OperationSelector op_selector(.op_type(op_type) , .opcode(opcode) , .op_select(op_select));
 ArithLogicUnit alu_unit(.val1(src1_data) , .val2(alu_src2_input) , .op_select(op_select) , .clk(clk) , .result(alu_result) , .shift_amount(shift_amt));
 ProgramCounter pc_unit(.clk(clk) , .counter_val(pc_val) , .reset(reset) , .jump_enable(jump_en) , .jump_addr(jump_target) , .branch_enable(branch_en & alu_result[0]) , .branch_offset(imm_value));

    // FP registers section
    assign fp_reg_data_in = (move_cpu_to_fp)? src2_data : fp_result;
    FloatToIntConverter converter1(.float_val(fp_src2_data) , .int_out(int_converted));
    IntToFloatConverter converter2(.int_val(src2_data) , .float_out(float_converted));

    RegBank fp_register_bank(.dest_reg(field3) , .src_reg1(field2) , .src_reg2(field1) , .clk(clk) , .write_enable(fp_reg_write_en) , .data_in(fp_reg_data_in) , .src1_data(fp_src1_data) , .src2_data(fp_src2_data) , .reset(reset));

    FPRegController fp_controller(.op_type(op_type) , .fp_reg_write(fp_reg_write_en) , .move_cpu_to_fp(move_cpu_to_fp));

    FloatingPointUnit fpu(.val1(fp_src1_data), .val2(fp_src2_data), .fp_result(fp_result), 
                       .op_type(op_type), .condition_flag(cond_flag));
endmodule

module ProcessorTestbench;
    reg clk, reset;
    reg [31:0] instr_data;
    reg [9:0] addr_in;
    reg write_instr, write_data_en;
    wire [31:0] src1_output;
    
    // Instantiate processor
    ProcessorCore uut (
        .reset(reset),
        .clk(clk),
        .instr_data(instr_data),
        .addr_in(addr_in),
        .write_instr(write_instr),
        .write_data_en(write_data_en),
        .src1_output(src1_output)
    );
    
    // clk Generation
    always #10 clk = ~clk;

    initial begin
        // Insertion sort implementation for array of 10 elements
        reset = 0;
      
        uut.data_memory.mem_cells[0] = 31;
        uut.data_memory.mem_cells[1] = 34;
        uut.data_memory.mem_cells[2] = 62;
        uut.data_memory.mem_cells[3] = 12;
        uut.data_memory.mem_cells[4] = 0;
        uut.data_memory.mem_cells[5] = 7;
        uut.data_memory.mem_cells[6] = 2;
        uut.data_memory.mem_cells[7] = 17;
        uut.data_memory.mem_cells[8] = 89;
        uut.data_memory.mem_cells[9] = 19;

        clk = 0;
        reset = 1;
        addr_in = 0;
        write_instr = 0;
        write_data_en = 0;
        instr_data = 32'b0;

        #10;
        // Write the addi instruction to instruction memory at address 0
        write_instr = 1;

        // $1 ---------> n
        // $2 ---------> i
        // $3 ---------> a[i]
        // $4 ---------> j
        // $5 ---------> a[j]
        // $6 ---------> key

        $monitor("PC: %d, branch_en: %b, alu_result: %d, i: %d, j: %d", uut.pc_unit.counter_val, uut.branch_en, uut.alu_result, uut.register_bank.reg_array[2], uut.register_bank.reg_array[4]);

        addr_in = 0;
        // addi $1, $0, 10  ---> initialize n = 10
        instr_data = 32'b000001_00001_00000_0000000000001010;
        #20;

        addr_in = 1;
        // addi $2, $0, 1  ---> initialize i = 1
        instr_data = 32'b000001_00010_00000_0000000000000001;
        #20;

        // outer loop
        addr_in = 2;
        // bgte $2, $1, 13  ---> if i > n, jump to end
        instr_data = 32'b010011_00010_00000_00001_00000001101;
        #20;

        addr_in = 3;
        // lw $3, 0($2)  ---> $3 = a[i]
        instr_data = 32'b000111_00011_00010_00000_00000000000;
        #20;

        addr_in = 4;
        // addi $6, $3, 0  ---> key = a[i]
        instr_data = 32'b000001_00110_00011_0000000000000000;
        #20;

        addr_in = 5;
        // addi $4, $2, -1  ---> j = i - 1
        instr_data = 32'b000001_00100_00010_1111111111111111;
        #20;

        // inner loop
        addr_in = 6;
        // ble $4, $0, 5  ---> if j < 0, jump to insert
        instr_data = 32'b010100_00100_00000_00000_00000000101;
        #20;

        addr_in = 7;
        // lw $5, 0($4)  ---> $5 = a[j]
        instr_data = 32'b000111_00101_00100_00000_00000000000;
        #20;

        addr_in = 8;
        // bgte $6, $5, 3  ---> if key >= a[j], jump to insert
        instr_data = 32'b010011_00110_00000_00101_00000000011;
        #20;

        addr_in = 9;
        // sw $5, 1($4)  ---> a[j+1] = a[j]
        instr_data = 32'b001000_00100_00000_00101_00000000001;
        #20;

        addr_in = 10;
        // addi $4, $4, -1  ---> j = j - 1
        instr_data = 32'b000001_00100_00100_1111111111111111;
        #20;

        addr_in = 11;
        // j 6 ---> jump to inner loop
        instr_data = 32'b011000_00000000000000000000000110;
        #20;

        // insert block
        addr_in = 12;
        // addi $4, $4, 1  ---> j = j + 1
        instr_data = 32'b000001_00100_00100_0000000000000001;
        #20;

        addr_in = 13;
        // sw $6, 0($4)  ---> a[j+1] = key
        instr_data = 32'b001000_00100_00000_00110_00000000000;
        #20;

        addr_in = 14;
        // addi $2, $2, 1  ---> i = i + 1
        instr_data = 32'b000001_00010_00010_0000000000000001;
        #20;

        addr_in = 15;
        // j 2  ---> jump to outer loop
        instr_data = 32'b011000_00000000000000000000000010;
        #20;

        // end
        addr_in = 16;
        // addi $1, $0, 0  ---> n = 0
        instr_data = 32'b000001_00001_00000_0000000000000000;
        #20;

        #52;
        reset = 0;
        write_instr = 0;
        #6000;

        // Check output
        $display("Register $1 value = %d", $signed(uut.register_bank.reg_array[1]));
        $display("Register $2 value = %d", $signed(uut.register_bank.reg_array[2]));
        $display("Register $3 value = %d", $signed(uut.register_bank.reg_array[3]));
        $display("Register $4 value = %d", $signed(uut.register_bank.reg_array[4]));
        $display("Register $5 value = %d", $signed(uut.register_bank.reg_array[5]));
        $display("Register $6 value = %d", $signed(uut.register_bank.reg_array[6]));
        $display("Register $7 value = %d", $signed(uut.register_bank.reg_array[7]));
        $display("Register $8 value = %d", $signed(uut.register_bank.reg_array[8]));
        $display("Register $9 value = %d", $signed(uut.register_bank.reg_array[9]));

        // Display array
        $display("Array values:");
        for (integer i = 0; i < 10; i = i + 1) begin
            $display("a[%0d] = %d", i, uut.data_memory.mem_cells[i]);
        end

        $finish;
    end
endmodule
