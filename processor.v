// Code your design here
`timescale 1ns / 1ps

module DistributedMemory (
 input wire [9:0] a, input wire[9:0] dpra , input we , input clk ,
 input wire [31 : 0] d , output wire [31 : 0] dpo
);
    reg [31:0] Address_locations [1023:0];
    assign dpo = Address_locations[dpra];
    always @(negedge clk) begin
        if(we) begin
            Address_locations[a] = d;
        end
        else begin
            Address_locations[a] = Address_locations[a];
        end
    end
endmodule

//Reads at positive edge and writes at negative edge
module RegisterFile (
 input wire [4:0] rd, input wire [4:0] rs, input wire [4:0] rt, input clk, input we, input wire [31:0] write_data,
 output [31:0] rs_out, output [31:0] rt_out, input wire rst
);
    reg [31:0] Registers [31:0];
    integer i;

    // Read on negedge - ensures we read updated values
    // always @(posedge clk) begin
    assign rt_out = Registers[rt];
    assign rs_out = Registers[rs];
    // end

    // Write on posedge
    always @(posedge clk) begin
        if(rst) begin
            for (i = 0; i < 32; i = i + 1) begin
                Registers[i] = 0;
            end
        end
        else if(we) begin
            Registers[rd] = write_data;
        end
    end
endmodule

module Splitter (
 input wire [31:0] instruction, output wire [4:0] rt , output wire [4:0] rs , output wire [4:0] rd ,
 output wire [5:0] func , output wire [4:0] shamt , output wire [5:0] opcode , output wire [15:0] address_constant,
 output wire [25:0] jaddress
);
    assign opcode = instruction[31:26];
    assign rd = instruction[25:21];
    assign rs = instruction[20:16];
    assign rt = instruction[15:11];
    assign shamt = instruction[10:6];
    assign func = instruction[5:0];
    assign address_constant = instruction[15:0];
    assign jaddress = instruction[25:0];
endmodule
/*
ALUctrl = 0 => ADD
1 == Sub (inp1 - inp2) == rs - rt
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
module ALU (
 input wire [31:0] inp1, input wire [31:0] inp2 , input wire [4:0] ALUctrl , input wire clk ,
 output reg [31:0] ALUout , input wire [4:0] shamt
);
    wire[63:0] product;
    assign product = $signed(inp1) * $signed(inp2);
    always @(inp1 or inp2 or ALUctrl or shamt or product) begin
        case (ALUctrl)
            0: ALUout = inp1 + inp2;
            1: ALUout = inp1 - inp2;
            2: ALUout = inp1 & inp2;
            3: ALUout = inp1 ^ inp2;
            4: ALUout = inp1 | inp2;
            5: ALUout = ~inp1;
            6: ALUout = inp1 << shamt;
            7: ALUout = inp1 >> shamt;
            8: ALUout = ($signed(inp1) != $signed(inp2));
            9: ALUout = ($signed(inp1) == $signed(inp2));
            10: ALUout = ($signed(inp1) < $signed(inp2));
            11: ALUout = ($signed(inp1) <= $signed(inp2));
            12: ALUout = ($signed(inp1) > $signed(inp2));
            13: ALUout = ($signed(inp1) >= $signed(inp2));
            14: ALUout = inp2 << 16;
            15: ALUout = product[31:0];
            16: ALUout = product[63:32];
            default: ALUout = 32'b0;
        endcase
    end
endmodule

module Floating_point_to_Binary (
    input [31:0] floating_input , output reg [31:0] bin_output
);
    reg sign;
    reg [32:0] mantissa;
    reg [7 : 0] exponent;
    reg [7:0] diff;
    integer i;
    always @(floating_input) begin
        {sign, exponent, mantissa[31:9]} = floating_input;
        mantissa[32] = 1;
        mantissa[8:0] = 0;
        bin_output = 0;
        diff = exponent - 127;
        if (diff > 30) begin
            bin_output = 0;
        end
        else if(diff < 0) begin
            bin_output = 0;
        end
        else begin
            for(i = diff ; i > -1 ; i = i - 1) begin
                bin_output[i] = mantissa[32 + (i - (diff))];
            end
        end
    end
endmodule

module Binary_to_FloatingPoint (
    input [31:0] bin_input , output reg[31:0] floating_output
);
    reg[22:0] mantissa;
    reg sign;
    reg [7:0] exponent;
  reg[4:0] i;
    integer j;
    always @(bin_input) begin
        sign = 0;
        exponent = 31;
        if (bin_input == 0) begin
            floating_output = 32'h00000000;
        end
        else begin
          for (i = 31; bin_input[i] == 0; i = i - 1) begin
            exponent = i;
          end

            exponent = exponent - 1;

            mantissa = 0;
            if (i >= 23) begin
                for (j = 0; j < 23; j = j + 1) begin
                    mantissa[22 - j] = bin_input[i - 1 - j];
                end
            end 
            else begin
                for(j = exponent - 1 ; j >= 0 ; j = j - 1) begin
                    mantissa[22 + (j - (exponent - 1))] = bin_input[j];
                end
            end
            exponent = exponent + 127;
            floating_output = {sign , exponent, mantissa};
        
        end
    end
endmodule

module floating_adder (
    input wire [31:0] inp1, input [31:0] inp2, output reg[31:0] out
);
    reg signa, signb;
    reg [7:0] exponenta, exponentb;
    reg [7:0] diff;
    reg [23:0] ruffa, ruffb; // Changed to 24-bit for better alignment
    reg[24:0] ans;
    reg[23:0] manta, mantb;
    integer i;
    
    always @(inp1 or inp2) begin
        signa = inp1[31]; signb = inp2[31];
        exponenta = inp1[30:23]; exponentb = inp2[30:23];
        manta = {1'b1, inp1[22:0]}; mantb = {1'b1, inp2[22:0]};
        
        if(signa == signb) begin
            if(exponenta > exponentb) begin
                diff = exponenta - exponentb;
                ruffb = {1'b1, inp2[22:0]} >> diff; // Explicit shift with proper width
                ans = {1'b0, manta} + {1'b0, ruffb}; // Use full 25 bits for addition
                
                if(ans[24] == 1) begin
                    ans = ans >> 1;
                    exponenta = exponenta + 1;
                end
                out = {signa, exponenta, ans[22:0]};
            end
            else begin
                diff = exponentb - exponenta;
                ruffa = {1'b1, inp1[22:0]} >> diff; // Explicit shift with proper width
                ans = {1'b0, ruffa} + {1'b0, mantb}; // Use full 25 bits for addition
                
                if(ans[24] == 1) begin
                    ans = ans >> 1;
                    exponentb = exponentb + 1;
                end
                out = {signb, exponentb, ans[22:0]};
            end
        end
        else begin
            if (exponenta == exponentb && manta == mantb) begin
                out = 32'b0; // Return true zero (all bits zero)
            end
            else begin
            // Rest of your code for different signs remains the same
            if(inp1[30:0] > inp2[30:0]) begin
                diff = exponenta - exponentb;
                ruffb = mantb >> diff;
                manta = manta - ruffb;
                
                // Find first '1' bit for normalization
                for (i = 22; i >= 0; i = i - 1) begin
                    if (manta[i] == 1) begin
                        exponenta = exponenta - (23 - i);
                        manta = manta << (23 - i);
                        i = -1; // Break the loop
                    end
                end
                
                out = {signa, exponenta, manta[22:0]};
            end 
            else begin
                diff = exponentb - exponenta;
                ruffb = manta >> diff;
                manta = mantb - ruffb;
                exponenta = exponentb;
                
                // Find first '1' bit for normalization
                for (i = 22; i >= 0; i = i - 1) begin
                    if (manta[i] == 1) begin
                        exponenta = exponenta - (23 - i);
                        manta = manta << (23 - i);
                        i = -1; // Break the loop
                    end
                end
                
                out = {signb, exponenta, manta[22:0]};
            end
            end
        end
    end
endmodule

module bit_inverser (
    input invert , input [31:0] inp , output [31:0] out
);
    assign out = (invert)? inp ^ 32'h80000000 : inp;
endmodule

//Put rt into inp1
module FPU (
    input wire [31:0] inp1, input wire [31:0] inp2, output reg [31:0] FPUout,
    input wire [5:0] opcode, output reg cc  // Added cc as output
);
    wire [31:0] Adder_cout;
    // reg cc;
    wire invert;
    assign invert = (opcode == 6'b100011) || (opcode == 6'b100100) || // c.eq.ss 
                   (opcode == 6'b100101) || // c.le.ss
                   (opcode == 6'b100110) || // c.lt.ss
                   (opcode == 6'b100111) || // c.ge.ss
                   (opcode == 6'b101000);   // c.gt.ss
                   
    wire [31:0] inp2_intoALU;
    bit_inverser bt(.invert(invert), .inp(inp2), .out(inp2_intoALU));
    floating_adder fa(.inp1(inp1), .inp2(inp2_intoALU), .out(Adder_cout));
    
    // For comparison operations, we need to evaluate zero as well
    wire is_zero;
    assign is_zero = (Adder_cout[30:0] == 31'b0); // Check if exponent and mantissa are all zeros

    always @(inp1 or inp2 or opcode or Adder_cout or is_zero) begin
        case (opcode)
            6'b100010: FPUout = Adder_cout; // add.s
            6'b100011: FPUout = Adder_cout; // sub.s
            
            // Comparison operations using subtraction results
            6'b100100: cc = is_zero;                  // c.eq.s: true if result is zero
            6'b100101: cc = Adder_cout[31] || is_zero; // c.le.s: true if result is negative or zero
            6'b100110: cc = Adder_cout[31] && !is_zero; // c.lt.s: true if result is negative and not zero
            6'b100111: cc = !Adder_cout[31] || is_zero; // c.ge.s: true if result is positive or zero
            6'b101000: cc = !Adder_cout[31] && !is_zero; // c.gt.s: true if result is positive and not zero
            
            6'b101001: FPUout = inp1; // mov.s.cc
            default: FPUout = FPUout;
        endcase
    end
endmodule

module PC_Controller (
 input clk , output reg [9:0] PC, input rst , input jump , input [25:0] jaddress , input branch , input [15:0] branchval
);
    always @(posedge clk ) begin
        if(rst) begin
            PC = 0;
        end
        else begin
            if(jump) begin
                PC = jaddress[9:0];  // Only use the bottom 10 bits
            end
            else if(branch) begin
                PC = PC + 1 + branchval;
            end
            else begin
                PC = PC + 1;
            end
        end
    end
endmodule
/*
ALUctrl = 0 => ADD
1 == Sub (inp1 - inp2) == rs - rt
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
module ALU_controller (
 input wire [5:0] opcode , input wire[5:0] func , output reg [4:0] ALUctrl
);
    always @(opcode or func) begin
        if(opcode == 6'b000111 || opcode == 6'b001000) begin // lw or sw
            ALUctrl = 0; // add
        end
        else if(opcode == 1) begin // addi
            ALUctrl = 0;
        end
        else if(opcode == 2) begin // andi
            ALUctrl = 2;
        end
        else if(opcode == 3) begin // ori
            ALUctrl = 4;
        end
        else if(opcode == 4) begin // xori
            ALUctrl = 3;
        end
        else if(opcode == 5) begin // addui
            ALUctrl = 0;
        end
        else if(opcode == 6'b001001 || opcode == 6'b010100 || opcode == 6'b010110) begin // slti and ble , bleu
            ALUctrl = 10;
        end
        else if(opcode == 6'b001010 || opcode == 6'b010000) begin // seq, beq
            ALUctrl = 9;
        end
        else if(opcode == 6'b010001) begin // bne
            ALUctrl = 8;
        end
        else if(opcode == 6'b010010 || opcode == 6'b010111) begin // bgt ,bgtu
            ALUctrl = 12;
        end
        else if(opcode == 6'b010011) begin // bgte
            ALUctrl = 13;
        end
        else if(opcode == 6'b010101) begin // bleq
            ALUctrl = 11;
        end
        else if(opcode == 6'b001011) begin//lui
            ALUctrl = 14;
        end
        else begin
            if(opcode == 6'b000000) begin
                case (func)
                    0: ALUctrl = 0;
                    1: ALUctrl = 1;
                    2: ALUctrl = 2;
                    3: ALUctrl = 4;
                    4: ALUctrl = 5;
                    5: ALUctrl = 3;
                    6: ALUctrl = 0;
                    7: ALUctrl = 1;
                    8: ALUctrl = 10;
                    9: ALUctrl = 6;
                    10: ALUctrl = 7;
                    11: ALUctrl = 16;
                    12: ALUctrl = 15;
                    default: ALUctrl = 31;
                endcase
            end
            else begin
                ALUctrl = 5'b11111;
            end
        end
    end
endmodule

module SignExtender (
 input wire[15:0] inp , output wire [31:0] out
);
    assign out = {{16{inp[15]}}, inp};
endmodule

module mux2_1 #(
 parameter size = 32
) (
 input wire [size - 1 : 0] inp0 , input wire [size - 1 : 0] inp1, input select , output wire[size - 1 : 0] out
);
 assign out = (select)? inp0 : inp1;
endmodule

module Controller (
 input clk , input wire [5:0] opcode , output reg write_reg , output reg data_write ,
 output reg immediate , output reg jump, output reg branch , output reg jal , output reg select_ALU_or_Mem,//0 for ALU, 1 for MEM
 input rst , output reg mfc1
);
  always @(opcode or rst) begin
    if(rst) begin
      immediate = 0;
      jump = 0;
      branch = 0;
      select_ALU_or_Mem = 0;
      data_write = 0;
      write_reg = 0;
      jal = 0;
      mfc1 = 0;
    end
    else if(opcode == 0) begin // Rtype instruction
      data_write = 0;
      write_reg = 1;
      immediate = 0;
      select_ALU_or_Mem = 0;
      branch = 0;
      jump = 0;
      jal = 0;
      mfc1 = 0;
    end
    else if(opcode == 1 ||opcode == 2 ||opcode == 3 ||opcode == 4 ||opcode == 5) begin //addi, andi , xori, ori , addiu
      data_write = 0;
      write_reg = 1;
      immediate = 1;
      select_ALU_or_Mem = 0;
      branch = 0;
      jump = 0;
      jal = 0;
      mfc1 = 0;
    end
    else if(opcode == 7) begin // lw
      write_reg = 1;
      data_write = 0;
      immediate = 1;
      select_ALU_or_Mem = 1;
      branch = 0;
      jump = 0;
      jal = 0;
      mfc1 = 0;
    end
    else if(opcode == 8) begin // sw
      write_reg = 0;
      data_write = 1;
      immediate = 1;
      select_ALU_or_Mem = 0;
      branch = 0;
      jump = 0;
      jal = 0;
      mfc1 = 0;
    end
    else if(opcode == 9 || opcode == 10) begin // slti , seq
      write_reg = 1;
      data_write = 0;
      immediate = 1;
      select_ALU_or_Mem = 0;
      branch = 0;
      jump = 0;
      jal = 0;
      mfc1 = 0;
    end
    else if(opcode == 6'b001011) begin //lui
      write_reg = 1;
      data_write = 0;
      immediate = 1;
      select_ALU_or_Mem = 0;
      branch = 0;
      jump = 0;
      jal = 0;
      mfc1 = 0;
    end
    else if(opcode == 16 || opcode == 17 ||opcode == 18 || opcode == 19 || opcode == 20 || opcode == 21 || opcode == 22 || opcode == 23) begin //all branches
      write_reg = 0;
      data_write = 0;
      immediate = 0;
      select_ALU_or_Mem = 0;
      branch = 1;
      jump = 0;
      jal = 0;
      mfc1 = 0;
    end
    else if(opcode == 6'b011000 || opcode == 6'b011001) begin // All jump except jal
      write_reg = 0;
      data_write = 0;
      immediate = 0;
      select_ALU_or_Mem = 0;
      branch = 0;
      jump = 1;
      jal = 0;
      mfc1 = 0;
    end
    else if(opcode == 6'b011010) begin // jal handled seperately as it requires to store the value of PC + 4 into $ra
      write_reg = 1;
      data_write = 0;
      immediate = 0;
      select_ALU_or_Mem = 0;
      branch = 0;
      jump = 1;
      jal = 1;
      mfc1 = 0;
    end
    else if(opcode == 6'b100000) begin//mfc1
      write_reg = 1;
      data_write = 0;
      immediate = 0;
      select_ALU_or_Mem = 0;
      branch = 0;
      jump = 0;
      jal = 0;
      mfc1 = 1;
    end
    else begin
      write_reg = 0;
      data_write = 0;
      immediate = 0;
      select_ALU_or_Mem = 0;
      branch = 0;
      jump = 0;
      jal = 0;
      mfc1 = 0;
    end
  end
endmodule

module FPR_controller (
    input [5:0] opcode, output write_fpr , output mtc1
);
  assign write_fpr = (opcode == 6'b100001 || opcode == 6'b100010 || opcode == 6'b100011 || opcode == 6'b101001);//mtc1, add.s, sub.s , mov.s.cc
  assign mtc1 = (opcode == 6'b100001);
endmodule

module CPU (
 input rst , input clk , input wire [31:0] inst_data ,
 input wire [9:0] address , input wire write_instruction , input wire write_data,
 output wire [31:0] OutputOfRs
);
 wire [9:0] PC;
 wire [31:0] instruction;
 wire [31:0] ALU_out;
 wire [9:0] memory_in;
 wire [31:0] rt_out , rs_out , memory_write , memory_out , rt_or_address_to_ALU, extended_address , rtout_f, rsout_f , FPU_out , converted_bin , converted_float , final_wire_going_into_register_rd , data_write_in_fpr;
 wire [4:0] rt , rs , rd , shamt;
 wire [5:0] opcode , func;
 wire [15:0] address_constant;
 wire [25:0] jaddress;
 wire branch , jump , jal , mem_write , immediate, select_ALU_or_Mem, write_reg, write_f_register , mtc1 , mfc1;
 wire [4:0] wire_going_into_rs;
 wire cc_out;
 //mem_write instead of data_write
 wire [4:0] ALUctrl;
 assign OutputOfRs = rs_out;
 assign wire_going_into_rs = (branch | mem_write)? rd : rs;
 //this thing is to handle jal
 wire[4:0] write_in_Register;
 assign write_in_Register = (jal)? 5'd3 : rd;
 wire [31:0] write_data_in_register , wire_going_into_register_rd;
 assign wire_going_into_register_rd = (jal)? {22'b0 , PC} : write_data_in_register;
 assign final_wire_going_into_register_rd = (mfc1)? converted_bin : wire_going_into_register_rd; // essentially for mfc1
 assign rt_or_address_to_ALU = (immediate)? extended_address : rt_out;
 SignExtender sign_extend_addr_const(.inp(address_constant) , .out(extended_address));
 assign write_data_in_register = (select_ALU_or_Mem)? memory_out : ALU_out;
 DistributedMemory instruction_mem(.a(address),.d(inst_data),.dpra(PC),.clk(clk),.we(write_instruction),.dpo(instruction));

 assign memory_in = (write_data)? address : ALU_out[9:0];
 assign memory_write = (write_data)? inst_data : rt_out;
 DistributedMemory data_mem(.a(memory_in) , .d(memory_write) , .dpra(ALU_out[9:0]) , .clk(clk) , .we(mem_write | write_data) , .dpo(memory_out));

 Splitter split(.instruction(instruction) , .rt(rt) , .rs(rs) , .rd(rd) , .shamt(shamt), .func(func), .opcode(opcode) , .address_constant(address_constant) , .jaddress(jaddress));
 Controller brain(.clk(clk) , .opcode(opcode) , .write_reg(write_reg) , .data_write(mem_write) ,
 .immediate(immediate) , .jump(jump) , .branch(branch) , .jal(jal) , .select_ALU_or_Mem(select_ALU_or_Mem) ,
 .rst(rst) , .mfc1(mfc1));
 RegisterFile RAM(.rd(write_in_Register) , .rs(wire_going_into_rs) , .rt(rt) , .clk(clk) , .we(write_reg) , .write_data(final_wire_going_into_register_rd) , .rst(rst) , .rt_out(rt_out) , .rs_out(rs_out));

 ALU_controller nerves(.opcode(opcode) , .func(func) , .ALUctrl(ALUctrl));
 ALU brawn(.inp1(rs_out) , .inp2(rt_or_address_to_ALU) , .ALUctrl(ALUctrl) , .clk(clk) , .ALUout(ALU_out) , .shamt(shamt));
 PC_Controller legs(.clk(clk) , .PC(PC) , .rst(rst) , .jump(jump) , .jaddress(jaddress) , .branch(branch & ALU_out[0]) , .branchval(address_constant));


    //FP registers
    assign data_write_in_fpr = (mtc1)? rt_out : FPU_out;
    Floating_point_to_Binary fptb(.floating_input(rtout_f) , .bin_output(converted_bin));
    Binary_to_FloatingPoint btfp(.bin_input(rt_out) , .floating_output(converted_float));


    RegisterFile Fpr(.rd(rd) , .rs(rs) , .rt(rt) , .clk(clk) , .we(write_f_register) , .write_data(data_write_in_fpr) , .rs_out(rsout_f) , .rt_out(rtout_f) , .rst(rst));

    FPR_controller fpr_ctrlr(.opcode(opcode) , .write_fpr(write_f_register) , .mtc1(mtc1));

    FPU floating_point_ALU(.inp1(rsout_f), .inp2(rtout_f), .FPUout(FPU_out), 
                       .opcode(opcode), .cc(cc_out));
endmodule


module CPU_tb;
    reg clk, rst;
    reg [31:0] inst_data;
    reg [9:0] address;
    reg write_instruction, write_data;
    wire [31:0] OutputOfRs;
    // Instantiate the CPU
    CPU uut (
        .rst(rst),
        .clk(clk),
        .inst_data(inst_data),
        .address(address),
        .write_instruction(write_instruction),
        .write_data(write_data),
        .OutputOfRs(OutputOfRs)
    );
    // Clock Generation
    always #10 clk = ~clk;

    initial begin
      // $monitor("PC: %d, branch: %b, ALU_out: %d", uut.PC, uut.branch, uut.ALU_out);
        
      // I am going to implement insertion sort of array of 10 elements
      // Load the numbers in data memory at address 0 to 9
      rst = 0;
      
        uut.data_mem.Address_locations[0] = 31;
        uut.data_mem.Address_locations[1] = 34;
        uut.data_mem.Address_locations[2] = 62;
        uut.data_mem.Address_locations[3] = 12;
        uut.data_mem.Address_locations[4] = 0;
        uut.data_mem.Address_locations[5] = 7;
        uut.data_mem.Address_locations[6] = 2;
        uut.data_mem.Address_locations[7] = 17;
        uut.data_mem.Address_locations[8] = 89;
        uut.data_mem.Address_locations[9] = 19;




        // $monitor("immediate: %b, rt: %d", uut.immediate, uut.RAM.Registers[18]);

        clk = 0;
        rst = 1;
        address = 0;
        write_instruction = 0;
        write_data = 0;
        inst_data = 32'b0;


        #10;
        // Write the addi instruction to instruction memory at address 0
        write_instruction = 1;

        // $1 ---------> n
        // $2 ---------> i
        // $3 ---------> a[i]
        // $4 ---------> j
        // $5 ---------> a[j]
        // $6 ---------> key

        // $monitor("i: %d, n: %d, a[i]: %d, j: %d, a[j]: %d", uut.RAM.Registers[2], uut.RAM.Registers[1], uut.RAM.Registers[3], uut.RAM.Registers[4], uut.RAM.Registers[5]);
        $monitor("PC: %d, branch: %b, ALU_out: %d, i: %d, j: %d", uut.PC, uut.branch, uut.ALU_out, uut.RAM.Registers[2], uut.RAM.Registers[4]);

        address = 0;
        // addi $1, $0, 10  ---> initialize n = 10
        inst_data = 32'b000001_00001_00000_0000000000001010;
        #20;

        address = 1;
        // addi $2, $0, 1  ---> initialize i = 1
        inst_data = 32'b000001_00010_00000_0000000000000001;
        #20;

// outer loop

        address = 2;
        //bgte $2, $1, 13  ---> if i > n, jump to end
        inst_data = 32'b010011_00010_00000_00001_00000001101;
        #20;

        address = 3;
        // lw $3, 0($2)  ---> $3 = a[i]
        inst_data = 32'b000111_00011_00010_00000_00000000000;
        #20;

        address = 4;
        // addi $6, $3, 0  ---> key = a[i]
        inst_data = 32'b000001_00110_00011_0000000000000000;
        #20;
        
        address = 5;
        // addi $4, $2, -1  ---> j = i - 1
        inst_data = 32'b000001_00100_00010_1111111111111111;
        #20;

// inner loop
        
        address = 6;
        // ble $4, $0, 5  ---> if j < 0, jump to insert
        inst_data = 32'b010100_00100_00000_00000_00000000101;
        #20;

        address = 7;
        // lw $5, 0($4)  ---> $5 = a[j]
        inst_data = 32'b000111_00101_00100_00000_00000000000;
        #20;

        address = 8;
        // bgte $6, $5, 3  ---> if key >= a[j], jump to insert
        inst_data = 32'b010011_00110_00000_00101_00000000011;
        #20;

        address = 9;
        // sw $5, 1($4)  ---> a[j+1] = a[j]
        inst_data = 32'b001000_00100_00000_00101_00000000001; // ERROR: 4 kaha ayega vo dekhana hai
        #20;

        address = 10;
        // addi $4, $4, -1  ---> j = j - 1
        inst_data = 32'b000001_00100_00100_1111111111111111;
        #20;

        address = 11;
        // j 6 ---> jump to inner loop
        inst_data = 32'b011000_00000000000000000000000110;
        #20;

    // insert block

        address = 12;
        // addi $4, $4, 1  ---> j = j + 1
        inst_data = 32'b000001_00100_00100_0000000000000001;
        #20;

        address = 13;
        // sw $6, 0($4)  ---> a[j+1] = key
        inst_data = 32'b001000_00100_00000_00110_00000000000;
        #20;

        address = 14;
        // addi $2, $2, 1  ---> i = i + 1
        inst_data = 32'b000001_00010_00010_0000000000000001;
        #20;

        address = 15;
        // j 2  ---> jump to outer loop
        inst_data = 32'b011000_00000000000000000000000010;
        #20;

        //end
        address = 16;
        // addi $1, $0, 0  ---> n = 0
        inst_data = 32'b000001_00001_00000_0000000000000000;
        #20;



        





      	#52
      	rst = 0;
        write_instruction = 0;
        #6000;

        // Check output
        // $display("1111111111111111 = %d", $signed(16'b1111111111111111));
        $display("Register $1 value = %d", $signed(uut.RAM.Registers[1]));
        $display("Register $2 value = %d", $signed(uut.RAM.Registers[2]));
        $display("Register $3 value = %d", $signed(uut.RAM.Registers[3]));
        $display("Register $4 value = %d", $signed(uut.RAM.Registers[4]));
        $display("Register $5 value = %d", $signed(uut.RAM.Registers[5]));
        $display("Register $6 value = %d", $signed(uut.RAM.Registers[6]));
        $display("Register $7 value = %d", $signed(uut.RAM.Registers[7]));
        $display("Register $8 value = %d", $signed(uut.RAM.Registers[8]));
        $display("Register $9 value = %d", $signed(uut.RAM.Registers[9]));

        // display array
        $display("Array values:");
        for (integer i = 0; i < 10; i = i + 1) begin
            $display("a[%0d] = %d", i, uut.data_mem.Address_locations[i]);
        end

      
        $finish;
    end
endmodule

