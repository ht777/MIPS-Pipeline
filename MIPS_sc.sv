// Code your design here

module MIPS_ALU(source1,source2,ctrl,zero_flag,ALU_result,clk);
  parameter n=32;
  input clk;
  input logic [n-1:0] source1,source2;
  input logic [2:0] ctrl;
  output logic [n-1:0] ALU_result;
  output logic zero_flag;
  parameter NOP=3'b000, ADD=3'b010, SUB=3'b110, AND=3'b101, OR=3'b001,
  			XOR=3'b011, SLT=3'b100, SLL=3'b111;
  
  logic [n-1:0] add_result,sub_result,or_result,and_result, xor_result,
  				 result, sll_result, slt_result;
  logic add_carry,zf;
  
  adder32 A1 (source1,source2,add_result);
  sub32 A2 (source1,source2,sub_result,zf);
  or32 A3 (source1,source2,or_result);
  and32 A4 (source1,source2,and_result);
  xor32 A5 (source1,source2,xor_result);
  
  slt32 A6 (.a(source1[31]),.b(source2[31]),.subres(sub_result[31]),.r(slt_result));
  sll32 A7 (source1, source2, sll_result);
 
  
  //always @(posedge clk)
    always @(*)

  case (ctrl)
    
    NOP:  begin result = {n{1'b0}}; zero_flag = 1'b0; end
    ADD:  begin result = add_result; zero_flag = 1'b0; end
    SUB: begin result = sub_result; zero_flag = zf; end
    AND: begin result = and_result; zero_flag = 1'b0; end
     OR: begin result = or_result; zero_flag = 1'b0; end
     XOR: begin result = xor_result; zero_flag = 1'b0; end
     SLL: begin result = sll_result; zero_flag = 1'b0; end
    SLT: begin result = slt_result;  zero_flag = 1'b0; end
    default : begin result = {n{1'b0}}; zero_flag = 1'b0; end
  endcase

  
  //111 slt 
  assign ALU_result = result;
  
endmodule


module sll32 #(parameter n=32) (input logic [31:0] a,b,
                output logic [31:0] c);
  always @(*)
    //begin
    case (b)
      0: assign c = a;
      1: assign c = {a[n-1:1],{1{1'b0}}};
      2: assign c = {a[n-1:2],{2{1'b0}}};
      3: assign c = {a[n-1:3],{3{1'b0}}};
      4: assign c = {a[n-1:4],{4{1'b0}}};
      5: assign c = {a[n-1:5],{5{1'b0}}};
      6: assign c = {a[n-1:6],{6{1'b0}}};
      7: assign c = {a[n-1:7],{7{1'b0}}};
      8: assign c = {a[n-1:8],{8{1'b0}}};
      9: assign c = {a[n-1:9],{9{1'b0}}};
      10: assign c = {a[n-1:10],{10{1'b0}}};
      11: assign c = {a[n-1:11],{11{1'b0}}};
      12: assign c = {a[n-1:12],{12{1'b0}}};
      13: assign c = {a[n-1:13],{13{1'b0}}};
      14: assign c = {a[n-1:14],{14{1'b0}}};
      15: assign c = {a[n-1:15],{15{1'b0}}};
      16: assign c = {a[n-1:16],{16{1'b0}}};      
      17: assign c = {a[n-1:17],{17{1'b0}}};
      18: assign c = {a[n-1:18],{18{1'b0}}};
      19: assign c = {a[n-1:19],{19{1'b0}}};
      20: assign c = {a[n-1:20],{20{1'b0}}};
      21: assign c = {a[n-1:21],{21{1'b0}}};
      22: assign c = {a[n-1:22],{22{1'b0}}};
      23: assign c = {a[n-1:23],{23{1'b0}}};
      24: assign c = {a[n-1:24],{24{1'b0}}};      
      25: assign c = {a[n-1:25],{25{1'b0}}};
      26: assign c = {a[n-1:26],{26{1'b0}}};
      27: assign c = {a[n-1:27],{27{1'b0}}};
      28: assign c = {a[n-1:28],{28{1'b0}}};
      29: assign c = {a[n-1:29],{29{1'b0}}};
      30: assign c = {a[n-1:30],{30{1'b0}}};
      31: assign c = {31{1'b0}};
    endcase
  endmodule
  
  
  
  module slt32 ( input logic a,b,
                input logic subres,
                output logic [31:0] r);
    
    assign r ={{31{1'b0}}, (~(a^b) & subres) | (a & ~b)};
  endmodule


module ALU_control (input logic[5:0] funct,
                    input logic clk,
                    input logic [1:0] ALUop,
                    output logic [2:0] ALU_ctrl);
  //always @(posedge clk)
    always @(*)

  case (ALUop)
    2'b00: begin
      assign ALU_ctrl= 3'b010;
    end
    2'b01: begin
      assign ALU_ctrl = 3'b110;
    end
    default: begin
      case(funct)
        6'b100110: begin //xor
           assign ALU_ctrl= 3'b011;
        end
       
        6'b000000: begin// sll
           assign ALU_ctrl= 3'b111;
        end
        6'b100000: begin// add
           assign ALU_ctrl= 3'b010;
        end
        6'b100010: begin //sub
           assign ALU_ctrl= 3'b110;
        end
        6'b100100: begin //and
           assign ALU_ctrl= 3'b101;
        end
        6'b100101: begin //or
           assign ALU_ctrl= 3'b001;
        end
        6'b101010: begin //slt
           assign ALU_ctrl= 3'b100;
        end
     
      endcase
        end
  endcase
          
endmodule

module control_logic(input logic [5:0] opcode,
                     input clk,
                    output logic RegDst,Jump,Branch,MemRead,MemtoReg,MemWrite,
                    ALUsrc,RegWrite,
                     output logic [1:0] ALUop);
  //always @(posedge clk)
    //begin
	  always @(*)

  case (opcode)
    6'b000100: begin //branch
      assign Branch=1'b1;
      assign ALUop=2'b01;

      assign RegDst=1'bx;assign Jump=1'b0;assign MemRead=1'b0;assign MemtoReg=1'b0;
      assign MemWrite=1'b0;assign ALUsrc=1'b0;assign RegWrite=1'b0;
    end
    //Jump
    6'b000010: begin
      assign Branch=1'b0;
      assign ALUop=2'bxx;
      assign RegDst=1'bx;assign Jump=1'b1;assign MemRead=1'b0;assign MemtoReg=1'b0;
      assign MemWrite=1'b0;assign ALUsrc=1'b1;assign RegWrite=1'b1;
    end
    //Load Byte
    6'b100000: begin 
      assign Branch=1'b0;
      assign ALUop=2'b00;
      assign RegDst=1'b0;assign Jump=1'b0;assign MemRead=1'b1;assign MemtoReg=1'b1;
      assign MemWrite=1'b0;assign ALUsrc=1'b1;assign RegWrite=1'b1;
    end
      //store byte
    6'b101000: begin 
      assign Branch=1'b0;
      assign ALUop=2'b00;
      assign RegDst=1'b0;assign Jump=1'b0;assign MemRead=1'b0;assign MemtoReg=1'b0;
      assign MemWrite=1'b1;assign ALUsrc=1'b1;assign RegWrite=1'b0;
    end
    
    //Reg type
	 6'b000000: begin 
      assign Branch=1'b0;
      assign ALUop=2'b10;
      assign RegDst=1'b1;assign Jump=1'b0;assign MemRead=1'b0;assign MemtoReg=1'b0;
      assign MemWrite=1'b0;assign ALUsrc=1'b0;assign RegWrite=1'b1;
    end
  endcase
    //end
endmodule

module registers #(parameter n=32)
  (input logic [4:0] read_register1,read_register2, write_register,
   input logic [n-1:0] write_data,
   input logic RegWrite,clk,
   output logic [n-1:0] read_data1,read_data2);
  
  logic [31:0] RAM [0:31];
  
      initial
      $readmemh ("C:/Users/thaku/Desktop/Project files/regfile.txt",RAM);
  /*    
  initial
    begin
    RAM[0] = 32'h00000000;
  RAM[1] = 32'h00000000;
  RAM[2] = 32'h00000000;
  RAM[3] = 32'h00000000;
  RAM[4] = 32'h00000000;
  RAM[5] = 32'h00000000;
  RAM[6] = 32'h00000000;
  RAM[7] = 32'h00000000;
  RAM[8] = 32'h00000000;
  RAM[9] = 32'h00000000;
  RAM[10] = 32'h00000000;
  RAM[11] = 32'h00000000;
  RAM[12] = 32'h00000000;
  RAM[13] = 32'h00000000;
  RAM[14] = 32'h00000000;
  RAM[15] = 32'h00000000;
  RAM[16] = 32'h00000000;
  RAM[17] = 32'h00000000;
  RAM[18] = 32'h00000000;
  RAM[19] = 32'h00000000;
  RAM[20] = 32'h00000000;
  RAM[21] = 32'h00000000;
  RAM[22] = 32'h00000000;
  RAM[23] = 32'h00000000;
  RAM[24] = 32'h00000000;
  RAM[25] = 32'h00000000;
  RAM[26] = 32'h00000000;
  RAM[27] = 32'h00000000;
  RAM[28] = 32'h00000000;
  RAM[29] = 32'h00000000;
  RAM[30] = 32'h00000000;
  RAM[31] = 32'h00000000;
    end
	*/
  
  always @(posedge clk)
  if (RegWrite)
    RAM[write_register]<=write_data;
  
  assign read_data1 = RAM[read_register1];
  assign read_data2 = RAM[read_register2];

  
endmodule

module instr_mem
  (input logic [31:0] address,
   output logic [31:0] data);
  
  //logic addr_equal;
  parameter entries = 128;
  
  
  logic [31:0] instr_mem_data[0:127];
  
  initial
    begin
      $readmemh ("C:/Users/thaku/Desktop/Project files/memfile.txt", instr_mem_data, 0 ,127);
    end
  


    
        assign data = instr_mem_data[address[8:2]][31:0];
    
    
  
endmodule

module instr_mem_cache 
  (input logic [31:0] address,
   input logic clk,
   output logic [31:0] data,
   output logic hit);
  
  wire [31:0] temp_data;
  
  
  //            ADDRESS
	//  31         5 4         2 1   0
	// +------------+-----------+-----+
	// |     tag    |   index   | X X |
	// +------------+-----------+-----+
	//
	// 							  X X - byte index
	//
	//  tag: 27 bits
  //  index: 3 bits (8 blocks)
	//
	//             BLOCK
	//    59  58     32 31     0
	//   +---+---------+--------+
	//   | V |   tag   |  word  |
	//   +---+---------+--------+
	
  logic [26:0] address_tag,block_tag;
  logic [2:0] address_index;
  logic [2:0] byte_index;
  logic [59:0] block;
  logic [31:0] block_word;
  logic valid;
  
  assign address_tag = address[31:5];
  assign address_index = address [4:2];
  assign byte_index = address[1:0];
  
  logic [59:0] block0;
  logic [59:0] block1;
  logic [59:0] block2;
  logic [59:0] block3;
  logic [59:0] block4;
  logic [59:0] block5;
  logic [59:0] block6;
  logic [59:0] block7;
  
    instr_mem I1 (.address(address),.data(temp_data));

  //assigning valid bit to 0
  initial
    begin
      block0[59] = 1'b0;
      block1[59] = 1'b0;
      block2[59] = 1'b0;
      block3[59] = 1'b0;
      block4[59] = 1'b0;
      block5[59] = 1'b0;
      block6[59] = 1'b0;
      block7[59] = 1'b0;

    end
  
 // always @(posedge clk)
   // begin
  always @(address)

    case (address_index)
      0: begin 
        if ((block0[58:32] == address_tag) && (block0[59]==1'b1))
          begin 
            hit =1'b1;
            data = block0 [31:0];
          end
        else 		//possibility for waiting for another memory request
          begin
            hit = 1'b0;
            block0[58:32] = address_tag;
            block0[59] = 1'b0;
            block0 [31:0] = temp_data;
            data = temp_data;
          end
      end
    
      1: begin 
        if ((block1[58:32] == address_tag) && block1[59]==1'b1)
          begin 
            hit =1'b1;
            data = block1 [31:0];
          end
        else 		
          begin
            hit = 1'b0;
            block1[58:32] = address_tag;
            block1[59] = 1'b1;
            block1 [31:0] = temp_data;
            data = temp_data;
          end
      end
      2: begin 
        if ((block2[58:32] == address_tag) && block2[59]==1'b1)
          begin 
            hit =1'b1;
            data = block2 [31:0];
          end
        else 		
          begin
            hit = 1'b0;
            block2[58:32] = address_tag;
            block2[59] = 1'b1;
            block2 [31:0] = temp_data;
            data = temp_data;
          end
      end
      3: begin 
        if ((block3[58:32] == address_tag) && block3[59]==1'b1)
          begin 
            hit =1'b1;
            data = block3 [31:0];
          end
        else 		
          begin
            hit = 1'b0;
            block3[58:32] = address_tag;
            block3[59] = 1'b1;
            block3 [31:0] = temp_data;
            data = temp_data;
          end
      end
      4: begin 
        if ((block4[58:32] == address_tag) && block4[59]==1'b1)
          begin 
            hit =1'b1;
            data = block4 [31:0];
          end
        else 		
          begin
            hit = 1'b0;
            block4[58:32] = address_tag;
            block4[59] = 1'b1;
            block4 [31:0] = temp_data;
            data = temp_data;
          end
      end
      5: begin 
        if ((block5[58:32] == address_tag) && block5[59]==1'b1)
          begin 
            hit =1'b1;
            data = block5 [31:0];
          end
        else 		
          begin
            hit = 1'b0;
            block5[58:32] = address_tag;
            block5[59] = 1'b1;
            block5 [31:0] = temp_data;
            data = temp_data;
          end
      end
      6: begin 
        if ((block6[58:32] == address_tag) && block6[59]==1'b1)
          begin 
            hit =1'b1;
            data = block6 [31:0];
          end
        else 		
          begin
            hit = 1'b0;
            block6[58:32] = address_tag;
            block6[59] = 1'b1;
            block6 [31:0] = temp_data;
            data = temp_data;
          end
      end
      7: begin 
        if ((block7[58:32] == address_tag) && block7[59]==1'b1)
          begin 
            hit =1'b1;
            data = block7 [31:0];
          end
        else 		
          begin
            hit = 1'b0;
            block7[58:32] = address_tag;
            block7[59] = 1'b1;
            block7 [31:0] = temp_data;
            data = temp_data;
          end
      end
       endcase
 // end
  
 
  
endmodule
  
module datapath #(parameter n=32)
  (

    input logic [n-1:0] PC,
    input logic regwrite,memtoreg,
    			jump,branch,regdst,alusrc,clk,MemRead, MemWrite,
    input logic [2:0] alucontrol,
    input logic [n-1:0] instr,
    output logic [31:0]  newPC,
    output logic [31:0] ALUresult
  
  );
   logic [4:0] ra1,ra2,wa;
  logic [n-1:0] wd,ra1_data,ra2_data,alusrc1,alusrc2,pc_next, 
  				jump_address,branch_address,jump_address2, memory_data, alu_result, instr32sign_ext,readdata,tPC;
  logic PCsrc2;
  assign ra1 = instr[25:21];
  assign ra2 = instr[20:16];
  assign instr32sign_ext = {{16{instr[15]}},instr[15:0]};
  
  mux5 regmux (instr[20:16],instr[15:11],regdst,wa);
  
  
  registers reg_datapath (ra1,ra2, wa, memory_data,
  regwrite,clk,ra1_data,ra2_data);
  
  
  mux32 ALUsrc2_mux (ra2_data,instr32sign_ext,alusrc,alusrc2);
  assign alusrc1 = ra1_data;
  adder32 A1 (PC,32'h00000004,pc_next);
  adder32 A2 (pc_next,{instr32sign_ext[29:0],2'b00}, branch_address);
  
  assign jump_address = {pc_next[31:28], instr[25:0], 2'b00}; 
  
  MIPS_ALU alu (alusrc1,alusrc2,alucontrol,zero,alu_result,clk);
  assign PCsrc2= zero & branch;
  
  //mux32 branch_mux (pc_next,branch_address,PCSrc2,jump_address2);
  //mux32 jump_mux (jump_address2,jump_address,jump,newPC);
  //always @(posedge clk)
  //begin
  assign newPC = jump ? {pc_next[31:28], instr[25:0], 2'b00} : (PCsrc2 ? branch_address : pc_next);
  //end
  datamem dm (clk, MemRead, MemWrite,alu_result[6:0],alusrc2, readdata); 
  
  mux32 regupdate (alu_result, readdata, memtoreg, memory_data);
  //always @(posedge clk)
  //begin
  assign ALUresult = alu_result;
  //assign newPC = tPC;
//end

 
endmodule
module datamem(
		input logic			clk,
		input logic			MemRead, MemWrite,
  		input logic	[6:0]	address,
        input logic	[31:0]	writedata,
  		output logic [31:0]	readdata);
 
	reg [31:0] mem [0:127];  // 32-bit memory with 128 entries
  
    initial
      $readmemh ("C:/Users/thaku/Desktop/Project files/datamemfilename.txt",mem);
 /*
initial
  begin
    mem[0] = 32'h00000000;
    mem[1] = 32'h00000001;
    mem[2] = 32'h00000002;
    mem[3] = 32'h00000003;
    mem[4] = 32'h00000000;
    mem[5] = 32'h00000001;
    mem[6] = 32'h00000002;
    mem[7] = 32'h00000003;
    mem[8] = 32'h00000000;
    mem[9] = 32'h00000001;
    mem[10] = 32'h00000002;
    mem[11] = 32'h00000003;
    mem[12] = 32'h00000000;
    mem[13] = 32'h00000001;
    mem[14] = 32'h00000002;
    mem[15] = 32'h00000003;
    mem[16] = 32'h00000000;
    mem[17] = 32'h00000001;
    mem[18] = 32'h00000002;
    mem[19] = 32'h00000003;
    mem[20] = 32'h00000000;
    mem[21] = 32'h00000001;
    mem[22] = 32'h00000002;
    mem[23] = 32'h00000003;
    
  end
  */

	always @(*) begin
	  //always @(*)

      if (MemWrite && !MemRead) begin
        mem[address] <= writedata;
		end
      else if (MemRead && !MemWrite) begin
        readdata = mem[address][31:0];
      end
	end


endmodule


module MIPS #(parameter n=32)(
  input logic clk,reset,
  output logic [n-1:0] address,tinstr,tPC,
  output logic [n-1:0] data  );

  logic [5:0] opcode;
  logic [5:0] funct;
  logic [5:0] rs;
  logic [5:0] rt;
  logic [5:0] rd;
  logic [1:0] ALUop;
  logic [2:0] ALU_ctrl;
  logic MemRead,MemWrite;

  logic RegDst,Jump,Branch,MemtoReg,ALUsrc,RegWrite;
  logic [n-1:0] PC,newPC,alu_result,tdata;
  logic [n-1:0] instruction;
  logic hit;
  //initial PC = 32'b00000000;
  int count = 50;
  instr_mem_cache IM (PC, clk, instruction, hit);
  
  control_logic cl (instruction [31:26],clk, RegDst, Jump, Branch, MemRead, MemtoReg, MemWrite, ALUsrc, RegWrite, ALUop);
  
  ALU_control aluctrl (instruction[5:0],clk, ALUop, ALU_ctrl);

  datapath dp (PC,RegWrite,MemtoReg,Jump,Branch,RegDst,ALUsrc,clk, MemRead, MemWrite,ALU_ctrl, instruction, newPC, tdata);
 // assign tinstr= instruction;
  //assign tPC = newPC;

  
  
  always @(posedge clk)
  if (count>0)
  begin
  begin
  if (reset == 1)
  begin
    PC <= 32'h00000000;
	count<=count-1;
	
	end
	
	else
	begin
	address=PC;
	tPC <= newPC;
  PC <= newPC;
  data<=tdata;
  tinstr<= instruction;
  	count<=count-1;

  end
   
  end
  end
endmodule

module zero_detector #(parameter n=32)
  (input logic [n-1:0] a,
   output logic y);
  assign y = (a==0);
endmodule

module mux32 #(parameter n=32)
  (input logic[n-1:0] a,b,
  input logic sel,
   output logic [n-1:0] y);
  
  assign y = sel ? b :a;
endmodule

module signextend #(parameter n=32)
  (input logic [(n/2)-1:0] a,
   output logic [n-1:0] y);
  assign y = {{(n/2){a[(n/2)-1]}}, a};
endmodule

module shiftleftby2 #(parameter n=32)
  (input logic [n-1:0] a,
   output logic [n-1:0] y);
  assign y = {a[n-3:0],1'b0,1'b0};
endmodule

module adder32 (input logic [31:0] a,b,
                output logic [31:0] sum);
  assign sum = a+b;
endmodule

module sub32 (input logic [31:0]a,b,
              output logic [31:0] res,
              output logic zero);
  assign res = a-b;
  assign zero = ~|res;
endmodule

module and32 (input logic [31:0]a,b,
              output logic [31:0] res);
  assign res = a & b;
endmodule

module xor32 (input logic [31:0]a,b,
              output logic [31:0] res);
  assign res = a ^ b;
endmodule

module or32 (input logic [31:0]a,b,
              output logic [31:0] res);
  assign res = a | b;
endmodule

module mux5 (input logic [4:0] a,b,
            input logic sel,
             output logic [4:0] res);
  
  assign res = sel ? b : a;
endmodule

  
  