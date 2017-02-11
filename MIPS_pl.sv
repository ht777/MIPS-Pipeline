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
    2'b11: begin//nop
      assign ALU_ctrl = 3'b000;
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
                     output logic [1:0] ALUop,
					 output logic [2:0] inst_type);
  //always @(posedge clk)
    //begin
	  always @(*)
//type 0:reg-reg, 1: alu imm, 2: load, 3:store, 4: branch

  case (opcode)
  6'b111111: begin //nop
      assign Branch=1'b0;
      assign ALUop=2'b11;
	 assign inst_type = 3'b111;
      assign RegDst=1'b0;assign Jump=1'b0;assign MemRead=1'b0;assign MemtoReg=1'b0;
      assign MemWrite=1'b0;assign ALUsrc=1'b0;assign RegWrite=1'b0;
    end
  
    6'b000100: begin //branch
      assign Branch=1'b1;
      assign ALUop=2'b01;
	 assign inst_type = 3'b100;
      assign RegDst=1'b0;assign Jump=1'b0;assign MemRead=1'b0;assign MemtoReg=1'b0;
      assign MemWrite=1'b0;assign ALUsrc=1'b0;assign RegWrite=1'b0;
    end
    //Jump
    6'b000010: begin
      assign Branch=1'b0;
      assign ALUop=2'bxx;
	 assign inst_type = 3'b111;
      assign RegDst=1'b0;assign Jump=1'b1;assign MemRead=1'b0;assign MemtoReg=1'b0;
      assign MemWrite=1'b0;assign ALUsrc=1'b1;assign RegWrite=1'b1;
    end
    //Load Byte
    6'b100000: begin 
      assign Branch=1'b0;
      assign ALUop=2'b00;
	 assign inst_type = 3'b010;
      assign RegDst=1'b0;assign Jump=1'b0;assign MemRead=1'b1;assign MemtoReg=1'b1;
      assign MemWrite=1'b0;assign ALUsrc=1'b1;assign RegWrite=1'b1;
    end
      //store byte
    6'b101000: begin 
      assign Branch=1'b0;
      assign ALUop=2'b00;
	 assign inst_type = 3'b011;
      assign RegDst=1'b0;assign Jump=1'b0;assign MemRead=1'b0;assign MemtoReg=1'b0;
      assign MemWrite=1'b1;assign ALUsrc=1'b1;assign RegWrite=1'b0;
    end
    
    //Reg type
	 6'b000000: begin 
      assign Branch=1'b0;
      assign ALUop=2'b10;
	 assign inst_type = 3'b000;
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
  
  always @(clk)
  if (RegWrite)
    RAM[write_register]<=write_data;
  
  always @(negedge clk)
  begin
  if ((read_register1==write_register) && RegWrite)
	read_data1 <= write_data;
 else 	read_data1 <= RAM[read_register1];
   if ((read_register2==write_register) && RegWrite)
	read_data2 <= write_data;
 else 	read_data2 <= RAM[read_register2];
end
  
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

module regarray_ifid(input logic [31:0] instructioni,PCi,
					 input logic clk,
					output logic [31:0] instructiono,PCo);
	always @(posedge clk)
		begin
		 instructiono <= instructioni;
		 PCo <= PCi;
		 end
	
	
	
endmodule

module regarray_idex(input logic RegDst,Jump,Branch,MemRead,MemtoReg,MemWrite,
                    ALUsrc,RegWrite,clk,
                     input logic [1:0] ALUop,mux_src2i,mux_src1i,
					 input logic [31:0] instr16_signexti,PCi,reg_data1i,reg_data2i,ii,src1x,src2x,
					 input logic [4:0] dest_reg,
					 input logic [2:0] typei,
					 output logic [4:0] dest_regidex,
					 output logic [1:0] WB, mux_src2o, mux_src1o,
					 output logic [2:0] M,typeo,
					 output logic [3:0] EX,					 
					 output logic [31:0] PCo,reg_data1o,reg_data2o,instr16_signexto,instructionidex,src1,src2);
		always @(posedge clk)
		begin
		src1<=src1x;
		src2<=src2x;
		typeo <= typei;
		instructionidex<=ii;
		WB[0] <= RegWrite;
		WB[1] <= MemtoReg;
		M[0]  <= Branch;
		M[1]  <= MemRead;
		M[2]  <= MemWrite;
		EX[0] <= ALUop[0];
		EX[1] <= ALUop[1];
		EX[2] <= RegDst;
		EX[3] <= ALUsrc;
		mux_src1o <= mux_src1i;
		mux_src2o <= mux_src2i;
		dest_regidex <= dest_reg;
		reg_data1o <= reg_data1i;
		reg_data2o <= reg_data2i;
		instr16_signexto <= instr16_signexti;
		end
endmodule 

module regarray_exmem(input logic [1:0] WBi,
					  input logic [2:0] Mi,typei,
					  input logic [31:0] branch_taken_addressi,ALU_resulti,read_data2i,ii,
					  input logic zeroi,clk,
					  input logic [4:0] dest_regi,
					  output logic [1:0] WBo,
					  output logic [2:0] typeo,
					  output logic [31:0] branch_taken_addresso,ALU_resulto,read_data2o,instructionexmem,
					  output logic zeroo,branchx,MemReadx,MemWritex,
					  output logic [4:0] dest_rego);

	always @(posedge clk)
	begin
		instructionexmem<=ii;
		typeo <= typei;
		branchx <= Mi[0];
		MemReadx <=Mi[1];
		MemWritex<=Mi[2];
		zeroo <= zeroi;
		branch_taken_addresso <= branch_taken_addressi;
		ALU_resulto <= ALU_resulti;
		read_data2o <= read_data2i;
		dest_rego <= dest_regi;
		WBo <= WBi;		
	end
					  			  
endmodule					  

module regarray_memwb (input logic [31:0] read_datai,ALU_resulti,ii,
						input logic [1:0] WBi,
						input logic [2:0] typei,
						input logic [4:0] dest_regi,
						input logic clk,
						output logic [31:0] read_datao,ALU_resulto,instructionmemwb,
						output  logic RegWritewb,MemtoRegwb,
						output logic [4:0] dest_rego,
						output logic [2:0] typeo);

	always @(posedge clk)
	begin
		instructionmemwb<=ii;
		typeo <= typei;
		read_datao <= read_datai;
		ALU_resulto <= ALU_resulti;
		dest_rego <= dest_regi;
		RegWritewb <= WBi[0];
		MemtoRegwb <= WBi[1];
	end
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


		
  module ff (input logic [31:0] d,
				input logic  clk,reset,en,
				output reg [31:0] q);
		//always @(posedge clk)
		always @(*)
		begin
		if (reset)
		  q<=32'h00000000;
		else if (!en)
			q<=d;
		end
  endmodule		

module MIPS #(parameter n=32)(
  input logic clk,reset,
  output logic [n-1:0] address,  tinstrfetch,tinstrdecode,tinstrexecute,tinstrmemory,tPC,
					alu_result_exmemo,alu_result_memwbo,memory_datao, tinstrwriteback,data1,data2,
  output logic [4:0] source1,source2,odest_idex,odest_exmem,odest_memwb,
  output logic [2:0] control,
  output logic stall,
  output logic [5:0] opcode,
  output logic data_fwd,
  output logic [n-1:0] data,
  output logic [1:0] mux_src1s, mux_src2s,
  output logic [2:0] type_idexs, type_exmems, type_memwbs, 
  output logic [4:0] rt_exmem,rt_memwb,rs_idex, rd_exmem, rd_memwb, rt_idex);
  

  logic [5:0] funct;
  logic [5:0] rs,temp_opcode;
  logic [5:0] rt;
  logic [5:0] rd;
  logic [4:0] ra1,ra2,wa;
  logic [1:0] ALUop;
  logic [2:0] ALU_ctrl;
  logic MemRead,MemWrite;
  logic [31:0] PCidex,ra1_dataidex,ra2_dataidex,instr32sign_extidex,alusrc2,alusrc1,branch_address,branch_address_exmem,oldPC;
  logic [31:0] alu_result_exmem,read_data2_exmem,readdata,readdata_memwb,alu_result_memwb,src1,src2, alu_result_previous, dest_reg_hold;
  logic [4:0] dest_regidex,dest_reg_exmem,dest_reg;
  logic [1:0] WBidex,WBexmem, mux_src2_df, mux_src1_df;
  logic [2:0] Midex;
  logic [3:0] EXidex;
  logic zeroo,branchx,MemReadx,MemWritex,PCsrc2,MemtoRegwb,zero;
  logic RegDst,Jump,Branch,MemtoReg,ALUsrc,RegWrite,stallx;
  logic [n-1:0] newPC,alu_result,tdata,pc_next,instruction_fetch,instruction_execute,instruction_memory,instruction_write_back,PCifid,memory_data,pc_nexti;
  logic [n-1:0] instruction,instr32sign_ext,writeregisterwb,ra1_data,ra2_data,instructionifidf,src1x,src2x;
  logic hit,RegWritewb,equal, data_f;
  logic [2:0] inst_type, type_idex, type_exmem, type_memwb;
  reg [31:0] PC;
  logic [1:0] mux_src1, mux_src2;
  //initial PC = 32'b00000000;
  int count = 50;
initial
begin
Jump=1'b0;
Branch=1'b0;
WBidex[0]=1'b0;
WBexmem[0]=1'b0;
RegWritewb=1'b0;
//PC=32'h00000000;
RegWrite=1'b0;

//dest_regidex=5'b10000;
//dest_reg_exmem=5'b10000;
//dest_reg=5'b10000;
end
  
  
  ff PCff (.d(newPC),.q(PC),.clk(clk),.reset(reset),.en(stallx));

  instr_mem_cache IM (PC, clk, instruction, hit);
  
  adder32 A1 (PC,32'h00000004,pc_next);
  //mux32 MPC (.a(pc_next),.b(PC-4),.sel(stallx),.y(pc_nexti));

   //stall_logic st (.opcode_idex(instruction[31:26]),.reg_writeidex(RegWrite),.reg_writeexmem(WBidex[0]),.reg_writememwb(WBexmem[0]),
	//					.rt_idex(dest_reg),.rt_exmem(dest_regidex),.rt_memwb(dest_reg_exmem),.rs_ifid(instruction[25:21]),.rt_ifid(instruction[20:16]),.stall(stallx));
	
  regarray_ifid IFID (.instructioni(instruction), .PCi(pc_next), .clk(clk), .instructiono(instruction_fetch),.PCo(PCifid));

  assign ra1 = stallx ? 5'b00000:instruction_fetch[25:21];
  assign ra2 = stallx ? 5'b00000:instruction_fetch[20:16];
  assign instr32sign_ext = stallx? 32'h00000000:{{16{instruction_fetch[15]}},instruction_fetch[15:0]};
 // assign temp_opcode = stallx ? 6'b111111 :instruction_fetch [31:26] ;
  //mux5 regmux (instr[20:16],instr[15:11],regdst,wa);
 
    mux32 regupdate (alu_result_memwb, readdata_memwb, MemtoRegwb, memory_data);

  registers reg_datapath (.read_register1(ra1),.read_register2(ra2), .write_register(wa), .write_data(memory_data),
  .RegWrite(RegWritewb),.clk(clk),.read_data1(ra1_data),.read_data2(ra2_data));
  
  control_logic cl (instruction_fetch [31:26],clk, RegDst, Jump, Branch, MemRead, MemtoReg, MemWrite, ALUsrc, RegWrite, ALUop, inst_type );
  //equal_comparator eq (ra1_data,ra2_data,equal)

 
// data_forward df (.type_idex(inst_type), .type_exmem(type_idex), .type_memwb(type_exmem), .rt_idex(instruction_fetch[20:16]),  .mux_src1(mux_src1_df), .mux_src2(mux_src2_df), .data_f(data_f),
	//				.rt_exmem(instruction_execute[20:16]), .rt_memwb(instruction_memory[20:16]), .rs_idex(instruction_fetch[25:21]), .rd_exmem (dest_regidex), 
		//			.rd_memwb(dest_reg_exmem));
  data_forward df (.type_idex(inst_type), .type_exmem(type_idex), .type_memwb(type_exmem), .rt_idex(instruction_fetch[20:16]),  .mux_src1(mux_src1_df), .mux_src2(mux_src2_df), .data_f(data_f),
					.rt_exmem(instruction_execute[20:16]), .rt_memwb(instruction_memory[20:16]), .rs_idex(instruction_fetch[25:21]), .rd_exmem (dest_regidex), 
					.rd_memwb(dest_reg_exmem));

					
 stall_logic st (.opcode_idex(instruction_fetch[31:26]),.reg_writeidex(WBidex[0]),.reg_writeexmem(WBexmem[0]),.reg_writememwb(RegWritewb), .data_f(data_f),
						.rt_idex(dest_regidex),.rt_exmem(dest_reg_exmem),.rt_memwb(wa),.rs_ifid(instruction_fetch[25:21]),.rt_ifid(instruction_fetch[20:16]),.stall(stallx));

//stall_logic st (.opcode_idex(instruction_fetch[31:26]),.reg_writeidex(WBidex[0]),.reg_writeexmem(WBexmem[0]),.reg_writememwb(RegWritewb), .data_f(data_f),
	//					.rt_idex(dest_reg),.rt_exmem(dest_regidex),.rt_memwb(dest_reg_exmem),.rs_ifid(instruction_fetch[25:21]),.rt_ifid(instruction_fetch[20:16]),.stall(stallx));
 
    mux32 ALUsrc2_mux (ra2_data,instr32sign_ext,ALUsrc,alusrc2);//4 input mux

 mux_4 alu1 (.a(alu_result), .b(alu_result_exmem), .c(readdata), .d(ra1_data), .sel(mux_src1_df), .y(src1x));
  mux_4 alu2 (.a(alu_result), .b(alu_result_exmem), .c(readdata), .d(alusrc2), .sel(mux_src2_df), .y(src2x));
  
  assign equal = (src1x == src2x);
  adder32 A2 (PCifid,{instr32sign_ext[29:0],2'b00}, branch_address);

  
    assign PCsrc2= equal & Branch;
 assign dest_reg_hold = (RegDst ? instruction_fetch[15:11]:instruction_fetch[20:16]);
assign dest_reg = stallx ? 5'b00000: dest_reg_hold;
  					

assign instructionifidf = stallx ? {{6{1'b1}},{26{1'b0}}} : instruction_fetch; 
assign newPC =  Jump ? {PCifid[31:28], instruction_fetch[25:0], 2'b00} : (PCsrc2 ? branch_address :  PCifid);



regarray_idex IDEX (.ii(instructionifidf),.RegDst(RegDst),.Jump(Jump),.Branch(Branch),.MemRead(MemRead),.MemtoReg(MemtoReg),.MemWrite(MemWrite), .ALUsrc(ALUsrc),.RegWrite(RegWrite),
                 .ALUop(ALUop),.clk(clk), .instr16_signexti(instr32sign_ext),.PCi(PCifid),.reg_data1i(ra1_data),.reg_data2i(ra2_data), .mux_src2i(mux_src2_df), .mux_src1i(mux_src1_df),
				 .dest_reg(dest_reg),.dest_regidex(dest_regidex),.instructionidex(instruction_execute),  .typei(inst_type), .typeo(type_idex), .mux_src2o(mux_src2), .mux_src1o(mux_src1),
				 .WB(WBidex), .M(Midex), .EX(EXidex), .PCo(PCidex),.reg_data1o(ra1_dataidex),.reg_data2o(ra2_dataidex),.instr16_signexto(instr32sign_extidex),
				 .src1x(src1x),.src2x(src2x),.src1(src1),.src2(src2));
  				 
 // mux32 ALUsrc2_mux (ra2_dataidex,instr32sign_extidex,EXidex[3],alusrc2);//4 input mux
 // assign alusrc1 = ra1_dataidex;
  ALU_control aluctrl (instr32sign_extidex[5:0],clk, EXidex[1:0], ALU_ctrl);
  
//  mux_4 alu1 (.a(alu_result_exmem), .b(alu_result_memwb), .c(readdata_memwb), .d(alusrc1), .sel(mux_src1), .y(src1));
//  mux_4 alu2 (.a(alu_result_exmem), .b(alu_result_memwb), .c(readdata_memwb), .d(alusrc2), .sel(mux_src2), .y(src2));
  


  MIPS_ALU alu (src1,src2,ALU_ctrl,zero,alu_result,clk);
 
 //alu_hold ah (.a(alu_result),.clk,.y(alu_result_previous))
  				 
  regarray_exmem EXMEM (.ii(instruction_execute),.WBi(WBidex), .Mi(Midex), .branch_taken_addressi(branch_address),.ALU_resulti(alu_result),.read_data2i(ra2_dataidex),
				.zeroi(zero),.clk(clk), .dest_regi(dest_regidex), .WBo(WBexmem),.instructionexmem(instruction_memory), .typei(type_idex), .typeo(type_exmem),
				  .branch_taken_addresso(branch_address_exmem),.ALU_resulto(alu_result_exmem),.read_data2o(read_data2_exmem),
				  .zeroo(zeroo),.branchx(branchx),.MemReadx(MemReadx),.MemWritex(MemWritex), .dest_rego(dest_reg_exmem));
	
  
  //end
  datamem dm (clk, MemReadx, MemWritex,alu_result_exmem[6:0],read_data2_exmem, readdata);	

  regarray_memwb MEMWB (.ii(instruction_memory),.read_datai(readdata),.ALU_resulti(alu_result_exmem), .WBi(WBexmem), .dest_regi(dest_reg_exmem), .clk(clk),.instructionmemwb(instruction_write_back),
				  .read_datao(readdata_memwb),.ALU_resulto(alu_result_memwb), .RegWritewb(RegWritewb),.MemtoRegwb(MemtoRegwb), .dest_rego(wa) , .typei(type_exmem), .typeo(type_memwb));



  //datapath dp (PC,RegWrite,MemtoReg,Jump,Branch,RegDst,ALUsrc,clk, MemRead, MemWrite,ALU_ctrl, instruction, newPC, tdata);
 // assign tinstr= instruction;
  //assign tPC = newPC;

  
assign  	address=PC;
assign	data1 = src1;
assign	data2 =src2;

  always @(posedge clk)// the write back stage values will be updated
  begin
  if (count>0)
  
  
	begin
	tPC <= newPC;
 //PC <= newPC;
  data<=alu_result;
  tinstrfetch<= instruction;
  tinstrdecode<= instruction_fetch;
  tinstrexecute<= instruction_execute;
  tinstrmemory<= instruction_memory;
  tinstrwriteback <=instruction_write_back;
  data_fwd<=data_f;
  stall<=PCsrc2;
  count<=count-1;
  type_idexs<=inst_type; type_exmems<=type_idex; type_memwbs<=type_exmem;
  rt_idex <= instruction_fetch[20:16];  mux_src1s <= mux_src1; mux_src2s <= mux_src2; 
  rt_exmem <= instruction_execute[20:16]; rt_memwb<=instruction_memory[20:16]; rs_idex<=instruction_fetch[25:21]; 
  rd_exmem <= dest_regidex;rd_memwb <= dest_reg_exmem;
  
	opcode <=instruction_fetch[31:26];
	source1<=instruction_fetch[25:21];
	source2<= instruction_fetch[20:16];
	control <= ALU_ctrl;
	odest_idex<=dest_regidex;
	odest_exmem<=dest_reg_exmem;
	odest_memwb<=wa;

	
	alu_result_exmemo <= alu_result_exmem;
alu_result_memwbo <= alu_result_memwb;
memory_datao <= memory_data;
	
 end
  end
endmodule

  module mux_4 (input logic [31:0] a,b,c,d,
				input logic [1:0] sel,
				output logic [31:0] y);
		always @(*)
		case (sel)
		0: y <= a;
		1: y <= b;
		2: y <= c;
		3: y <= d;
		default: y <= d;

		endcase
endmodule

// Code your design here
//type 0:reg-reg, 1: alu imm, 2: load, 3:store, 4: branch
// Code your design here
//type 0:reg-reg, 1: alu imm, 2: load, 3:store, 4: branch

module data_forward (input logic [2:0] type_idex,type_exmem,type_memwb,
                     input logic [4:0] rt_idex,rt_exmem,rt_memwb, 
                     rs_idex,rd_exmem, rd_memwb,
					//input logic stall,
					output logic [1:0] mux_src1, mux_src2,
					output logic data_f);
//mux 0:exmem,1: alu_o/p, 2: mem_data		
  
		always @(*)
          begin
            if (((type_idex == 3'b000 || type_idex ==3'b001 || type_idex ==3'b010 || type_idex ==3'b011 || type_idex ==3'b100)) && (type_exmem == 3'b000) && (rd_exmem == rs_idex))
			begin
            mux_src1 = 2'b00;
			//data_f = 1'b1;
			end
            else if (((type_idex == 3'b000 || type_idex ==3'b001 || type_idex ==3'b010 || type_idex ==3'b011 || type_idex ==3'b100)) && (type_memwb == 3'b000) && (rd_memwb == rs_idex))
            begin
			mux_src1 = 2'b01;            
			//data_f = 1'b1;
			end
            else if (((type_idex == 3'b000 || type_idex ==3'b001 || type_idex ==3'b010 || type_idex ==3'b011 || type_idex ==3'b100)) && (type_exmem == 3'b001) && (rt_exmem == rs_idex))
            begin
			mux_src1 = 2'b00;            
			//data_f = 1'b1;
			end
            
            else if (((type_idex == 3'b000 || type_idex ==3'b001 || type_idex ==3'b010 || type_idex ==3'b011 || type_idex ==3'b100)) && (type_memwb == 3'b001) && (rt_memwb == rs_idex))
            begin
			mux_src1 = 2'b01;
			//data_f = 1'b1;
			end
            
            else if ((type_idex == 3'b000 || type_idex ==3'b001 || type_idex ==3'b010 || type_idex ==3'b011 || type_idex ==3'b100) && (type_memwb == 3'b010) && (rt_memwb == rs_idex))
            begin
			mux_src1 = 2'b10;
            //data_f = 1'b1;
			end
			
            else 
			begin
                mux_src1 = 2'b11;            
            //    data_f = 1'b0;
			end
				
            if ((type_idex == 3'b000) 
                     && (type_exmem == 3'b000) && (rd_exmem == rt_idex))
            begin
			mux_src2 = 2'b00;
			//data_f = 1'b1;
			end
           
            else if ((type_idex == 3'b000) 
                     && (type_memwb == 3'b000) && (rd_memwb == rt_idex))
            begin
			mux_src2 = 2'b01;
            //data_f = 1'b1;
			end

            
            else  if ((type_idex == 3'b000) && (type_exmem == 3'b001) && (rt_exmem == rt_idex))
            begin
			mux_src2 = 2'b00;
			//data_f = 1'b1;
			end
            
           else if ((type_idex == 3'b000) 
                    && (type_memwb == 3'b001) && (rt_memwb == rt_idex))
            begin
			mux_src2 = 2'b01;
			//data_f = 1'b1;
			end
            
            else if ((type_idex == 3'b000) 
                     && (type_memwb == 3'b010) && (rt_memwb == rt_idex))
            begin
			mux_src2 = 2'b10;
			//data_f = 1'b1;
			end
			
			else
              mux_src2 = 2'b11;
      end       // data_f =1'b0;
  always @(*)
   data_f = ((mux_src1 == 2'b11) && (mux_src2 == 2'b11))?1'b0:1'b1;

  
 endmodule

module stall_logic(input logic [5:0] opcode_idex,
					  input logic reg_writeidex,reg_writeexmem,reg_writememwb,
                   input logic [4:0] rt_idex,rt_exmem,rt_memwb,rs_ifid,rt_ifid,
				   input logic data_f,
					  output logic stall);
				
logic cond1, cond2;
  //condt1 actually covers 2 instructions
  
  assign cond1 =  ((opcode_idex==6'b000000) && ((rs_ifid == rt_idex && reg_writeidex) || (rs_ifid == rt_exmem && reg_writeexmem) || /*(rs_ifid == rt_memwb&& 	reg_writememwb ) ||*/ (rt_ifid == rt_idex && reg_writeidex) || 
                                                (rt_ifid == rt_exmem && reg_writeexmem) /*|| (rt_ifid == rt_memwb && reg_writememwb)*/));
  
  //for cond2...we need one more opcode_idex like condition like below
  assign cond2 =  ( ((opcode_idex==6'b000100) || (opcode_idex==6'b100000)||(opcode_idex==6'b101000) || (opcode_idex==6'b000000)) &&
                   ((rs_ifid == rt_idex && reg_writeidex) ||
                    (rs_ifid == rt_exmem && reg_writeexmem) /*|| 
                    (rs_ifid == rt_memwb && reg_writememwb)*/) );
	 
 assign stall =  data_f ? 1'b0 : (cond1 || cond2);   
  //assign stall = ((temp == 1'bz) || (temp == 1'bx)) ? 1'b0 : temp; 
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

module alu_hold (input logic [31:0] a,
				input logic clk,
				output logic [31:0] y);
		always @(posedge clk)
			y<=a;
endmodule
  
  