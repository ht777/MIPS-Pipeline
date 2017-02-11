module MIPS_tb();
  logic clk,reset;
  logic [31:0] address,data,tinstrfetch,tinstrdecode,tinstrexecute,tinstrmemory,tPC,tinstrwriteback,data1,data2, alu_result_exmemo,alu_result_memwbo,memory_datao;
  logic [4:0] source1,source2,odest_idex,odest_exmem,odest_memwb;
  logic [2:0] control, type_idexs, type_exmems, type_memwbs;
  logic [5:0] opcode;
  logic  data_fwd;
  logic stall;
  logic [1:0] mux_src1s, mux_src2s;
  logic [4:0] rt_exmem,rt_memwb,rs_idex, rd_exmem, rd_memwb, rt_idex;

  MIPS M (.address(address),.data(data),.clk(clk),.reset(reset),.tinstrfetch(tinstrfetch),.tinstrdecode(tinstrdecode),.tinstrexecute(tinstrexecute),.tinstrmemory(tinstrmemory),.tPC(tPC),.source1(source1),.source2(source2),.control(control),.stall(stall),
  .odest_idex(odest_idex),.odest_exmem(odest_exmem),.odest_memwb(odest_memwb), .data_fwd(data_fwd), .data1(data1), .data2(data2),.alu_result_exmemo(alu_result_exmemo),
  .alu_result_memwbo(alu_result_memwbo), .memory_datao(memory_datao), .opcode(opcode), .tinstrwriteback(tinstrwriteback),.type_idexs(type_idexs), .type_exmems(type_exmems), .type_memwbs(type_memwbs), .mux_src1s(mux_src1s), .mux_src2s(mux_src2s),
  .rt_exmem(rt_exmem), .rt_memwb(rt_memwb), .rs_idex(rs_idex), .rd_exmem(rd_exmem), .rd_memwb(rd_memwb), .rt_idex(rt_idex));
  

  initial
  begin
	clk = 0;reset=1;
	#30 reset = 0;
    end
  always #20 clk=!clk;

	initial  begin
	     $dumpfile ("counter.vcd"); 
	     $dumpvars; 
        end 
	     
        initial  begin
          $display("\t\ttime,\taddress,\tdata,\tclk,\treset,\ttinstrfetch,\ttinstrdecode,\ttinstrexecute,\ttinstrmemory,\ttinstrwriteback,\ttPC,\tsource1,\tsource2,\tcontrol,\tstall,\todest_idex,
		  \todest_exmem,\todest_memwb,\topcode,\tdata_fwd,\ttype_idexs, \ttype_exmems, \ttype_memwbs, \tmux_src1s, \tmux_src2s, \talu_result_exmemo,\talu_result_memwbo,\tmemory_datao,
		  \trt_exmem,\trt_memwb,\trs_idex,\trd_exmem, \trd_memwb, \trt_idex,\tdata1, \tdata2"); 
          $monitor("%d,\t%b,\t%b,\t%b,\t%b,\t%b,\t%b,\t%b,\t%b,\t%b,\t%b,\t%b,\t%b,\t%b,\t%b,\t%b,\t%b,\t%b,\t%b,\t%b,\t%b,\t%b,\t%b,\t%b
		  ,\t%b,\t%b,\t%b,\t%b,\t%b,\t%b,\t%b,\t%b,\t%b,\t%b,\t%b,\t%b",
		  $time,address,data,clk,reset,tinstrfetch,tinstrdecode,tinstrexecute,tinstrmemory,tinstrwriteback,tPC,source1,source2,control,
		  stall,odest_idex,odest_exmem,odest_memwb,opcode, data_fwd, alu_result_exmemo,alu_result_memwbo,memory_datao,
		  type_idexs, type_exmems, type_memwbs, mux_src1s, mux_src2s, rt_exmem,rt_memwb,rs_idex, rd_exmem, rd_memwb, rt_idex, data1, data2,); 
        end
		
 initial 
	    #2000  $finish;
	     /*
       always @(negedge clk)
begin
assert (address==76 && data ==7)
begin
$display ("Simulation completed successfully!!!");
$finish;
end
else
begin
$display ("Simulation failed!!!");
$finish;
end
end
	   */
endmodule



/*
data memory 128 locations , 7 bits

instr mem cache  28 bit tag, 2 bit index(4 blocks) address

case index 
if index =00
if address tag == block0 tag...
else ...bring the block from instr memory


module datapath #(parameter n=32)
  (
    input logic [n-1:0] memory_data, //read data
    input logic regwrite,memtoreg,
    			jump,branch,regdst,alusrc,clk, //instr or data
    output logic  PCSrc2,
    input logic [2:0] alucontrol,
    input logic [4:0] registerwrite_sel,//doubt if needed or not
    output logic [n-1:0] instr,
    output logic [31:0] PC,alu_result,
  );
  
  
module ALU_control (input logic[5:0] funct,
                    input logic clk,
                    input logic [1:0] ALUop,
                    output logic [2:0] ALU_ctrl);
  
  module control_logic(input logic [5:0] opcode,
                     input clk,
                    output logic RegDst,Jump,Branch,MemRead,MemtoReg,MemWrite,
                    ALUsrc,RegWrite,
                     output logic [1:0] ALUop);

   module registers #(parameter n=32)
  (input logic [4:0] read_register1,read_register2, write_register,
   input logic [n-1:0] write_data,
   input logic RegWrite,clk,
   output logic [n-1:0] read_data1,read_data2);
  
  logic [31:0] RAM [2**5-1:0];
  always @(posedge clk)
  if (RegWrite)
    RAM[write_register]<=write_data;
  
  assign read_data1 = RAM[read_register1];
  assign read_data2 = RAM[read_register2];

  
endmodule*/