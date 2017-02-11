module MIPS_tb();
  logic clk,reset;
  logic [31:0] address,data,tinstr,tPC;
  

  MIPS M (.address(address),.data(data),.clk(clk),.reset(reset),.tinstr(tinstr),.tPC(tPC));
  

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
          $display("\t\ttime,\taddress,\tdata,\tclk,\treset,\ttinstr,\ttPC"); 
          $monitor("%d,\t%b,\t%b,\t%b,\t%b,\t%b,\t%b",$time,address,data,clk,reset,tinstr,tPC); 
        end
		
 initial 
	    #1000  $finish;
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