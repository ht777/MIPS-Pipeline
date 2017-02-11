module instr_cache_tb();
	logic [31:0] address;
	logic clk;
	logic [31:0] data,watch;
	logic hit;
	
	  instr_mem_cache IMC (.address(address),.data(data),.clk(clk),.hit(hit),.watch(watch));

  initial
  begin
	clk = 0;
	#20 address = 32'h00000000; 
	#165 address = 32'h00000000;
	
    end
	
  always #20 clk=!clk;
  always #40 address=address+4;

	initial  begin
	     $dumpfile ("counter.vcd"); 
	     $dumpvars; 
        end 
	     
        initial  begin
          $display("\t\ttime,\taddress,\tdata,\tclk,\thit,\twatch"); 
          $monitor("%d,\t%h,\t%h,\t%b,\t%b,\t%h",$time,address,data,clk,hit,watch); 
        end
	     
        initial 
	    #1000 $finish; 
endmodule

