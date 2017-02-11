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
  
  always @(posedge clk)
    begin

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
  end
  
 
  
endmodule