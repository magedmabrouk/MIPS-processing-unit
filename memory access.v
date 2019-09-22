module Dmemory ( input clk , input memwrite_enable , input memread_enable , input [9:0] address , input [31:0] write_data , output reg [31:0] read_data ) ;

reg [31:0] Data_memory [0:1023] ;
always @ ( posedge clk )
	begin
		if ( memwrite_enable == 1 ) 
			begin
				Data_memory [address] <= write_data ;
			end
		if ( memread_enable == 1 )
			begin 
				read_data <= Data_memory[address] ;
			end
	end
		
endmodule


module memAccess_test ;

reg clk = 1 ;
reg memwrite_enable ; reg memread_enable ;
reg [9:0] address ;
reg [31:0] write_data ; 
wire [31:0] read_data ;

always
begin
#1 clk <= ~clk ;
end

Dmemory myDmemory ( clk , memwrite_enable , memread_enable , address , write_data , read_data ) ;

initial 
begin
#10
memwrite_enable <= 1 ;
address <= 10 ;
write_data = 87 ;
#10
memwrite_enable <= 1 ;
address <= 20 ;
write_data <= 96 ;
#10
memwrite_enable <= 1 ;
address <= 21 ;
write_data <= 33 ;
#10
memwrite_enable <= 0 ;
memread_enable <= 1 ;
address <= 10 ;
#2
$display ( "memory [10] content : %d", read_data );
#10
memread_enable <= 1 ;
address <= 21 ;
#2
$display ( "memory [21] content : %d", read_data );
#10
memread_enable <= 0 ;
memwrite_enable <= 1 ;
address <= 1000 ; 
write_data <= 48 ;
#10
memwrite_enable <= 0 ;
memread_enable <= 1 ;
address <= 1000 ;
#2
$display ( "memory [1000] content : %d", read_data );
end
endmodule