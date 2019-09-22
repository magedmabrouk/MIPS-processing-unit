
module DataMemory (  clk ,  memwrite_enable ,  memread_enable ,  address ,  write_data , read_data ) ;
input clk;
//input
input [31:0] address;
input signed [31:0] write_data;
//Control
input memread_enable;
input memwrite_enable;
//Output
output reg signed  [31:0] read_data;
//IntermediateRegister
reg signed[31:0] Data_memory [0:55000] ;
always @ ( posedge clk )
	begin
		if ( memwrite_enable == 1 ) 
			
				Data_memory [address] <= write_data ;
			
		if ( memread_enable == 1  )
			
				read_data <= Data_memory[address] ;
			
	end
initial
begin
Data_memory[0]=5321;
Data_memory[1]=3123;
Data_memory[2]=2315;
Data_memory[3]=3133;
Data_memory[4]=54657;
Data_memory[5]=3897;
Data_memory[6]=56757;
Data_memory[7]=10973;
Data_memory[8]=45675;
Data_memory[9]=5783;
Data_memory[18471]=22;


end
		
endmodule

module MemWBreg ( clk , inMemoryToRegister , outMemoryToRegister , inReadData , outReadData , inAluResult , outAluResult , inWriteRegister , outWriteRegister ) ;

input clk ;
input inMemoryToRegister ;
output reg outMemoryToRegister ;
input signed [31:0] inReadData ;
output reg signed [31:0] outReadData ;
input signed [31:0] inAluResult ;
output reg signed [31:0] outAluResult ;
input [4:0] inWriteRegister ;
output reg [4:0] outWriteRegister ;

always @ ( posedge clk )
begin

#1
outMemoryToRegister <= inMemoryToRegister ;
outReadData <= inReadData ;
outAluResult <= inAluResult ;
outWriteRegister <= inWriteRegister ;

end

endmodule 

module WriteBackStage (clk,ReadData,AluResult,MemoryToRegister,WriteData);
input clk;
//Input
input signed [31:0] ReadData;
input signed [31:0] AluResult;
//ControlInput
input MemoryToRegister;
//Output
output reg signed [31:0] WriteData;

always @(*)
begin
if (MemoryToRegister==0)

WriteData=AluResult;

if (MemoryToRegister==1)

WriteData=ReadData;

end

endmodule 


module memwbreg_test ;

reg memwrite_enable ; //in
reg memread_enable ; //in
reg [31:0] address ; //in
reg signed [31:0] write_data ; //in
wire [31:0] connect1_read_data ;

reg inMemoryToRegister ; //in
wire connect1_outMemoryToRegister ;
wire [31:0] connect2_outReadData ;
reg signed [31:0] inAluResult ; //in
wire [31:0] connect1_outAluResult ;
reg [4:0] inWriteRegister ; //in
wire [4:0] outWriteRegister ; //out
wire [31:0] WriteData ; //out

reg clk = 1 ;

DataMemory mem1 (  clk ,  memwrite_enable ,  memread_enable ,  address ,  write_data , connect1_read_data ) ;
MemWBreg reg1 ( clk , inMemoryToRegister , connect1_outMemoryToRegister , connect1_read_data , connect2_outReadData , inAluResult , connect1_outAluResult , inWriteRegister , outWriteRegister ) ;
WriteBackStage WB1 (clk,connect2_outReadData,connect1_outAluResult,connect1_outMemoryToRegister,WriteData);

always
begin
#1 clk <= ~clk ;
end

initial
begin
#2
memwrite_enable <= 0 ; //in
memread_enable <= 1 ; //in
address <= 1 ; //in
write_data <= 35 ; //in
inMemoryToRegister = 1 ; //in
inAluResult <= 64 ; //in
inWriteRegister <= 11 ; //in
$display ( "write %d in register number : %d ", WriteData ,outWriteRegister ) ;
#2
memwrite_enable <= 1 ; //in
memread_enable <= 0 ; //in
address <= 78 ; //in
write_data <= 17 ; //in
inMemoryToRegister = 0 ; //in
inAluResult <= 69 ; //in
inWriteRegister <= 13 ; //in
$display ( "write %d in register number : %d ", WriteData ,outWriteRegister ) ;
#2
memwrite_enable <= 0 ; //in
memread_enable <= 1 ; //in
address <= 9 ; //in
write_data <= 104 ; //in
inMemoryToRegister = 1 ; //in
inAluResult <= 47 ; //in
inWriteRegister <= 23 ; //in
$display ( "write %d in register number : %d ", WriteData ,outWriteRegister ) ;
#1
$display ( $time ,,, "write %d in register number : %d ", WriteData ,outWriteRegister ) ;

end

endmodule