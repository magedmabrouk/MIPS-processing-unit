
/*module ControlUnit ( Instruction_op , RegDst , Branch , MemRead , MemtoReg , MemWrite , RegWrite, ALUsrc ) ;
//Input
input wire [5:0] Instruction_op ;
//Output 
output reg RegDst ; 
output reg Branch ; 
output reg MemRead ; 
output reg MemtoReg ;
output reg MemWrite ;
output reg RegWrite ;
output reg ALUsrc ;

always @ ( Instruction_op )
begin

if( Instruction_op ==6'b000000) 
	begin
 	      RegDst <= 1'b1 ;  // R-Format
	      Branch <= 1'b0 ;
	      MemRead <= 1'b0 ;
	      MemtoReg <= 1'b0 ;
	      MemWrite <= 1'b0 ;
	      RegWrite <= 1'b1 ;
	      ALUsrc <= 0'b0 ;
	end

else if( Instruction_op == 6'b101011)  
	begin
	      RegDst <= 1'b0 ;  // SW
	      Branch <= 1'b0 ;
	      MemRead <= 1'b0 ;
	      MemtoReg <= 1'b0 ;
	      MemWrite <= 1'b1 ;
	      RegWrite <= 1'b0 ;
	      ALUsrc <= 1'b1;
	end

else if( Instruction_op ==6'b100011) 
	begin
	      RegDst <= 1'b0 ;  // LW
	      Branch <= 1'b0 ;
	      MemRead <= 1'b1 ;
	      MemtoReg <= 1'b1 ;
	      MemWrite <= 1'b0 ;
	      RegWrite <= 1'b1 ;
	      ALUsrc <= 1'b1;
	end

else if( Instruction_op ==6'b000100) 
	begin
	      RegDst <= 1'b0 ;  // beq
	      Branch <= 1'b1 ;
	      MemRead <= 1'b0 ;
	      MemtoReg <= 1'b0 ;
	      MemWrite <= 1'b0 ;
	      RegWrite <= 1'b0 ;
	      ALUsrc <= 1'b1 ;
	end
else
	begin
	      RegDst <= 1'b0 ;  //immediate instructions
	      Branch <= 1'b0 ;
	      MemRead <= 1'b0 ;
	      MemtoReg <= 1'b0 ;
	      MemWrite <= 1'b0 ;
	      RegWrite <= 1'b1 ;
	      ALUsrc <= 1'b1 ;
	end
end

endmodule*/
module InstructionFetch (clk,IpInstruction,Zero,OpInstruction,EnableIpToIrregister,Branch,IpPcB,OpPc);
input clk;
//Input
input [31:0] IpInstruction;
input Zero;
input [31:0] IpPcB;
//Extra (this part to make it able to take instructions from user (input))
reg [31:0] Ipc=0;		
input EnableIpToIrregister;				
//Intermediate
reg [31:0] IrMemory [0:1024];
reg [31:0] IpPc=0;
//reg [31:0] ImmediateValue;
//Output
output reg [31:0] OpInstruction;
output reg [31:0] OpPc;
//ControlInput
input Branch;


always @(posedge clk )
begin
if (IpPcB!=0)
IpPc=IpPcB;
OpPc=IpPc;
IpPc=IpPc+1;
if (EnableIpToIrregister==1)
begin
IrMemory[Ipc]=IpInstruction;
Ipc=Ipc+1;
end
if (IrMemory[OpPc]!=0)
begin
OpInstruction=IrMemory[OpPc];
//IpPc=IpPc+1;
//Opc=Opc+1;
end
/*ImmediateValue=OpInstruction[15:0];
if (Branch&&Zero)
IpPc=OpPc+1+ImmediateValue;
Zero=0;
else
IpPc=IpPc;*/
end
initial 
begin

IrMemory [1] = 32'b 000000_00100_00010_00100_00000_100000 ;///add done
//IrMemory [1] = 32'b 001000_00100_00010_0000000000000001 ;//addi done

IrMemory [3] = 32'b 000000_00100_00010_00000_00000_100010 ;//sub done

/*IrMemory [3] = 32'b 000000_00001_00010_00000_00000_100100;//and done 

IrMemory [3] = 32'b 000000_00001_00010_00000_00000_100101 ;//or done
IrMemory [4] = 32'b 000000_00001_00010_00000_00000_100111 ;//nor done

IrMemory [5] = 32'b 101011_00001_00010_00000_00000_100110 ;//xor done 


IrMemory [7] = 32'b 001100_00001_00101_0100100000000100 ;//andi done

IrMemory [8] = 32'b 001110_00001_00010_0100100000000010 ;//xori done

IrMemory [9] = 32'b 001101_00001_01000_0000000000010100 ;//ori done
IrMemory [0] = 32'b 101011_00000_00101_0100100000100100 ;//sw done

IrMemory [1] = 32'b 100011_00000_00101_0100100000100100 ;//lw done*/


end
endmodule

module InstructionDecode (clk,Instruction,WriteData,ReadData1,ReadData2,ReadData_2 , Branch , MemRead , MemtoReg , MemWrite  );
input clk;

//inputs
input signed [31:0] Instruction;
input signed [31:0] WriteData;
//IntermediateRegisters
reg [4:0] ReadRegister1;
reg [4:0] ReadRegister2;
reg [4:0] WriteRegister;
reg signed [31:0] RegisterFile [0:31];
reg RegDst ; 
reg RegWrite ;
reg ALUsrc ;
reg indicator=0;

//outputs
output reg signed [31:0] ReadData1;
output reg signed [31:0] ReadData2;
output reg signed [31:0] ReadData_2;
//ControlOutputs
output reg Branch ; 
output reg MemRead ; 
output reg MemtoReg ;
output reg MemWrite ;

 
always@(Instruction)
begin

	if( Instruction[31:26] ==6'b000000) 
	begin
 	      RegDst <= 1'b1 ;  // R-Format
	      Branch <= 1'b0 ;
	      MemRead <= 1'b0 ;
	      MemtoReg <= 1'b0 ;
	      MemWrite <= 1'b0 ;
	      RegWrite <= 1'b1 ;
	      ALUsrc <= 1'b0 ;
	     indicator<=1;
	end

else if( Instruction[31:26] == 6'b101011)  
	begin
	      RegDst <= 1'b0 ;  // SW
	      Branch <= 1'b0 ;
	      MemRead <= 1'b0 ;
	      MemtoReg <= 1'b0 ;
	      MemWrite <= 1'b1 ;
	      RegWrite <= 1'b0 ;
	      ALUsrc <= 1'b1;
	      indicator<=1;
	end

else if( Instruction[31:26] ==6'b100011) 
	begin
	      RegDst <= 1'b0 ;  // LW
	      Branch <= 1'b0 ;
	      MemRead <= 1'b1 ;
	      MemtoReg <= 1'b1 ;
	      MemWrite <= 1'b0 ;
	      RegWrite <= 1'b1 ;
	      ALUsrc <= 1'b1;
	      indicator <=1;
	end

else if( Instruction[31:26] ==6'b000100) 
	begin
	      RegDst <= 1'b0 ;  // beq
	      Branch <= 1'b1 ;
	      MemRead <= 1'b0 ;
	      MemtoReg <= 1'b0 ;
	      MemWrite <= 1'b0 ;
	      RegWrite <= 1'b0 ;
	      ALUsrc <= 1'b0 ;
		  indicator<=1;
	end
else
	begin
	      RegDst <= 1'b0 ;  //immediate instructions
	      Branch <= 1'b0 ;
	      MemRead <= 1'b0 ;
	      MemtoReg <= 1'b0 ;
	      MemWrite <= 1'b0 ;
	      RegWrite <= 1'b1 ;
	      ALUsrc <= 1'b1 ;
		  indicator<=1;
	end
end
always@(posedge indicator)
begin
	ReadRegister1=Instruction[25:21];
        ReadRegister2=Instruction[20:16];
	ReadData1=RegisterFile[ReadRegister1];
	ReadData_2=RegisterFile[ReadRegister2];

	if (ALUsrc) 
	ReadData2=Instruction[15:0];
	else 
	ReadData2=RegisterFile[ReadRegister2];

	if (RegDst)
	WriteRegister=Instruction[15:11];
	else
	WriteRegister=Instruction[20:16];



	if(RegWrite && WriteData)
	RegisterFile[WriteRegister]=WriteData;
	else
	RegisterFile[WriteRegister]=RegisterFile[WriteRegister];

	indicator=0;
end
initial 
begin

RegisterFile[0]=0;
RegisterFile[1]=25;
RegisterFile[2]=22;
RegisterFile[3]=2;
RegisterFile[4]=3;
RegisterFile[5]=5;
RegisterFile[6]=6;
RegisterFile[7]=28;
RegisterFile[8]=25;
RegisterFile[9]=43;
RegisterFile[10]=35;
RegisterFile[11]=52;
RegisterFile[12]=43;
RegisterFile[13]=75;
RegisterFile[14]=82;
RegisterFile[15]=245;
RegisterFile[16]=334;
RegisterFile[17]=512;
RegisterFile[18]=2343;
RegisterFile[19]=2395;
RegisterFile[20]=222;
RegisterFile[21]=213;
RegisterFile[22]=378;
RegisterFile[23]=567;
RegisterFile[24]=3456;
RegisterFile[25]=505;
RegisterFile[26]=2993;
RegisterFile[27]=2765;
RegisterFile[28]=2542;
RegisterFile[29]=278;
RegisterFile[30]=3986;
RegisterFile[31]=5467;

end
endmodule


module ALU_processor(clk,A,B,op,func,Result,Zero_flag);

input clk;

//Input
input signed [31:0] A,B;
input [5:0] op,func; 
//Output
output reg signed [31:0] Result;
output reg Zero_flag;	


always @(*)
begin
	if(op != 6'b000100)
	begin
		 Result   <= 	(op == 6'b000000 && func == 6'b100000)? (A+B): //ADD
				(op == 6'b000000 && func == 6'b100010)? (A-B): //SUB 
				(op == 6'b000000 && func == 6'b100100)? (A&B): // AND 
				(op == 6'b000000 && func == 6'b100101)? (A|B): // OR	
				(op == 6'b000000 && func == 6'b100111)? !(A|B): // NOR
				(op == 6'b000000 && func == 6'b100110)? (A^B): // XOR
				(op == 6'b000000 && func == 6'b101010)? ( (A < B )? 1'b1 : 1'b0 ): //SLT
				(op == 6'b001000)? (A+B): //ADDI
				(op == 6'b001100)? (A&B): //ANDI
				(op == 6'b001110)? (A^B): //XORI
				(op == 6'b001101)? (A|B): //ORI
				(op == 6'b100011)? (A+B): //LW
				(op == 6'b101011)? (A+B): //SW
				(op == 6'b001010)? ( (A < B )? 1'b1 : 1'b0 ): 1'bx ; //SLTI
	end
					
	else if	(op == 6'b000100) begin Zero_flag <= (A==B)? 1'b1 : 1'b0 ; end //BQE
	

end

endmodule
module Beq (IpPc,OpPcB,Zero,Branch,OpInstruction);
input Zero,Branch;
input [31:0] IpPc;
input signed [31:0] OpInstruction;
reg[31:0] ImmediateValue;
output reg [31:0] OpPcB=0;
always @(*)
begin
ImmediateValue=OpInstruction[15:0];
if (Zero&&Branch)
OpPcB=IpPc+1+ImmediateValue;
else
OpPcB=0;
end
endmodule

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
always @ ( * )
	begin
		if ( memwrite_enable == 1 ) 
			
				Data_memory [address] <= write_data ;
			
		if ( memread_enable == 1 )
			
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


module Processor_test;
//ControlUnit******begin*****//na2s 3 signals
// ControlUnit_InstructionFetch*******begin********
wire Branch;
//ControlUnit_InstructionFetch ******end******* // na2s Alusrc
// ControlUnit_DataMemory*******begin********
wire Memread;
wire Memwrite;
// ControlUnit_DataMemory*******end********
// ControlUnit_InstructionDecode*******begin********

// ControlUnit_InstructionDecode*******end********
// ControlUnit_WriteBack*******begin
wire MemtoReg;
// ControlUnit_WriteBack*******end
//ControlUnit******end*****
//*****************************************************************************************************************************************************************************


//InstructionFetch******begin*****
reg clk=0;
reg signed[31:0] IpInstruction;
//InstructionAlu_Fetch ******begin******
wire Zero;
//InstructionAlu_Fetch ******end******
//InstructionFetch_Decode_ControlUnit ******begin*******
wire signed [31:0]OpInstruction;
//InstructionFetch_Decode_ControlUnit *****end*****
reg EnableIpToIrregister;
wire [31:0] OpPcB;
wire [31:0] OpPc;
//InstructionFetch*****end*****
//****************************************************************************************************************************************************************************


//InstructionDecode******begin*****
//InstructionDecode_Alu******begin*****
wire signed [31:0]ReadData1;
wire signed [31:0]ReadData2;
wire EndOfDecoding;
//InstructionDecode_Alu******end*****
//InstructionDecode_Wb******begin*****
wire signed [31:0] ReadData_2;
//InstructionDecode_Alu******end*****
//InstructionDecode******end*****
//******************************************************************************************************************************************************************************
//Alu*******begin
//alu_Dmemory*****begin
wire signed [31:0] AluResult ;
//alu_Dmemory*****end
//Alu********end

//DataMemory********begin
//DataMemory_WriteBack********begin
wire signed [31:0] read_data;
//DataMemory_WriteBack********end
//DataMemory*******end
//****************************************************************************************************************************************************************************

//WriteBackStage******begin*****
//WriteBackStage_InstructionDecode******begin*****
wire signed [31:0] WriteData;
//WriteBackStage_InstructionDecode******end*****
//WriteBackStage******end*****
//****************************************************************************************************************************************************************************
reg [5:0] PositiveEdgeOfClkCounter=0;


InstructionFetch  F1	(clk,IpInstruction,Zero,OpInstruction,EnableIpToIrregister,Branch,OpPcB,OpPc);
InstructionDecode D1	 (clk,OpInstruction,WriteData,ReadData1,ReadData2,ReadData_2 , Branch , MemRead , MemtoReg , MemWrite );
ALU_processor	  A1	(clk,ReadData1,ReadData2,OpInstruction[31:26],OpInstruction[5:0],AluResult,Zero);
Beq 		  B1	(OpPc,OpPcB,Zero,Branch,OpInstruction);
DataMemory 	  DM1	(clk ,  MemWrite ,  MemRead ,  AluResult ,  ReadData_2 , read_data ) ;
WriteBackStage	  WBS1	(clk,read_data,AluResult,MemtoReg,WriteData);
always
#2
clk=~clk;
always@(posedge clk)
PositiveEdgeOfClkCounter=PositiveEdgeOfClkCounter+1;
initial
begin
$monitor("PositiveEdgeOfClkCounter= %d  AluResult=%d  WriteData=%d  OpInstruction= %d  ReadData1=%d ReadData2=%d ReadData_2=%d",PositiveEdgeOfClkCounter,AluResult,WriteData,OpInstruction,ReadData1,ReadData2,ReadData_2);
end
endmodule