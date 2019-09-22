module control_unit ( Instruction_op , RegDst , Branch , MemRead , MemtoReg , MemWrite , RegWrite ) ;

input wire [5:0] Instruction_op ;
output reg RegDst ; 
output reg Branch ; 
output reg MemRead ; 
output reg MemtoReg ;
output reg MemWrite ;
output reg RegWrite ;

always @ ( Instruction_op )
begin
case( Instruction_op ) 
( 6'b000000) :
	begin
 	      RegDst <= 1'b1 ;  // R-Format
	      Branch <= 1'b0 ;
	      MemRead <= 1'b0 ;
	      MemtoReg <= 1'b0 ;
	      MemWrite <= 1'b0 ;
	      RegWrite <= 1'b1 ;
	end

( 6'b101011) : 
	begin
	      RegDst <= 1'b0 ;  // SW
	      Branch <= 1'b0 ;
	      MemRead <= 1'b0 ;
	      MemtoReg <= 1'b0 ;
	      MemWrite <= 1'b1 ;
	      RegWrite <= 1'b0 ;
	end

(6'b100011) : 
	begin
	      RegDst <= 1'b0 ;  // LW
	      Branch <= 1'b0 ;
	      MemRead <= 1'b1 ;
	      MemtoReg <= 1'b1 ;
	      MemWrite <= 1'b0 ;
	      RegWrite <= 1'b1 ;
	end

(6'b000100) :
	begin
	      RegDst <= 1'b0 ;  // beq
	      Branch <= 1'b1 ;
	      MemRead <= 1'b0 ;
	      MemtoReg <= 1'b0 ;
	      MemWrite <= 1'b0 ;
	      RegWrite <= 1'b0 ;
	end
endcase
end

endmodule

module control_unit_test ;
reg [5:0] Instruction_op ;
wire RegDst ; 
wire Branch ; 
wire MemRead ; 
wire MemtoReg ;
wire MemWrite ;
wire RegWrite ;
control_unit myControlUnit ( Instruction_op , RegDst , Branch , MemRead , MemtoReg , MemWrite , RegWrite ) ;

initial
begin

$monitor ( $time ,,, "regdst:%b , branch:%b , memread:%b , memtoreg:%b , memwrite:%b , regwrite:%b", RegDst,Branch,MemRead,MemtoReg,MemWrite,RegWrite) ;
#5
Instruction_op <= 6'b 000000 ;
#5
Instruction_op <= 6'b 100011 ;

end

endmodule