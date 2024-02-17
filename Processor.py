import os
import argparse

MemSize = 1000 # memory size, in reality, the memory size should be 2^32, but for this lab, for the space resaon, we keep it as this large number, but the memory is still 32-bit addressable.

class InsMem(object):
    def __init__(self, name, ioDir):
        self.id = name       
        with open(ioDir + "\\imem.txt") as im:
            self.IMem = [data.replace("\n", "") for data in im.readlines()]
        while len(self.IMem) < MemSize: 
            self.IMem.append('00000000')

    def readInstr(self, ReadAddress):
        #read instruction memory
        instruction = ''
        for i in range(4): 
            instruction += self.IMem[ReadAddress + i]
        return instruction
          
class DataMem(object):
    def __init__(self, name, ioDir):
        self.id = name
        self.ioDir = ioDir
        with open(ioDir + "\\dmem.txt") as dm:
            self.DMem = [data.replace("\n", "") for data in dm.readlines()]
        while len(self.DMem) < MemSize: 
            self.DMem.append('00000000')

    def readDataMem(self, ReadAddress):
        #read data memory
        if isinstance(ReadAddress, str): 
            ReadAddress = bin_to_int(ReadAddress)
        data = ''
        for i in range(4): 
            data += self.DMem[ReadAddress + i]
        return data
        
    def writeDataMem(self, Address, WriteData):
        # write data into byte addressable memory
        if isinstance(WriteData, int): 
            WriteData = int_to_bin(WriteData)
        if isinstance(Address, str): 
            Address = bin_to_int(Address)
        Data_mem = [WriteData[:8], WriteData[8:16], WriteData[16:24], WriteData[24:]]
        for i in range(4): 
            self.DMem[Address + i] = Data_mem[i]
                     
    def outputDataMem(self):
        resPath = self.ioDir + "\\" + self.id + "_DMEMResult.txt"
        with open(resPath, "w") as rp:
            rp.writelines([str(data) + "\n" for data in self.DMem])

class RegisterFile(object):
    def __init__(self, ioDir):
        self.outputFile = ioDir + "RFResult.txt"
        self.Registers = ['0'*32 for i in range(32)]
    
    def readRF(self, Reg_addr):
        # Read register files
        if isinstance(Reg_addr, str): 
            Reg_addr = bin_to_int(Reg_addr)
        return self.Registers[Reg_addr]
    
    def writeRF(self, Reg_addr, Wrt_reg_data):
        # Write register files
        if isinstance(Wrt_reg_data, int): 
            Wrt_reg_data = int_to_bin(Wrt_reg_data)
        self.Registers[Reg_addr] = Wrt_reg_data
         
    def outputRF(self, cycle):
        op = ["-"*70+"\n", "State of RF after executing cycle:" + "\t" + str(cycle) + "\n"]
        op.extend([val+"\n" for val in self.Registers])
        if(cycle == 0): perm = "w"
        else: perm = "a"
        with open(self.outputFile, perm) as file:
            file.writelines(op)

# implementation of decoder: determine instruction type and operation
class Decoder(object):
    def __init__(self, instruction) -> None:
        assert(len(instruction) == 32)
        self.instr = instruction
        self.funct7 = self.instr[:7]
        self.rs2 = self.instr[7:12]
        self.rs1 = self.instr[12:17]
        self.funct3 = self.instr[17:20]
        self.rd = self.instr[20:25]
        self.opcode = self.instr[25:]
    
    # determine instruction type
    def decode_instr_type(self): 
        opcode = self.opcode
        if opcode == '0110011': 
            return 'R'
        elif opcode in ('0010011','0000011'): 
            return 'I'
        elif opcode == '0100011': 
            return 'S'
        elif opcode == '1100011': 
            return 'B'
        elif opcode == '1101111': 
            return 'J'       
        elif opcode == '1111111': 
            return 'H'
        else: 
            raise Exception('Invalid opcode')
    
    # determine instruction operation
    def decode_instr_operation(self): 
        operation = ''  
        opcode = self.opcode
        funct3 = self.funct3
        funct7 = self.funct7
        rs1 = self.rs1
        rs2 = self.rs2
        rd = self.rd
        instr_type = self.decode_instr_type()

        if instr_type == 'R': 
            if funct7 == '0000000' and funct3 == '000': 
                operation = 'ADD'
            elif funct7 == '0100000' and funct3 == '000': 
                operation = 'SUB'
            elif funct7 == '0000000' and funct3 == '100':
                operation = 'XOR' 
            elif funct7 == '0000000' and funct3 == '110': 
                operation = 'OR'
            elif funct7 == '0000000' and funct3 == '111': 
                operation = 'AND'
            else: 
                raise Exception('Invalid R type instruction')       
        elif instr_type == 'I': 
            if funct3 == '000' and opcode == '0010011': 
                operation = 'ADDI'
            elif funct3 == '000' and opcode == '0000011': 
                operation = 'LW'
            elif funct3 == '100': 
                operation = 'XORI'
            elif funct3 == '110': 
                operation = 'ORI'
            elif funct3 == '111': 
                operation = 'ANDI'
            else: 
                raise Exception('Invalid I type instruction')       
        elif instr_type == 'S': 
            if funct3 == '010': 
                operation = 'SW'
            else: 
                raise Exception('Invalid S type instruction')
        elif instr_type == 'B': 
            if funct3 == '000': 
                operation = 'BEQ'
            elif funct3 == '001': 
                operation = 'BNE'
            else: 
                raise Exception('Invalid B type instruction')
        elif instr_type == 'J':
            operation = 'JAL'
        else: 
            operation = 'HALT'

        return(instr_type,operation,rs2,rs1,rd)

# implementation of the Control Unit
class ControlUnit(object):
    def __init__(self, type, ins) -> None:
        self.Branch = 0
        self.MemRead = 0
        self.MemtoReg = 0
        self.ALUOp = 0
        self.MemWrite = 0
        self.ALUSrc = 0
        self.RegWrite = 0
        self.control(type, ins)
        
    # implementation of the Control logic
    def control(self, type, ins):
        if type == 'R':
            self.ALUSrc = 0
            self.MemtoReg = 0
            self.RegWrite = 1
            self.MemRead = 0
            self.MemWrite = 0
            self.Branch = 0
            self.ALUOp = 0b10        
        elif type == 'I':
            self.ALUSrc = 1
            self.MemtoReg = 0
            self.RegWrite = 1
            self.MemRead = 0
            self.MemWrite = 0
            self.Branch = 0
            self.ALUOp = 0b10
            if ins == 'LW': 
                self.MemRead = 1
                self.MemtoReg = 1
        elif type == 'S':
            self.ALUSrc = 1
            self.RegWrite = 0
            self.MemRead = 0
            self.MemWrite = 1
            self.Branch = 0
            self.ALUOp = 0b00
        elif type == 'B': 
            self.ALUSrc = 0
            self.RegWrite = 0
            self.MemRead = 0
            self.MemWrite = 0
            self.Branch = 1
            self.ALUOp = 0b01
        elif type == 'J':
            self.ALUSrc = 0
            self.MemtoReg = 0
            self.RegWrite = 1
            self.MemRead = 0
            self.MemWrite = 0
            self.Branch = 1
            self.ALUOp = 0b10
    
    # implementation of the stall logic
    def stall(self): 
        self.Branch = 0
        self.MemRead = 0
        self.MemtoReg = 0
        self.ALUOp = 0
        self.MemWrite = 0
        self.ALUSrc = 0
        self.RegWrite = 0

class State(object):
    def __init__(self):
        self.IF = {"nop": False, "PC": 0, 'PCSrc': 0, 'PCWrite': 1, 'Flush': 0}
        self.ID = {"nop": False, 'PC': 0, "Instr": '0', "Rs1": 0, "Rs2": 0, "Rd": 0,}
        self.EX = {"nop": False, 'Ins': '', "Read_data1": 0, "Read_data2": 0, "Imm": 0, "Rs1": 0, "Rs2": 0, "Rd": 0, 'funct7': '', 'funct3': '', 'opcode': '', 'ALUoutput': '0', "Branch": 0, "MemRead": 0, "MemtoReg": 0, "ALUOp": 0, 'MemWrite': 0, 'ALUSrc': 0, 'RegWrite': 0}
        self.MEM = {"nop": False, "ALUoutput": 0, 'Read_data2': 0, 'Load_data': 0, "Rs1": 0, "Rs2": 0, 'Rd': 0, "MemtoReg": 0, "MemRead": 0, "MemWrite": 0, "RegWrite": 0}
        self.WB = {"nop": False, "ALUoutput": 0, 'Write_data': 0, "Rs1": 0, "Rs2": 0, "Rd": 0, "RegWrite": 0, 'MemtoReg': 0}

class Core(object):
    def __init__(self, ioDir, imem, dmem):
        self.myRF = RegisterFile(ioDir)
        self.cycle = 0
        self.halted = False
        self.ioDir = ioDir
        self.state = State()
        self.nextState = State()
        self.ext_imem = imem
        self.ext_dmem = dmem
        
class SingleStageCore(Core):
    def __init__(self, ioDir, imem, dmem):
        super(SingleStageCore, self).__init__(ioDir + "\\SS_", imem, dmem)
        self.opFilePath = ioDir + "\\StateResult_SS.txt"

    def step(self):
        # fetch the instruction from memory
        PC = self.state.IF['PC']
        instr = self.ext_imem.readInstr(PC)

        # decode the instruction - type and operation
        Decode_instr = Decoder(instr)
        opcode = Decode_instr.opcode
        funct7 = Decode_instr.funct7
        funct3 = Decode_instr.funct3
        type, ins, rs2_raw, rs1_raw, rd_raw = Decode_instr.decode_instr_operation()
        imm_raw = imm_gen(instr, type)
        rs2 = int(rs2_raw, 2)
        rs1 = int(rs1_raw, 2)
        rd = int(rd_raw, 2)
        imm = bin_to_int(imm_raw)
        rs1_data_raw = self.myRF.readRF(rs1)
        rs2_data_raw = self.myRF.readRF(rs2)
        if type == 'J': 
            rs1_data_raw = int_to_bin(PC)
            rs2_data_raw = int_to_bin(4)
        if type == 'H': 
            self.state.IF['nop'] = True
            self.state.ID['nop'] = True
            self.state.EX['nop'] = True
            self.state.MEM['nop'] = True
            self.state.WB['nop'] = True          
        self.state.ID['Instr'] = instr

        # perform the instruction operation and determine the address
        control_result = ControlUnit(type, ins)
        ALU_control_result = ALU_control(opcode, funct7, funct3, control_result.ALUOp)
        input2_MUX = self.MUX_exec(rs2_data_raw, imm_raw, control_result.ALUSrc)
        ALU_result = ALU(ALU_control_result, ins, rs1_data_raw, input2_MUX)

        # perform branch operation
        ALU_zero = ALU_result
        if ins == 'BEQ': 
            ALU_zero = ALU_result == '0' * 32
        elif ins == 'BNE': 
            ALU_zero = ALU_result != '0' * 32
        PCsrc = control_result.Branch and ALU_zero
        self.nextState.IF['PC'] = self.MUX_branch(PC + 4, PC + imm, PCsrc)
        self.state.IF['PC'] = self.nextState.IF['PC']
        self.state.EX['Read_data1'] = rs1_data_raw
        self.state.EX['Read_data2'] = rs2_data_raw
        self.state.EX['Imm'] = imm_raw
        self.state.EX['Rs1'] = rs1
        self.state.EX['Rs2'] = rs2
        self.state.EX['Rd'] = rd
        self.state.EX['Branch'] = control_result.Branch
        self.state.EX['MemRead'] = control_result.MemRead
        self.state.EX['MemtoReg'] = control_result.MemtoReg
        self.state.EX['ALUOp'] = control_result.ALUOp
        self.state.EX['MemWrite'] = control_result.MemWrite
        self.state.EX['ALUSrc'] = control_result.ALUSrc
        self.state.EX['RegWrite'] = control_result.RegWrite

        # perform the data memory operations
        lw_value = 0
        if control_result.MemWrite: 
            self.ext_dmem.writeDataMem(ALU_result, rs2_data_raw)
        elif control_result.MemRead: 
            lw_value = self.ext_dmem.readDataMem(ALU_result)     
        wb_value = self.MUX_write(ALU_result, lw_value, control_result.MemtoReg)
        self.state.MEM['ALUoutput'] = ALU_result
        self.state.MEM['Store_data'] = rs2
        self.state.MEM['Rs1'] = rs1
        self.state.MEM['Rs2'] = rs2
        self.state.MEM['Rd'] = rd
        self.state.MEM['MemtoReg'] = control_result.MemtoReg
        self.state.MEM['MemRead'] = control_result.MemRead
        self.state.MEM['MemWrite'] = control_result.MemWrite
        self.state.MEM['RegWrite'] = control_result.RegWrite
        
        # write the result to a register
        if control_result.RegWrite and rd != 0: 
            self.myRF.writeRF(rd, wb_value)
        self.state.WB['Write_data'] = wb_value
        self.state.WB['Rs1'] = rs1
        self.state.WB['Rs2'] = rs2
        self.state.WB['Rd'] = rd
        self.state.WB['RegWrite'] = control_result.RegWrite
        
        if self.state.IF["nop"]:
            self.state.IF['PC'] = PC
            self.halted = True             
        
        self.myRF.outputRF(self.cycle) # dump RF
        self.printState(self.nextState, self.cycle) # print states after executing cycle 0, cycle 1, cycle 2 ...            
        self.state = self.nextState #The end of the cycle and updates the current state with the values calculated in this cycle
        self.cycle += 1
        
        if self.halted == True:
            self.myRF.outputRF(self.cycle)
            self.printState(self.nextState, self.cycle)
            self.cycle += 1
        
    # implementation of EX MUX
    def MUX_exec(self, rs2, imm, ALUSrc): 
        if ALUSrc: 
            return imm
        return rs2
        
    # implementation of branch MUX
    def MUX_branch(self, next, branch, logic_bit):
        if logic_bit: 
            return branch
        return next
        
    # implementation of WB MUX
    def MUX_write(self, ALU_result, lw_value, MemtoReg): 
        if MemtoReg: 
            return lw_value
        return ALU_result
    
    def printState(self, state, cycle):
        printstate = ["-"*70+"\n", "State after executing cycle: " + str(cycle) + "\n"]
        printstate.append("IF.PC: " + str(state.IF["PC"]) + "\n")
        printstate.append("IF.nop: " + str(state.IF["nop"]) + "\n")
        # printing other states for reference (ignore if not necessary)
        # printstate.extend(["IF." + key + ": " + str(val) + "\n" for key, val in state.IF.items()])
        # printstate.extend(["ID." + key + ": " + str(val) + "\n" for key, val in state.ID.items()])
        # printstate.extend(["EX." + key + ": " + str(val) + "\n" for key, val in state.EX.items()])
        # printstate.extend(["MEM." + key + ": " + str(val) + "\n" for key, val in state.MEM.items()])
        # printstate.extend(["WB." + key + ": " + str(val) + "\n" for key, val in state.WB.items()])
        
        if(cycle == 0): perm = "w"
        else: perm = "a"
        with open(self.opFilePath, perm) as wf:
            wf.writelines(printstate)

class FiveStageCore(Core):
    def __init__(self, ioDir, imem, dmem):
        super(FiveStageCore, self).__init__(ioDir + "\\FS_", imem, dmem)
        self.opFilePath = ioDir + "\\StateResult_FS.txt"
        self.state.ID['nop'] = True
        self.state.EX['nop'] = True
        self.state.MEM['nop'] = True
        self.state.WB['nop'] = True

    def step(self):
        # --------------------- WB stage ---------------------
        if not self.state.WB['nop']: 
            self.WB()
        
        # --------------------- MEM stage --------------------
        if not self.state.MEM['nop']: 
            self.MEM()
        else: 
            self.nextState.WB['nop'] = True

        # --------------------- EX stage ---------------------
        if not self.state.EX['nop']: 
            forwarding = self.forwarding_unit()
            self.EX(forwarding)
        else: 
            self.nextState.MEM['nop'] = True

        # --------------------- ID stage ---------------------
        if not self.state.ID['nop']:
            self.ID()
        else: 
            self.nextState.EX['nop'] = True
        

        # --------------------- IF stage ---------------------
        if not self.state.IF['nop']: 
            self.IF()
        else: 
            self.nextState.IF['nop'] = True
            self.nextState.ID['nop'] = True

        if self.state.IF["nop"] and self.state.ID["nop"] and self.state.EX["nop"] and self.state.MEM["nop"] and self.state.WB["nop"]:
            self.halted = True
        
        self.myRF.outputRF(self.cycle) # dump RF
        self.printState(self.nextState, self.cycle) # print states after executing cycle 0, cycle 1, cycle 2 ...       
        self.state = self.nextState
        self.nextState = State() #The end of the cycle and updates the current state with the values calculated in this cycle
        self.cycle += 1

    # implementation of the WB stage
    def WB(self):
        ALU_result = self.state.WB['ALUoutput']
        lw_value = self.state.WB['Write_data']
        rd = self.state.WB['Rd']
        RegWrite = self.state.WB['RegWrite']
        MemtoReg = self.state.WB['MemtoReg']
        wb_value = self.MUX_write(ALU_result, lw_value, MemtoReg)
        if RegWrite and rd != 0: 
            self.myRF.writeRF(rd, wb_value)
            
    # implementation of the MEM stage
    def MEM(self):
        rs2_data_raw = self.state.MEM['Read_data2']
        MemWrite = self.state.MEM['MemWrite']
        MemRead = self.state.MEM['MemRead']
        ALU_result = self.state.MEM['ALUoutput']
        lw_value = 0
        if MemWrite: 
            self.ext_dmem.writeDataMem(ALU_result, rs2_data_raw)
        elif MemRead: 
            lw_value = self.ext_dmem.readDataMem(ALU_result)
        self.nextState.WB['nop'] = False
        self.nextState.WB['Rs1'] = self.state.MEM['Rs1']
        self.nextState.WB['Rs2'] = self.state.MEM['Rs2']
        self.nextState.WB['Rd'] = self.state.MEM['Rd']
        self.nextState.WB['RegWrite'] = self.state.MEM['RegWrite']
        self.nextState.WB['MemtoReg'] = self.state.MEM['MemtoReg']
        self.nextState.WB['ALUoutput'] = self.state.MEM['ALUoutput']
        self.state.MEM['Load_data'] = lw_value
        self.nextState.WB['Write_data'] = lw_value
        self.nextState.WB['nop'] = False
        self.nextState.MEM['nop'] = True

    # implementation of the EX stage
    def EX(self, forwarding): 
        forwardA, forwardB = forwarding
        ins = self.state.EX['Ins']
        ALUOp = self.state.EX['ALUOp']
        rs1_data_raw = self.state.EX['Read_data1']
        rs2_data_raw = self.state.EX['Read_data2']
        imm_raw = self.state.EX['Imm']
        funct7 = self.state.EX['funct7']
        funct3 = self.state.EX['funct3']
        opcode = self.state.EX['opcode']
        ALU_control_result = ALU_control(opcode, funct7, funct3, ALUOp)
        input1_raw = self.MUX_ex_a(rs1_data_raw, forwardA)
        inputB_raw = self.MUX_ex_b(rs2_data_raw, forwardB)
        input2_raw = self.MUX_ex_2(inputB_raw, imm_raw)
        ALU_result = ALU(ALU_control_result, ins, input1_raw, input2_raw)
        self.state.EX['ALUoutput'] = ALU_result
        self.nextState.MEM['nop'] = False
        self.nextState.MEM['ALUoutput'] = ALU_result
        self.nextState.MEM['Read_data2'] = inputB_raw
        self.nextState.MEM['Rs1'] = self.state.EX['Rs1']
        self.nextState.MEM['Rs2'] = self.state.EX['Rs2']
        self.nextState.MEM['Rd'] = self.state.EX['Rd']
        self.nextState.MEM['MemRead'] = self.state.EX['MemRead']
        self.nextState.MEM['MemWrite'] = self.state.EX['MemWrite']
        self.nextState.MEM['RegWrite'] = self.state.EX['RegWrite']
        self.nextState.MEM['MemtoReg'] = self.state.EX['MemtoReg']
        self.nextState.MEM['nop'] = False
        self.nextState.EX['nop'] = True

    # implementation of the ID stage
    def ID(self): 
        if not self.state.IF['Flush']: 
            instr = self.state.ID['Instr']
            PC = self.state.ID['PC']
            Decode_instr = Decoder(instr)
            funct7 = Decode_instr.funct7
            funct3 = Decode_instr.funct3
            opcode = Decode_instr.opcode
            type, ins, rs2_raw, rs1_raw, rd_raw = Decode_instr.decode_instr_operation()
            imm_raw = imm_gen(instr, type)
            rs2 = int(rs2_raw, 2)
            rs1 = int(rs1_raw, 2)
            rd = int(rd_raw, 2)
            rs1_data_raw = self.myRF.readRF(rs1)
            rs2_data_raw = self.myRF.readRF(rs2)
            if type == 'J': 
                rs1_data_raw = int_to_bin(PC)
                rs2_data_raw = int_to_bin(4)
            self.state.ID['Rs1'] = rs1
            self.state.ID['Rs2'] = rs2
            self.state.ID['Rd'] = rd

            # determine control unit results
            control_result = ControlUnit(type, ins)
            PCWrite, IF_IDWrite = self.hazard_detection_unit()
            self.state.IF['PCWrite'] = PCWrite
            self.MUX_ctrl(control_result, PCWrite)

            # perform branch operation
            jump = 1
            forwardA, forwardB = self.branch_forward()
            compare1_raw = self.MUX_id_a(rs1_data_raw, forwardA)
            compare2_raw = self.MUX_id_b(rs2_data_raw, forwardB)
            if ins == 'BEQ': 
                jump = compare1_raw == compare2_raw
            elif ins == 'BNE': 
                jump = compare1_raw != compare2_raw
            self.state.IF['PCSrc'] = control_result.Branch and jump
            self.nextState.IF['PC'] = self.MUX_branch(imm_raw, PCWrite)           
            if type != 'H': 
                if type != 'B':
                    self.nextState.EX['nop'] = False
                    self.nextState.EX['Ins'] = ins
                    self.nextState.EX['Read_data1'] = rs1_data_raw
                    self.nextState.EX['Read_data2'] = rs2_data_raw
                    self.nextState.EX['Imm'] = imm_raw
                    self.nextState.EX['Rs1'] = rs1
                    self.nextState.EX['Rs2'] = rs2
                    self.nextState.EX['Rd'] = rd
                    self.nextState.EX['funct3'] = funct3
                    self.nextState.EX['funct7'] = funct7
                    self.nextState.EX['opcode'] = opcode
                    self.nextState.EX['Branch'] = control_result.Branch
                    self.nextState.EX['MemRead'] = control_result.MemRead
                    self.nextState.EX['MemtoReg'] = control_result.MemtoReg
                    self.nextState.EX['ALUOp'] = control_result.ALUOp
                    self.nextState.EX['MemWrite'] = control_result.MemWrite
                    self.nextState.EX['ALUSrc'] = control_result.ALUSrc
                    self.nextState.EX['RegWrite'] = control_result.RegWrite
                else: 
                    self.nextState.EX['nop'] = True

                # perform NOP for stall
                if IF_IDWrite: 
                    pass
                else: 
                    self.nextState.IF['PC'] = self.state.IF['PC']
                    self.nextState.ID = self.state.ID

            else: 
                self.nextState.EX['nop'] = True
                self.nextState.ID['nop'] = True
                self.nextState.IF['nop'] = True
        else: 
            # perform NOP for branch taken
            self.nextState.IF['PC'] = self.state.IF['PC'] + 4
            self.nextState.ID = self.state.ID
            self.nextState.EX['nop'] = True

    # implementation of the IF stage
    def IF(self):
        PC = self.state.IF['PC']
        instr = self.ext_imem.readInstr(PC)

        if self.state.IF['PCWrite']:
            self.nextState.ID['PC'] = PC

        if self.state.ID['nop'] == True: 
            self.nextState.IF['PC'] = PC + 4
        
        if self.state.IF['PCWrite']: 
            self.nextState.ID['Instr'] = instr
        
    # implementation of the WB MUX
    def MUX_write(self, ALU_result, lw_value, MemtoReg): 
        if MemtoReg: 
            return lw_value
        return ALU_result

    # implementation of the EX forwardA MUX
    def MUX_ex_a(self, rs1, forwardA):
        if forwardA == 0b00: 
            return rs1
        elif forwardA == 0b10: 
            return self.state.MEM['ALUoutput']
        elif forwardA == 0b01: 
            return self.state.WB['Write_data']

    # implementation of the EX forwardB MUX
    def MUX_ex_b(self, rs2, forwardB): 
        if forwardB == 0b00: 
            return rs2
        elif forwardB == 0b10: 
            return self.state.MEM['ALUoutput']
        elif forwardB == 0b01: 
            return self.state.WB['Write_data']
   
    # implementation of the EX MUX
    def MUX_ex_2(self, inputB, imm_raw):
        if self.state.EX['ALUSrc']:
            return imm_raw
        return inputB

    # implementation of the control MUX
    def MUX_ctrl(self, control_result, PCWrite): 
        if not PCWrite: 
            control_result.stall()
 
    # implementation of the branch MUX
    def MUX_branch(self, imm_raw, PCWrite): 
        if PCWrite: 
            imm = bin_to_int(imm_raw)
            if self.state.IF['PCSrc']: 
                self.nextState.IF['Flush'] = 1
                return self.state.ID['PC'] + imm
            else: 
                return self.state.IF['PC'] + 4
        return self.state.ID['PC']
  
    # implementation of the ID forwardA MUX
    def MUX_id_a(self, rs1, forwardA):
        if forwardA == 0b00: 
            return rs1
        elif forwardA == 0b10: 
            return self.state.EX['ALUoutput']
        elif forwardA == 0b01: 
            return self.state.MEM['Load_data']

    # implementation of the ID forwardB MUX
    def MUX_id_b(self, rs2, forwardB): 
        if forwardB == 0b00: 
            return rs2
        elif forwardB == 0b10: 
            return self.state.EX['ALUoutput']
        elif forwardB == 0b01: 
            return self.state.MEM['Load_data']
 
    # implementation of branch forwarding
    def branch_forward(self): 
        forwardA = 0
        forwardB = 0
        EX_MEM = self.state.MEM
        ID_EX = self.state.EX
        IF_ID = self.state.ID
        if (ID_EX['RegWrite'] and (ID_EX['Rd'] != 0) and (ID_EX['Rd'] == IF_ID['Rs1'])): 
            forwardA = 0b10
        if (ID_EX['RegWrite'] and (ID_EX['Rd'] != 0) and (ID_EX['Rd'] == IF_ID['Rs2'])):
            forwardB = 0b10
        if EX_MEM['RegWrite'] and (EX_MEM['Rd'] != 0) and not(ID_EX['RegWrite'] and (ID_EX['Rd'] != 0) and (ID_EX['Rd'] == IF_ID['Rs1'])) and (EX_MEM['Rd'] == IF_ID['Rs1']):
            forwardA = 0b01
        if (EX_MEM['RegWrite'] and (EX_MEM['Rd'] != 0) and not(ID_EX['RegWrite'] and (ID_EX['Rd'] != 0) and (ID_EX['Rd'] == IF_ID['Rs2'])) and (EX_MEM['Rd'] == EX_MEM['Rs2'])): 
            forwardB = 0b01       
        return (forwardA, forwardB)

    # implementation of the hazard detection unit
    def hazard_detection_unit(self): 
        ID_EX = self.state.EX
        IF_ID = self.state.ID
        PCWrite = True
        IF_IDWrite = True
        if ID_EX['MemRead'] and ((ID_EX['Rd'] == IF_ID['Rs1']) or (ID_EX['Rd'] == IF_ID['Rs2'])): 
            PCWrite = False
            IF_IDWrite = False
        return (PCWrite, IF_IDWrite)
  
    # implementation of the forwarding unit
    def forwarding_unit(self): 
        forwardA = 0
        forwardB = 0
        EX_MEM = self.state.MEM
        MEM_WB = self.state.WB
        ID_EX = self.state.EX
        if (EX_MEM['RegWrite'] and (EX_MEM['Rd'] != 0) and (EX_MEM['Rd'] == ID_EX['Rs1'])): 
            forwardA = 0b10
        if (EX_MEM['RegWrite'] and (EX_MEM['Rd'] != 0) and (EX_MEM['Rd'] == ID_EX['Rs2'])):
            forwardB = 0b10
        if MEM_WB['RegWrite'] and (MEM_WB['Rd'] != 0) and not(EX_MEM['RegWrite'] and (EX_MEM['Rd'] != 0) and (EX_MEM['Rd'] == ID_EX['Rs1'])) and (MEM_WB['Rd'] == ID_EX['Rs1']):
            forwardA = 0b01
        if (MEM_WB['RegWrite'] and (MEM_WB['Rd'] != 0) and not(EX_MEM['RegWrite'] and (EX_MEM['Rd'] != 0) and (EX_MEM['Rd'] == ID_EX['Rs2'])) and (MEM_WB['Rd'] == ID_EX['Rs2'])): 
            forwardB = 0b01       
        return (forwardA, forwardB)

    def printState(self, state, cycle):
        printstate = ["-"*70+"\n", "State after executing cycle: " + str(cycle) + "\n"]
        printstate.extend(["IF." + key + ": " + str(val) + "\n" for key, val in state.IF.items()])
        printstate.extend(["ID." + key + ": " + str(val) + "\n" for key, val in state.ID.items()])
        printstate.extend(["EX." + key + ": " + str(val) + "\n" for key, val in state.EX.items()])
        printstate.extend(["MEM." + key + ": " + str(val) + "\n" for key, val in state.MEM.items()])
        printstate.extend(["WB." + key + ": " + str(val) + "\n" for key, val in state.WB.items()])

        if(cycle == 0): perm = "w"
        else: perm = "a"
        with open(self.opFilePath, perm) as wf:
            wf.writelines(printstate)

# assign operation values for ALU
AND = 0b0000
OR = 0b0001
ADD = 0b0010
SUB = 0b0110

# implementation of the ALU control 
def ALU_control(opcode, funct7, funct3, ALUop):
    if ALUop == 0b00: 
        return ADD
    elif ALUop == 0b01: 
        return SUB
    elif ALUop == 0b10: 
        if opcode == '1101111':
            return ADD
        elif funct7 == '0100000': 
            return SUB
        elif funct3 == '000': 
            return ADD
        elif funct3 == '111': 
            return AND
        elif funct3 == '110' or funct3 == '100': 
            return OR
        else: 
            return ADD
    else: 
        return ADD

# implementation of the ALU  
def ALU(ALU_control, operation, inputdata1, inputdata2):
    data1 =  bin_to_int(inputdata1)
    data2 = bin_to_int(inputdata2) 
    if ALU_control == ADD: 
        result = (data1 + data2)
    elif ALU_control == SUB: 
        result = (data1 - data2)
    elif ALU_control == AND: 
        result = (data1 & data2)
    elif ALU_control == OR: 
        if operation == 'OR' or operation == 'ORI': 
            result = (data1 | data2)
        else: 
            result = (data1 ^ data2)
    return int_to_bin(result)

# Convert integer to binary bitstring
def int_to_bin(bit: int) ->str: 
    if not isinstance(bit, int):
        if isinstance(bit, str): 
            while len(bit) < 32: 
                bit = '0' + bit 
            return bit
        else: 
            raise Exception('The input is neither a int nor a string')
    if bit < 0 : 
        reverse_bit = -bit - 1
        reverse_bitstr = bin(reverse_bit)[2:]
        bitstr = ''
        for bit in reverse_bitstr: 
            if bit == '1':
                bitstr += '0'
            else: 
                bitstr += '1'

        if len(bitstr) > 32:
            bitstr = bitstr[-32:]
        while len(bitstr) < 32: 
            bitstr = '1' + bitstr
    else: 
        bitstr = bin(bit)[2:]
        if len(bitstr) > 32:
            bitstr = bitstr[-32:]
        
        while len(bitstr) < 32: 
            bitstr = '0' + bitstr
    return bitstr

# Convert binary bitstring to integer
def bin_to_int(bitstr):
    if isinstance(bitstr, int): 
        return bitstr
    integer = int(bitstr, 2)
    bitlen = len(bitstr)
    if (integer & (1 << (bitlen - 1))) != 0:
        integer = integer - (1 << bitlen)
    return integer

# implementation of the immediate generator
def imm_gen(instr, type): 
    imm = '0'
    if type == 'I': 
        imm = instr[:12]
    elif type == 'S': 
        imm = instr[:7] + instr[20:25]
    elif type == 'B': 
        imm = instr[0] + instr[-8] + instr[1:7] + instr[20:24] + '0'
    elif type == 'J':
        imm = instr[0] + instr[12:20] + instr[1:11] + '0'   
    imm = bin_to_int(imm)
    imm = int_to_bin(imm)
    return imm
 
if __name__ == "__main__":
     
    #parse arguments for input file location
    parser = argparse.ArgumentParser(description='RV32I processor')
    parser.add_argument('--iodir', default="", type=str, help='Directory containing the input files.')
    args = parser.parse_args()

    test_case_no = 4
    test_case_path = '\\6913_ProjA_TC - Copy\\TC' + str(test_case_no)
    ioDir = os.path.abspath(args.iodir) + test_case_path
    print("IO Directory:", ioDir)

    imem = InsMem("Imem", ioDir)
    dmem_ss = DataMem("SS", ioDir)
    dmem_fs = DataMem("FS", ioDir)
    
    ssCore = SingleStageCore(ioDir, imem, dmem_ss)
    fsCore = FiveStageCore(ioDir, imem, dmem_fs)

    while(True):
        if not ssCore.halted:
            ssCore.step()
        
        if not fsCore.halted:
            fsCore.step()

        if ssCore.halted and fsCore.halted:
            break
    
    # dump SS and FS data mem.
    dmem_ss.outputDataMem()
    dmem_fs.outputDataMem()

    no_of_instr = ssCore.cycle
    ss_CPI = ssCore.cycle / no_of_instr
    fs_CPI = fsCore.cycle / no_of_instr
    ss_IPC = no_of_instr / ssCore.cycle
    fs_IPC = no_of_instr / fsCore.cycle
    print('Performance Metrics Result for test case {} :'.format(test_case_no))
    print('Number of instructions = {} \n'.format(int(no_of_instr)))
    print('Single stage:\nCycle time = {} \nCycle per instruction = {:.4f} \nInstruction per cycle = {:.4f} \n'.format(ssCore.cycle,ss_CPI,ss_IPC))
    print('Five stage:\nCycle time = {} \nCycle per instruction = {:.4f} \nInstruction per cycle = {:.4f} \n'.format(fsCore.cycle,fs_CPI,fs_IPC))
