use thiserror::Error;

#[derive(Error, Debug)]
pub enum Error {
    #[error("Compressed 16 bit instruction not yet supported")]
    CompressedInstruction,
    #[error("Invalid opcode: {0:b}")]
    InvalidOpcode(u32),
    #[error("Invalid funct: {0:b}")]
    InvalidFunct(u32),
    #[error("Out of memory bounds")]
    OutOfMem,
    #[error("Cannot load to zero register")]
    LoadToZero,
}

struct Rtype {
    rs1: u32,
    rs2: u32,
    rd: u32,
    funct3: u32,
    funct7: u32,
}
struct R4type {
    rs1: u32,
    rs2: u32,
    rs3: u32,
    rd: u32,
    funct3: u32,
    funct2: u32,
}
struct Itype {
    rs1: u32,
    rd: u32,
    funct3: u32,
    imm: u32,
}
struct Stype {
    rs1: u32,
    rs2: u32,
    imm: u32,
    funct3: u32,
}
struct SBtype {
    rs1: u32,
    rs2: u32,
    imm: u32,
    funct3: u32,
}
struct Utype {
    rd: u32,
    imm: u32,
}
struct UJtype {
    rd: u32,
    imm: u32,
}
const fn sign_extend(value: u32, sign_bit: u32) -> u32 {
    if value & (1 << sign_bit) != 0 {
        value | !((1 << (sign_bit + 1)) - 1)
    } else {
        value
    }
}
fn rtype(instruction: u32) -> Rtype {
    Rtype {
        rs1: (instruction >> 15) & 0b11111,
        rs2: (instruction >> 20) & 0b11111,
        rd: (instruction >> 7) & 0b11111,
        funct3: (instruction >> 12) & 0b111,
        funct7: instruction >> 25,
    }
}
fn r4type(instruction: u32) -> R4type {
    R4type {
        rs1: (instruction >> 15) & 0b11111,
        rs2: (instruction >> 20) & 0b11111,
        rs3: instruction >> 27,
        rd: (instruction >> 7) & 0b11111,
        funct3: (instruction >> 12) & 0b111,
        funct2: (instruction >> 25) & 0b11,
    }
}
fn itype(instruction: u32) -> Itype {
    Itype {
        rs1: (instruction >> 15) & 0b11111,
        rd: (instruction >> 7) & 0b11111,
        funct3: (instruction >> 12) & 0b111,
        imm: sign_extend(instruction >> 20, 11),
    }
}
fn stype(instruction: u32) -> Stype {
    Stype {
        rs1: (instruction >> 15) & 0b11111,
        rs2: (instruction >> 20) & 0b11111,
        imm: sign_extend(
            ((instruction >> 7) & 0b11111) | (instruction >> 25) << 5,
            11,
        ),
        funct3: (instruction >> 12) & 0b111,
    }
}
fn sbtype(instruction: u32) -> SBtype {
    SBtype {
        rs1: (instruction >> 15) & 0b11111,
        rs2: (instruction >> 20) & 0b11111,
        imm: sign_extend(
            ((instruction >> 8) & 0b1111) << 1
                | ((instruction >> 25) & 0b111111)
                | ((instruction >> 7) & 1) << 11
                | (instruction >> 31) << 12,
            12,
        ),
        funct3: (instruction >> 12) & 0b111,
    }
}
fn utype(instruction: u32) -> Utype {
    Utype {
        rd: (instruction >> 7) & 0b11111,
        imm: instruction & !((1 << 11) - 1),
    }
}
fn ujtype(instruction: u32) -> UJtype {
    UJtype {
        rd: (instruction >> 7) & 0b11111,
        imm: sign_extend(
            ((instruction >> 12) & 0b11111111) << 12
                | ((instruction >> 20) & 1) << 11
                | ((instruction >> 21) & 0b1111111111) << 1
                | (instruction >> 31) << 20,
            20,
        ),
    }
}
pub struct Vm<'a> {
    registers: [u32; 31], // r0 is zero register
    pc: u32,
    memory: &'a mut [u8],
}
impl<'a> Vm<'a> {
    pub fn new(memory: &'a mut [u8], entry_addr: u32) -> Vm {
        Vm {
            registers: [0; 31],
            pc: entry_addr,
            memory,
        }
    }
    fn set_reg(&mut self, index: u32, value: u32) {
        if index == 0 {
            return;
        }
        self.registers[index as usize - 1] = value;
    }
    fn get_reg(&self, index: u32) -> u32 {
        if index == 0 {
            return 0;
        }
        self.registers[index as usize - 1]
    }
    pub fn run(&mut self) -> Result<(), Error> {
        while self.memory.len() <= self.pc as usize + size_of::<u32>() {
            let instruction = unsafe {
                self.memory
                    .as_ptr()
                    .cast::<u32>()
                    .byte_add(self.pc as usize)
                    .read_unaligned()
            };

            if instruction & 0b11 != 0b11 {
                return Err(Error::CompressedInstruction);
            }
            self.pc += size_of::<u32>() as u32;

            let opcode = (instruction >> 2) & 0b11_1111;
            match opcode {
                LOAD => {
                    let Itype {
                        rs1,
                        rd,
                        funct3,
                        imm,
                    } = itype(instruction);

                    let offset = self.get_reg(rs1).wrapping_add(imm);
                    if offset as usize + size_of::<u32>() > self.memory.len() {
                        return Err(Error::OutOfMem);
                    }
                    let memory_ptr = unsafe { self.memory.as_ptr().byte_add(offset as usize) };

                    let value = unsafe {
                        match funct3 {
                            FUNCT_LB => memory_ptr.cast::<i8>().read_unaligned() as u32,
                            FUNCT_LH => memory_ptr.cast::<i16>().read_unaligned() as u32,
                            FUNCT_LW => memory_ptr.cast::<u32>().read_unaligned(),
                            FUNCT_LBU => memory_ptr.cast::<u8>().read_unaligned() as u32,
                            FUNCT_LHU => memory_ptr.cast::<u16>().read_unaligned() as u32,
                            _ => return Err(Error::InvalidFunct(funct3)),
                        }
                    };

                    self.set_reg(rd, value);

                    println!("load r{} <- [{:x}] = {:x}", rd, offset, value);
                }
                STORE => {
                    let Stype {
                        rs1,
                        rs2,
                        imm,
                        funct3,
                    } = stype(instruction);

                    let offset = self.get_reg(rs1).wrapping_add(imm);
                    if offset as usize + size_of::<u32>() > self.memory.len() {
                        return Err(Error::OutOfMem);
                    }
                    let memory_ptr = unsafe { self.memory.as_mut_ptr().byte_add(offset as usize) };

                    let value = self.get_reg(rs2);

                    unsafe {
                        match funct3 {
                            FUNCT_SB => memory_ptr.cast::<u8>().write_unaligned(value as u8),
                            FUNCT_SH => memory_ptr.cast::<u16>().write_unaligned(value as u16),
                            FUNCT_SW => memory_ptr.cast::<u32>().write_unaligned(value),
                            _ => return Err(Error::InvalidFunct(funct3)),
                        }
                    }

                    println!("store [{:x}] <- r{} = {:x}", offset, rs2, value);
                }
                LUI => {
                    let Utype { rd, imm } = utype(instruction);
                    self.set_reg(rd, imm);

                    println!("lui r{} <- {:x}", rd, self.get_reg(rd));
                }
                AUIPC => {
                    let Utype { rd, imm } = utype(instruction);
                    self.set_reg(rd, self.pc.wrapping_add(imm));

                    println!("auipc r{} <- {:x}", rd, self.get_reg(rd));
                }
                JAL => {
                    let UJtype { rd, imm } = ujtype(instruction);
                    self.set_reg(rd, self.pc + 4);
                    self.pc = self.pc.wrapping_add(imm);

                    println!("jal pc <- {:x}; r{} <- {:x}", self.pc, rd, self.get_reg(rd));
                }
                JALR => {
                    let Itype {
                        rs1,
                        rd,
                        funct3,
                        imm,
                    } = itype(instruction);

                    self.set_reg(rd, self.pc + 4);
                    self.pc = self.get_reg(rs1).wrapping_add(imm as u32);

                    if funct3 != 0b000 {
                        return Err(Error::InvalidFunct(funct3));
                    }
                    println!(
                        "jalr pc <- {:x}; r{} <- {:x}",
                        self.pc,
                        rd,
                        self.get_reg(rd)
                    );
                }
                _ => return Err(Error::InvalidOpcode(instruction)),
            }
        }

        Ok(())
    }
}

const FUNCT_LB: u32 = 0b000;
const FUNCT_LH: u32 = 0b001;
const FUNCT_LW: u32 = 0b010;
const FUNCT_LBU: u32 = 0b100;
const FUNCT_LHU: u32 = 0b101;

const FUNCT_SB: u32 = 0b000;
const FUNCT_SH: u32 = 0b001;
const FUNCT_SW: u32 = 0b010;

const LOAD: u32 = 0b00_000;
const LOAD_FP: u32 = 0b00_001;
const MISC_MEM: u32 = 0b00_011;
const OP_IMM: u32 = 0b00_100;
const AUIPC: u32 = 0b00_101; // Add Upper Immediate to PC
const OP_IMM32: u32 = 0b00_110;

const STORE: u32 = 0b01_000;
const STORE_FP: u32 = 0b01_001;
const AMO: u32 = 0b01_010; // Atomic Memory Operation
const OP: u32 = 0b01_100;
const LUI: u32 = 0b01_101;
const OP32: u32 = 0b01_110;

const MADD: u32 = 0b10_000;
const MSUB: u32 = 0b10_001;
const NMSUB: u32 = 0b10_010;
const NMADD: u32 = 0b10_011;
const OP_FP: u32 = 0b10_100;

const BRANCH: u32 = 0b11_000;
const JUMP: u32 = 0b11_001;
const JALR: u32 = 0b11_010;
const JAL: u32 = 0b11_011;
const SYSTEM: u32 = 0b11_101;
