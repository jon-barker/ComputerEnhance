// 8086 instruction decoder

#include <array>
#include <iostream>
#include <fstream>
#include <vector>
#include <cstdint>

const bool COUNT_CYCLES=false;
const bool DECODE=true;
const bool SIMULATE=true;

struct Instruction
{
    uint8_t opcode;
    uint8_t D;
    uint8_t W;
    uint8_t MOD;
    uint8_t REG;
    uint8_t RM;
    uint8_t DL;
    uint8_t DH;
};

struct Bytes {
    uint8_t low;
    uint8_t high;
};

struct Register16 {
    union {
        uint16_t full;
        Bytes bytes;
    };
};

struct CPUState {
    std::array<Register16, 8> registers;
    std::array<Register16, 4> segregisters;
    std::vector<uint8_t> memory;
    uint16_t ip = 0;
    bool zero_flag = false;
    bool sign_flag = false;
    bool parity_flag = false;

    CPUState() : registers{}, memory(64 * 1024, 0) {}
};

bool compute_parity_flag(uint8_t value) {
    value ^= value >> 4;
    value ^= value >> 2;
    value ^= value >> 1;
    return (~value) & 1;  // returns 1 if even parity
}

enum RegisterIndex {
    AX = 0, CX, DX, BX, SP, BP, SI, DI, REGISTER_COUNT
};

enum SegRegisterIndex {
    ES = 0, CS, SS, DS, SEG_REGISTER_COUNT
};

static const char *wideRegisters[] = {"ax", "cx", "dx", "bx", "sp", "bp", "si", "di"};
static const char *byteRegisters[] = {"al", "cl", "dl", "bl", "ah", "ch", "dh", "bh"};
static const char* segmentRegisters[] = { "es", "cs", "ss", "ds" };

std::array<std::string, 256> opcode_to_mnemonic = [] {
    std::array<std::string, 256> arr{};
    arr[0x70] = "jo";
    arr[0x71] = "jno";
    arr[0x72] = "jb";
    arr[0x73] = "jnb";
    arr[0x74] = "je";
    arr[0x75] = "jne";
    arr[0x76] = "jbe";
    arr[0x77] = "ja";
    arr[0x78] = "js";
    arr[0x79] = "jns";
    arr[0x7A] = "jp";
    arr[0x7B] = "jnp";
    arr[0x7C] = "jl";
    arr[0x7D] = "jnl";
    arr[0x7E] = "jle";
    arr[0x7F] = "jg";
    arr[0xE0] = "loopne";
    arr[0xE1] = "loope";
    arr[0xE2] = "loop";
    arr[0xE3] = "jcxz";
    // Add more as needed upto 256 total
    return arr;
}();


std::string registerToString(int value, bool wide)
{
    return wide ? wideRegisters[value] : byteRegisters[value];
}

uint16_t make_u16_from_bytes(uint8_t dh, uint8_t dl) {
    return static_cast<uint16_t>((static_cast<uint16_t>(dh) << 8) | dl);
}

std::string rmToString(uint8_t mod, uint8_t rm, uint8_t dh, uint8_t dl, bool wide)
{
    static const char* ea[] = {
        "bx + si", "bx + di", "bp + si", "bp + di",
        "si", "di", "bp", "bx"
    };
    if (mod == 0b11) return registerToString(rm, wide);
    if (mod == 0b01) {
        int8_t disp = static_cast<int8_t>(dl);
        if (disp == 0)
            return "[" + std::string(ea[rm]) + "]";
        else if (disp < 0)
            return "[" + std::string(ea[rm]) + " - " + std::to_string(std::abs(disp)) + "]";
        else if (disp > 0)
            return "[" + std::string(ea[rm]) + " + " + std::to_string(disp) + "]";
    }
    if (mod == 0b10) {
        int16_t disp = static_cast<int16_t>(make_u16_from_bytes(dh, dl));
        if (disp == 0)
            return "[" + std::string(ea[rm]) + "]";
        else if (disp < 0)
            return "[" + std::string(ea[rm]) + " - " + std::to_string(std::abs(disp)) + "]";
        else if (disp > 0)
            return "[" + std::string(ea[rm]) + " + " + std::to_string(disp) + "]";
    }
    if (mod == 0b00 && rm == 0b110) {
        int16_t data = static_cast<int16_t>(make_u16_from_bytes(dh, dl));
        return "[" + std::to_string(data) + "]";
    }
    return "[" + std::string(ea[rm]) + "]";
}

int decode(const uint8_t* base, const uint8_t* buffer, size_t buffer_size, CPUState& cpu) {
    if (buffer_size < 2) return -1;

    Instruction inst {};
    int size = 2;  // 1-byte opcode + 1-byte modrm
    uint8_t byte0 = buffer[0];
    uint8_t byte1 = buffer[1];

    // MOV instructions
    if ((byte0 & 0b11111100) == 0b10001000) {
        // MOV r/m, r
        inst.D = (byte0 >> 1) & 1;
        inst.W = byte0 & 1;
        inst.MOD = (byte1 >> 6) & 3;
        inst.REG = (byte1 >> 3) & 7;
        inst.RM = byte1 & 7;

        if (inst.MOD == 0b01) { // 8-bit displacement
            if (buffer_size < 3) return -1;
            inst.DL = buffer[2];
            inst.DH = 0;
            size = 3;
        } else if (inst.MOD == 0b10) { // 16-bit displacement
            if (buffer_size < 4) return -1;
            inst.DL = buffer[2];
            inst.DH = buffer[3];
            size = 4;
        } else if (inst.MOD == 0b00 && inst.RM == 0b110) {
            // Special case: direct address
            if (buffer_size < 4) return -1;
            inst.DL = buffer[2];
            inst.DH = buffer[3];
            size = 4;
        } else {
            inst.DL = inst.DH = 0;
            size = 2;
        }
        if (COUNT_CYCLES) {}
        if (DECODE) {
            std::cout << "mov ";
            if (inst.D) {
                std::cout << registerToString(inst.REG, inst.W) << ", "
                << rmToString(inst.MOD, inst.RM, inst.DH, inst.DL, inst.W) << '\n';
                if (SIMULATE) {
                    uint16_t curr = cpu.registers[inst.REG].full;
                    cpu.registers[inst.REG].full = cpu.registers[inst.RM].full;
                    std::cout << " ; " << wideRegisters[inst.REG] << ":0x" << std::hex << curr << "->0x" << cpu.registers[inst.RM].full << std::dec << std::endl;
                } else {
                    std::cout << std::endl;
                }
            } else {
                std::cout << rmToString(inst.MOD, inst.RM, inst.DH, inst.DL, inst.W) << ", "
                << registerToString(inst.REG, inst.W);
                if (SIMULATE) {
                    if (inst.W) {
                        uint16_t curr = cpu.registers[inst.RM].full;
                        cpu.registers[inst.RM].full = cpu.registers[inst.REG].full;
                        std::cout << " ; " << wideRegisters[inst.RM]
                                << ":0x" << std::hex << curr
                                << "->0x" << cpu.registers[inst.RM].full << std::dec << '\n';
                    } else {
                        int dest_reg_index = (inst.RM < 4) ? inst.RM : inst.RM - 4;
                        int src_reg_index  = (inst.REG < 4) ? inst.REG : inst.REG - 4;

                        uint16_t curr = cpu.registers[static_cast<size_t>(dest_reg_index)].full;

                        uint8_t& src = (inst.REG < 4)
                            ? cpu.registers[static_cast<size_t>(src_reg_index)].bytes.low
                            : cpu.registers[static_cast<size_t>(src_reg_index)].bytes.high;

                        uint8_t& dest = (inst.RM < 4)
                            ? cpu.registers[static_cast<size_t>(dest_reg_index)].bytes.low
                            : cpu.registers[static_cast<size_t>(dest_reg_index)].bytes.high;

                        dest = src;

                        std::cout << " ; " << wideRegisters[static_cast<size_t>(dest_reg_index)]
                                << ":0x" << std::hex << curr
                                << "->0x" << cpu.registers[static_cast<size_t>(dest_reg_index)].full
                                << std::dec << '\n';
                    }
                } else {
                    std::cout << std::endl;
                }
            }
        }
    } else if ((byte0 & 0b11110000) == 0b10110000) {
        // MOV r, imm
        inst.W = (byte0 >> 3) & 1;
        inst.REG = byte0 & 7;
        inst.DL = buffer[1];
        if (inst.W) {
            if (buffer_size < 3) return -1;
            inst.DH = buffer[2];
            size = 3;
        }
        if (buffer_size < 2) return -1;
        if (inst.W) {
            int data = static_cast<uint16_t>(static_cast<int16_t>(
                (static_cast<uint16_t>(inst.DH) << 8) | inst.DL));
            std::cout << "mov " << registerToString(inst.REG, inst.W) << ", "
                        << data;
            if (SIMULATE) {
                uint16_t curr = cpu.registers[inst.REG].full;
                cpu.registers[inst.REG].full = static_cast<uint16_t>((static_cast<uint16_t>(inst.DH) << 8) | inst.DL);
                std::cout << " ; " << wideRegisters[inst.REG] << ":0x" << std::hex << curr << "->0x" << cpu.registers[inst.REG].full << std::dec << std::endl;
            } else {
                std::cout << std::endl;
            }
        } else {
            std::cout << "mov " << registerToString(inst.REG, inst.W) << ", "
                    << static_cast<int>(static_cast<int8_t>(inst.DL));
            size = 2;
            if (SIMULATE) {
                int reg_index = inst.REG < 4 ? inst.REG : inst.REG - 4;
                bool is_high = inst.REG >= 4;

                uint8_t& dest = is_high
                    ? cpu.registers[static_cast<size_t>(reg_index)].bytes.high
                    : cpu.registers[static_cast<size_t>(reg_index)].bytes.low;

                uint16_t curr = cpu.registers[static_cast<size_t>(reg_index)].full;
                dest = static_cast<uint8_t>(inst.DL);

                std::cout << " ; " << byteRegisters[inst.REG] << ":0x"
                        << std::hex << static_cast<int>(curr) << "->0x"
                        << static_cast<int>(cpu.registers[static_cast<size_t>(reg_index)].full) << std::dec << '\n';
            } else {
                std::cout << std::endl;
            }
        }
    } else if ((byte0 & 0b11111110) == 0b11000110) {
        // MOV imm, r/m
        inst.W = byte0 & 1;
        inst.MOD = (byte1 >> 6) & 3;
        inst.REG = 0; // REG is not used by MOV_IMM_RM
        inst.RM = byte1 & 7;
        size_t index = 2;
        if (inst.MOD == 0b01) { // 8-bit displacement
            if (buffer_size < index + 1) return -1;
            inst.DL = buffer[index++];
            inst.DH = 0;
        } else if (inst.MOD == 0b10 || (inst.MOD == 0b00 && inst.RM == 0b110)) { // 16-bit displacement
            if (buffer_size < index + 2) return -1;
            inst.DL = buffer[index++];
            inst.DH = buffer[index++];
        } else {
            inst.DL = inst.DH = 0;
        }

        if (buffer_size < index + (inst.W ? 2 : 1)) return -1;
        uint8_t imm_lo = buffer[index++];
        uint8_t imm_hi = inst.W ? buffer[index++] : 0;
        uint16_t imm = make_u16_from_bytes(imm_hi, imm_lo); 

        size = static_cast<int>(index);

        if (inst.W) {
            std::cout << "mov " << rmToString(inst.MOD, inst.RM, inst.DH, inst.DL, inst.W) << ", "
                        << "word " << static_cast<int>(static_cast<int16_t>(imm)) << '\n';
        } else {
            std::cout << "mov " << rmToString(inst.MOD, inst.RM, inst.DH, inst.DL, inst.W) << ", "
                        << "byte " << static_cast<int>(static_cast<int16_t>(imm)) << '\n';
        }
    } else if (byte0 == 0xA1) {
        // mov ax, [imm16]
        if (buffer_size < 3) return -1;
        uint16_t addr = make_u16_from_bytes(buffer[2], buffer[1]);
        std::cout << "mov ax, [" << addr << "]\n";
        size = 3;
    } else if (byte0 == 0xA3) {
        // mov [imm16], ax
        if (buffer_size < 3) return -1;
        uint16_t addr = make_u16_from_bytes(buffer[2], buffer[1]);
        std::cout << "mov [" << addr << "], ax\n";
        size = 3;
    // ADD instructions
    } else if (byte0 == 0x04 || byte0 == 0x05) {
        // ADD AL/AX, imm8/imm16
        inst.W = byte0 & 1;
        if ((inst.W == 0 && buffer_size < 2) || (inst.W == 1 && buffer_size < 3)) return -1;
        if (inst.W) {
            uint16_t imm = make_u16_from_bytes(buffer[2], buffer[1]);
            std::cout << "add ax, " << imm << '\n';
            size = 3;
        } else {
            int8_t imm = static_cast<int8_t>(buffer[1]);
            std::cout << "add al, " << static_cast<int>(imm) << '\n';
            size = 2;
        }
    } else if (byte0 == 0x3C || byte0 == 0x3D) {
        // CMP AL/AX, imm8/imm16
        inst.W = byte0 & 1;
        if ((inst.W == 0 && buffer_size < 2) || (inst.W == 1 && buffer_size < 3)) return -1;
        if (inst.W) {
            uint16_t imm = make_u16_from_bytes(buffer[2], buffer[1]);
            std::cout << "cmp ax, " << imm << '\n';
            size = 3;
        } else {
            int8_t imm = static_cast<int8_t>(buffer[1]);
            std::cout << "cmp al, " << static_cast<int>(imm) << '\n';
            size = 2;
        }
    } else if ((byte0 & 0b11111100) == 0b00000000 ||  // 0x00, 0x01
         (byte0 & 0b11111110) == 0b00000010) {  // 0x02, 0x03
        // ADD r/m, r OR ADD r, r/m
        inst.D = (byte0 >> 1) & 1;
        inst.W = byte0 & 1;
        inst.MOD = (byte1 >> 6) & 3;
        inst.REG = (byte1 >> 3) & 7;
        inst.RM = byte1 & 7;

        if (inst.MOD == 0b01) { // 8-bit displacement
            if (buffer_size < 3) return -1;
            inst.DL = buffer[2];
            inst.DH = 0;
            size = 3;
        } else if (inst.MOD == 0b10) { // 16-bit displacement
            if (buffer_size < 4) return -1;
            inst.DL = buffer[2];
            inst.DH = buffer[3];
            size = 4;
        } else if (inst.MOD == 0b00 && inst.RM == 0b110) {
            // Special case: direct address
            if (buffer_size < 4) return -1;
            inst.DL = buffer[2];
            inst.DH = buffer[3];
            size = 4;
        } else {
            inst.DL = inst.DH = 0;
            size = 2;
        }
        std::cout << "add ";
        if (inst.D) {
            std::cout << registerToString(inst.REG, inst.W) << ", "
            << rmToString(inst.MOD, inst.RM, inst.DH, inst.DL, inst.W) << '\n';
        } else {
            std::cout << rmToString(inst.MOD, inst.RM, inst.DH, inst.DL, inst.W) << ", "
            << registerToString(inst.REG, inst.W) << '\n';
        }
        } else if (byte0 == 0x8E) {
            // MOV Sreg, r/m16
            inst.MOD = (byte1 >> 6) & 0b11;
            inst.REG = (byte1 >> 3) & 0b111;  // Segment register index (0–3)
            inst.RM  = byte1 & 0b111;

            size_t index = 2;

            if (inst.MOD == 0b01) { // 8-bit displacement
                if (buffer_size < index + 1) return -1;
                inst.DL = buffer[index++];
                inst.DH = 0;
            } else if (inst.MOD == 0b10 || (inst.MOD == 0b00 && inst.RM == 0b110)) {
                if (buffer_size < index + 2) return -1;
                inst.DL = buffer[index++];
                inst.DH = buffer[index++];
            } else {
                inst.DL = inst.DH = 0;
            }

            size = static_cast<int>(index);

            std::string source = rmToString(inst.MOD, inst.RM, inst.DH, inst.DL, true);
            std::string dest = segmentRegisters[inst.REG & 0b11];

            if (DECODE) {
                std::cout << "mov " << dest << ", " << source;
                if (SIMULATE) {
                        uint16_t& dest = cpu.segregisters[inst.REG & 0b11].full;
                        uint16_t src = cpu.registers[inst.RM].full;
                        uint16_t curr = dest;
                        dest = src;

                        std::cout << " ; " << segmentRegisters[inst.REG & 0b11]
                                << ":0x" << std::hex << curr
                                << "->0x" << dest << std::dec << '\n';
                } else {
                    std::cout << std::endl;
                }
            }
            } else if (byte0 == 0x8C) {
                // MOV r/m16, Sreg
                inst.MOD = (byte1 >> 6) & 0b11;
                inst.REG = (byte1 >> 3) & 0b111;  // Segment register index (0–3)
                inst.RM  = byte1 & 0b111;

                size_t index = 2;

                if (inst.MOD == 0b01) { // 8-bit displacement
                    if (buffer_size < index + 1) return -1;
                    inst.DL = buffer[index++];
                    inst.DH = 0;
                } else if (inst.MOD == 0b10 || (inst.MOD == 0b00 && inst.RM == 0b110)) {
                    if (buffer_size < index + 2) return -1;
                    inst.DL = buffer[index++];
                    inst.DH = buffer[index++];
                } else {
                    inst.DL = inst.DH = 0;
                }

                size = static_cast<int>(index);

                std::string dest = rmToString(inst.MOD, inst.RM, inst.DH, inst.DL, true);
                std::string source = segmentRegisters[inst.REG & 0b11];

                if (DECODE) {
                    std::cout << "mov " << dest << ", " << source;
                    if (SIMULATE) {
                        uint16_t& dest_reg = cpu.registers[inst.RM].full;
                        uint16_t src_val = cpu.segregisters[inst.REG & 0b11].full;
                        uint16_t curr = dest_reg;
                        dest_reg = src_val;

                        std::cout << " ; " << wideRegisters[inst.RM]
                                << ":0x" << std::hex << curr
                                << "->0x" << dest_reg << std::dec << '\n';
                    } else {
                        std::cout << '\n';
                    }
                }
    } else if (((byte0 & 0b11111100) == 0b10000000) && ((((byte1 >> 3) & 7) == 0b000) || (((byte1 >> 3) & 7) == 0b101) || (((byte1 >> 3) & 7) == 0b111))){
        // Group 1 instructions: ADD/SUB/etc with immediate
        uint8_t S = (byte0 >> 1) & 1;
        uint8_t W = byte0 & 1;

        inst.MOD = (byte1 >> 6) & 3;
        inst.REG = (byte1 >> 3) & 7; 
        inst.RM  = byte1 & 7;

        size_t index = 2;

        if (inst.MOD == 0b01) {  // 8-bit displacement
            if (buffer_size < index + 1) return -1;
            inst.DL = buffer[index++];
            inst.DH = 0;
        } else if (inst.MOD == 0b10 || (inst.MOD == 0b00 && inst.RM == 0b110)) {  // 16-bit displacement
            if (buffer_size < index + 2) return -1;
            inst.DL = buffer[index++];
            inst.DH = buffer[index++];
        } else {
            inst.DL = inst.DH = 0;
        }

        // Now decode the immediate
        int16_t imm = 0;
        if (S == 1 && W == 1) {
            if (buffer_size < index + 1) return -1;
            imm = static_cast<int8_t>(buffer[index++]);  // sign-extend 8-bit to 16-bit
        } else if (W == 1) {
            if (buffer_size < index + 2) return -1;
            uint8_t lo = buffer[index++];
            uint8_t hi = buffer[index++];
            imm = static_cast<int16_t>((hi << 8) | lo);
        } else {
            if (buffer_size < index + 1) return -1;
            imm = static_cast<int8_t>(buffer[index++]);  // 8-bit operand
        }

        size = static_cast<int>(index);

        if (inst.REG == 0b000)
            std::cout << "add ";
        else if (inst.REG == 0b101)
            std::cout << "sub ";
        else if (inst.REG == 0b111)
            std::cout << "cmp ";
        if (inst.MOD != 0b11) {
            std::cout << (W ? "word " : "byte ");
        }
        std::cout << rmToString(inst.MOD, inst.RM, inst.DH, inst.DL, W) << ", " << imm;
        if (SIMULATE) {
            if (SIMULATE && inst.MOD == 0b11) {
                uint16_t curr = cpu.registers[inst.RM].full;
                if (inst.REG == 0b000)
                    cpu.registers[inst.RM].full += imm;
                else if (inst.REG == 0b101)
                    cpu.registers[inst.RM].full -= imm;
                std::cout << " ; " << wideRegisters[inst.RM]
                        << ":0x" << std::hex << curr
                        << "->0x" << cpu.registers[inst.RM].full << std::dec;
                std::cout << " flags:";
                if (cpu.sign_flag) std::cout << "S";
                if (cpu.parity_flag) std::cout << "P";
                if (cpu.zero_flag) std::cout << "Z";
                std::cout << "->";
                cpu.zero_flag = (cpu.registers[inst.RM].full == 0);
                cpu.sign_flag = (cpu.registers[inst.RM].full & 0x8000) != 0;
                uint8_t lsb = cpu.registers[inst.RM].full & 0xFF;
                cpu.parity_flag = compute_parity_flag(lsb);
                if (cpu.sign_flag) std::cout << "S";
                if (cpu.parity_flag) std::cout << "P";
                if (cpu.zero_flag) std::cout << "Z";
                std::cout << '\n';
            }
        } else {
            std::cout << std::endl;
        }
    // SUB instructions
    } else if (byte0 == 0x2C || byte0 == 0x2D) {
        // SUB AL/AX, imm8/imm16
        inst.W = byte0 & 1;
        if ((inst.W == 0 && buffer_size < 2) || (inst.W == 1 && buffer_size < 3)) return -1;
        if (inst.W) {
            uint16_t imm = make_u16_from_bytes(buffer[2], buffer[1]);
            std::cout << "sub ax, " << imm << '\n';
            size = 3;
        } else {
            int8_t imm = static_cast<int8_t>(buffer[1]);
            std::cout << "sub al, " << static_cast<int>(imm) << '\n';
            size = 2;
        }
    } else if ((byte0 & 0b11111100) == 0x28 || (byte0 & 0b11111100) == 0x38) {
        // SUB r/m, r  or  SUB r, r/m
        inst.D = (byte0 >> 1) & 1;
        inst.W = byte0 & 1;
        inst.MOD = (byte1 >> 6) & 3;
        inst.REG = (byte1 >> 3) & 7;
        inst.RM  = byte1 & 7;

        size_t index = 2;
        if (inst.MOD == 0b01) {
            inst.DL = buffer[index++];
            inst.DH = 0;
            size = 3;
        } else if (inst.MOD == 0b10 || (inst.MOD == 0b00 && inst.RM == 0b110)) {
            inst.DL = buffer[index++];
            inst.DH = buffer[index++];
            size = 4;
        } else {
            inst.DL = inst.DH = 0;
            size = 2;
        }

        std::string op;
        if ((byte0 & 0b11111100) == 0x28)
            op = "sub ";
        else
            op = "cmp ";
        if (inst.D) {
            std::cout << op << registerToString(inst.REG, inst.W) << ", "
                      << rmToString(inst.MOD, inst.RM, inst.DH, inst.DL, inst.W) << '\n';
        } else {
            std::cout << op << rmToString(inst.MOD, inst.RM, inst.DH, inst.DL, inst.W) << ", "
                      << registerToString(inst.REG, inst.W);
            if (SIMULATE) {
                    uint16_t lhs = cpu.registers[inst.RM].full;
                    uint16_t rhs = cpu.registers[inst.REG].full;
                    uint16_t result = lhs - rhs;

                    if ((byte0 & 0b11111100) == 0x28) {
                        cpu.registers[inst.RM].full = result;
                    }

                    std::cout << " ; " << rmToString(inst.MOD, inst.RM, inst.DH, inst.DL, inst.W) << ":0x" << std::hex << lhs << "->0x" <<  cpu.registers[inst.RM].full << std::dec;
                    std::cout << " flags:";
                    if (cpu.sign_flag > 0) {
                        std::cout << "S";
                    }
                    if (cpu.parity_flag > 0) {
                        std::cout << "P";
                    }
                    if (cpu.zero_flag > 0) {
                        std::cout << "Z";
                    }
                    cpu.zero_flag = (result == 0);
                    cpu.sign_flag = inst.W
                        ? (result & 0x8000) != 0
                        : ((result & 0x80) != 0);
                    uint8_t lsb = result & 0xFF;
                    cpu.parity_flag = compute_parity_flag(lsb);
                    std::cout << "->";
                    if (cpu.sign_flag > 0) {
                        std::cout << "S";
                    }
                    if (cpu.parity_flag > 0) {
                        std::cout << "P";
                    }
                    if (cpu.zero_flag > 0) {
                        std::cout << "Z";
                    }
                    std::cout << std::endl;
            } else {
                std::cout << std::endl;
            }
        }
    } else if (byte0 == 0x80) {
        // SUB r/m, imm
        inst.W = (byte0 >> 3) & 1;
        inst.REG = byte0 & 7;
        inst.DL = buffer[1];
        if (inst.W) {
            if (buffer_size < 3) return -1;
            inst.DH = buffer[2];
            size = 3;
        }
        if (buffer_size < 2) return -1;
        if (inst.W) {
            int data = static_cast<int16_t>(
                (static_cast<uint16_t>(inst.DH) << 8) | inst.DL);
            std::cout << "sub " << registerToString(inst.REG, inst.W) << ", "
                        << data << '\n';
        } else {
            std::cout << "sub " << registerToString(inst.REG, inst.W) << ", "
                        << static_cast<int>(static_cast<int8_t>(inst.DL)) << '\n';
            size = 2;
        }
    } else if (!opcode_to_mnemonic[byte0].empty()) {
        if (buffer_size < 2) return -1;
        int8_t offset = static_cast<int8_t>(buffer[1]);
        size = 2;
        int current_offset = static_cast<int>(buffer - base);
        int target_address = current_offset + size + offset;
        std::cout << opcode_to_mnemonic[byte0] << " short 0x" << std::hex << target_address << std::dec << '\n';
    } else {
        std::cerr << "Unknown opcode: 0x" << std::hex << static_cast<int>(byte0) << std::dec << '\n';
        return -1;
    }

    return size;
}

int main(int argc, char *argv[])
{
    if (argc != 2)
    {
        std::cerr << "Usage: " << argv[0] << " <file_path>" << '\n';
        return 1;
    }

    std::ifstream file(argv[1], std::ios::binary);

    if (!file)
    {
        std::cerr << "Error opening file: " << argv[1] << '\n';
        return 1;
    }

    // Find the size of the file
    file.seekg(0, std::ios::end);
    std::streamsize fileSize = file.tellg();
    file.seekg(0, std::ios::beg);

    if (fileSize < 0) {
        std::cerr << "Failed to get file size\n";
        return 1;
    }

    // Read the file into a vector
    std::vector<uint8_t> buffer(static_cast<size_t>(fileSize));
    file.read(reinterpret_cast<char*>(buffer.data()), fileSize);
    file.close();

    CPUState cpu;

    std::cout << "bits 16" << '\n';
    std::cout << '\n';
    size_t i = 0;
    while (i < buffer.size()) {
        size_t remaining = buffer.size() - i;
        int instruction_size = decode(buffer.data(), buffer.data() + i, remaining, cpu);
        if (instruction_size <= 0) break;  // error or unknown instruction
        i += static_cast<size_t>(instruction_size);
    }

    std::cout << std::endl;
    if (SIMULATE) {
        std::cout << "Final registers:" << std::endl;
        std::array<RegisterIndex, 8> output_order = {AX, BX, CX, DX, SP, BP, SI, DI};

        for (RegisterIndex reg : output_order) {
            if (cpu.registers[reg].full > 0) {
                std::cout << "      " << wideRegisters[reg]
                        << ": 0x" << std::setfill('0') << std::setw(4)
                        << std::hex << cpu.registers[reg].full
                        << " (" << std::dec << cpu.registers[reg].full << ")\n";
            }
        }

        std::array<SegRegisterIndex, 4> seg_output_order = {ES, SS, DS, CS};

        for (SegRegisterIndex reg : seg_output_order) {
            if (cpu.segregisters[reg].full) {
                std::cout << "      " << segmentRegisters[reg]
                        << ": 0x" << std::setfill('0') << std::setw(4)
                        << std::hex << cpu.segregisters[reg].full
                        << " (" << std::dec << cpu.segregisters[reg].full << ")\n";
            }
        }
        std::cout << "   flags: ";
        if (cpu.sign_flag > 0) {std::cout << "S";}
        if (cpu.parity_flag > 0) {std::cout << "P";}
        if (cpu.zero_flag > 0) {std::cout << "Z";}
        std::cout << std::endl;
    }

    return 0;
}
