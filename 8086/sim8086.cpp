// 8086 instruction decoder

#include <iostream>
#include <fstream>
#include <vector>
#include <cstdint>

constexpr int MOV_RM_R = 34;
constexpr int MOV_IMM_REG = 11;
constexpr int MOV_IMM_RM = 22;

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

std::string registerToString(int value, bool wide)
{
    static const char *registers[] = {"al", "cl", "dl", "bl", "ah", "ch", "dh", "bh"};
    static const char *wideRegisters[] = {"ax", "cx", "dx", "bx", "sp", "bp", "si", "di"};
    return wide ? wideRegisters[value] : registers[value];
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
        return "[" + std::string(ea[rm]) + " + " + std::to_string(disp) + "]";
    }
    if (mod == 0b10) {
        uint16_t data = static_cast<uint16_t>((static_cast<uint16_t>(dh) << 8) | dl);
        if (data == 0)
            return "[" + std::string(ea[rm]) + "]";
        return "[" + std::string(ea[rm]) + " + " + std::to_string(data) + "]";
    }
    if (mod == 0b00 && rm == 0b110) {
        uint16_t data = static_cast<uint16_t>((static_cast<uint16_t>(dh) << 8) | dl);
        return "[" + std::to_string(data) + "]";
    }
    return "[" + std::string(ea[rm]) + "]";
}

int decode(const uint8_t* buffer, size_t buffer_size) {
        if (buffer_size < 2) return -1;

    Instruction inst {};
    int size = 2;  // 1-byte opcode + 1-byte modrm
    uint8_t byte0 = buffer[0];
    uint8_t byte1 = buffer[1];

    if ((byte0 & 0b11111100) == 0b10001000) {
        inst.opcode = MOV_RM_R;
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
    } else if ((byte0 & 0b11110000) == 0b10110000) {
        // MOV r, imm
        inst.opcode = MOV_IMM_REG;
        inst.W = (byte0 >> 3) & 1;
        inst.REG = byte0 & 7;
        inst.DL = buffer[1];
        if (inst.W) {
            if (buffer_size < 3) return -1;
            inst.DH = buffer[2];
            size = 3;
        }
    } else if ((byte0 & 0b11111110) == 0b11000110) {
        inst.opcode = MOV_IMM_RM;
    } else {
        std::cerr << "Unknown opcode: 0x" << std::hex << static_cast<int>(byte0) << '\n';
        return -1;
    }

    switch (inst.opcode) {
        case MOV_RM_R:
            std::cout << "mov ";
            if (inst.D) {
                std::cout << registerToString(inst.REG, inst.W) << ", "
                << rmToString(inst.MOD, inst.RM, inst.DH, inst.DL, inst.W) << '\n';
            } else {
                std::cout << rmToString(inst.MOD, inst.RM, inst.DH, inst.DL, inst.W) << ", "
                << registerToString(inst.REG, inst.W) << '\n';
            }
            break;

        case MOV_IMM_REG:
            if (buffer_size < 2) return -1;
            inst.W = (byte0 >> 3) & 1;
            inst.REG = byte0 & 7;
            inst.DL = buffer[1];
            if (inst.W) {
                if (buffer_size < 3) return -1;
                inst.DH = buffer[2];
                size = 3;
                int data = static_cast<int16_t>(
                    (static_cast<uint16_t>(inst.DH) << 8) | inst.DL);
                std::cout << "mov " << registerToString(inst.REG, inst.W) << ", "
                          << data << '\n';
            } else {
                std::cout << "mov " << registerToString(inst.REG, inst.W) << ", "
                          << static_cast<int>(static_cast<int8_t>(inst.DL)) << '\n';
                size = 2;
            }
            break;
        case MOV_IMM_RM:
            break;
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

    std::cout << "bits 16" << '\n';
    std::cout << '\n';
    size_t i = 0;
    while (i < buffer.size()) {
        size_t remaining = buffer.size() - i;
        int instruction_size = decode(buffer.data() + i, remaining);
        if (instruction_size <= 0) break;  // error or unknown instruction
        i += static_cast<size_t>(instruction_size);
    }

    return 0;
}
