// 8086 instruction decoder

#include <iostream>
#include <fstream>
#include <vector>
#include <cstdint>

constexpr int MOV_OPCODE = 34;
constexpr int I2R_OPCODE = 11;

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

int decode(const uint8_t* buffer, size_t buffer_size) {
    if (buffer_size < 1) return -1; // Not enough data

    Instruction inst{};
    int size = 2;  // default for most instructions

    const uint8_t byte0 = buffer[0];
    const uint8_t byte1 = buffer_size > 1 ? buffer[1] : 0;

    if ((byte0 & 0b11111100) == 0b10001000) {
        inst.opcode = (byte0 & 0b11111100) >> 2;
    } else if ((byte0 & 0b11110000) == 0b10110000) {
        inst.opcode = (byte0 & 0b11110000) >> 4;
    } else {
        std::cerr << "Unknown opcode: 0x" << std::hex << static_cast<int>(byte0) << '\n';
        return -1;
    }

    switch (inst.opcode) {
        case MOV_OPCODE:
            if (buffer_size < 2) return -1;
            inst.D = (byte0 >> 1) & 1;
            inst.W = byte0 & 1;
            inst.MOD = (byte1 >> 6) & 3;
            inst.REG = (byte1 >> 3) & 7;
            inst.RM = byte1 & 7;

            std::cout << "mov ";
            uint8_t dst, src;
            if (inst.D) {
                dst = inst.REG;
                src = inst.RM;
            } else {
                dst = inst.RM;
                src = inst.REG;
            }
            std::cout << registerToString(dst, inst.W) << ", "
                      << registerToString(src, inst.W) << '\n';
            break;

        case I2R_OPCODE:
            if (buffer_size < 2) return -1;
            inst.W = (byte0 >> 3) & 1;
            inst.REG = byte0 & 7;
            inst.DL = buffer[1];
            if (inst.W) {
                if (buffer_size < 3) return -1;
                inst.DH = buffer[2];
                size = 3;
                uint16_t data = static_cast<uint16_t>(
                    (static_cast<uint16_t>(inst.DH) << 8) | inst.DL);
                std::cout << "mov " << registerToString(inst.REG, inst.W) << ", "
                          << data << '\n';
            } else {
                std::cout << "mov " << registerToString(inst.REG, inst.W) << ", "
                          << static_cast<int>(static_cast<int8_t>(inst.DL)) << '\n';
                size = 2;
            }
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
